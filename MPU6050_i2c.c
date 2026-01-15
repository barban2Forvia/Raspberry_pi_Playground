
// mpu6050_udp_nosmbus.c
// MPU-6050 I2C accelerometer reader for Raspberry Pi + UDP sender (no i2c/smbus.h)
// Build: gcc -O3 -march=native -flto mpu6050_udp_nosmbus.c -o mpu6050_udp
// Run  : ./mpu6050_udp

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

// ======== CONFIG ========
static const char *I2C_DEV = "/dev/i2c-1";
#define PC_IP   "192.168.10.20"   // <-- replace with your Ubuntu PC IP
#define PORT    5005

// ======== MPU-6050 REGISTERS ========
#define WHO_AM_I      0x75
#define PWR_MGMT_1    0x6B
#define SMPLRT_DIV    0x19
#define CONFIG_REG    0x1A
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B

// ======== CONSTANTS ========
#define ACCEL_SCALE 0.000598550415  // (9.80665 / 16384.0) precomputed

// ======== GLOBALS ========
static volatile sig_atomic_t g_running = 1;

// ======== SIGNAL HANDLER ========
static void handle_sigint(int sig) {
    (void)sig;
    g_running = 0;
}

// ======== UTILS ========

// Format ISO-8601 UTC with millisecond precision: YYYY-MM-DDTHH:MM:SS.sss+00:00
static inline void iso8601_utc_ms(char *buf, struct timespec *ts) {
    time_t sec = ts->tv_sec;
    struct tm tm_utc;
    gmtime_r(&sec, &tm_utc);
    
    // Manual formatting - faster than strftime + snprintf
    long ms = ts->tv_nsec / 1000000L;
    register int pos = 0;
    buf[pos++] = '0' + (tm_utc.tm_year + 1900) / 1000;
    buf[pos++] = '0' + ((tm_utc.tm_year + 1900) / 100) % 10;
    buf[pos++] = '0' + ((tm_utc.tm_year + 1900) / 10) % 10;
    buf[pos++] = '0' + (tm_utc.tm_year + 1900) % 10;
    buf[pos++] = '-';
    buf[pos++] = '0' + (tm_utc.tm_mon + 1) / 10;
    buf[pos++] = '0' + (tm_utc.tm_mon + 1) % 10;
    buf[pos++] = '-';
    buf[pos++] = '0' + tm_utc.tm_mday / 10;
    buf[pos++] = '0' + tm_utc.tm_mday % 10;
    buf[pos++] = 'T';
    buf[pos++] = '0' + tm_utc.tm_hour / 10;
    buf[pos++] = '0' + tm_utc.tm_hour % 10;
    buf[pos++] = ':';
    buf[pos++] = '0' + tm_utc.tm_min / 10;
    buf[pos++] = '0' + tm_utc.tm_min % 10;
    buf[pos++] = ':';
    buf[pos++] = '0' + tm_utc.tm_sec / 10;
    buf[pos++] = '0' + tm_utc.tm_sec % 10;
    buf[pos++] = '.';
    buf[pos++] = '0' + ms / 100;
    buf[pos++] = '0' + (ms / 10) % 10;
    buf[pos++] = '0' + ms % 10;
    buf[pos++] = '+';
    buf[pos++] = '0';
    buf[pos++] = '0';
    buf[pos++] = ':';
    buf[pos++] = '0';
    buf[pos++] = '0';
    buf[pos] = '\0';
}

// Write 1 byte to register using I2C_RDWR
static inline int i2c_write_reg8(int fd, uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = {
        .addr  = addr,
        .flags = 0,          // write
        .len   = 2,
        .buf   = buf
    };
    struct i2c_rdwr_ioctl_data data = {
        .msgs  = &msg,
        .nmsgs = 1
    };
    int rc = ioctl(fd, I2C_RDWR, &data);
    return (rc == 1) ? 0 : -1;
}

// Read 1 byte from register using write-then-read with repeated start
static int i2c_read_reg8(int fd, uint8_t addr, uint8_t reg, uint8_t *val) {
    struct i2c_msg msgs[2];
    msgs[0].addr  = addr;
    msgs[0].flags = 0;       // write
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = addr;
    msgs[1].flags = I2C_M_RD; // read
    msgs[1].len   = 1;
    msgs[1].buf   = val;

    struct i2c_rdwr_ioctl_data data = { .msgs = msgs, .nmsgs = 2 };
    int rc = ioctl(fd, I2C_RDWR, &data);
    return (rc == 2) ? 0 : -1;
}

// Read block of n bytes starting at register
static int i2c_read_block(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    struct i2c_msg msgs[2];
    msgs[0].addr  = addr;
    msgs[0].flags = 0;          // write: set register address
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = addr;
    msgs[1].flags = I2C_M_RD;   // read n bytes
    msgs[1].len   = n;
    msgs[1].buf   = buf;

    struct i2c_rdwr_ioctl_data data = { .msgs = msgs, .nmsgs = 2 };
    int rc = ioctl(fd, I2C_RDWR, &data);
    return (rc == 2) ? 0 : -1;
}

static inline int16_t s16(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

static int find_addr(int fd, uint8_t *out_addr) {
    uint8_t candidates[2] = {0x68, 0x69};
    for (int i = 0; i < 2; i++) {
        uint8_t addr = candidates[i];
        uint8_t who = 0;
        if (i2c_read_reg8(fd, addr, WHO_AM_I, &who) == 0 && who == 0x68) {
            *out_addr = addr;
            return 0;
        }
    }
    return -1;
}

static int init_mpu(int fd, uint8_t addr) {
    // Wake up device
    if (i2c_write_reg8(fd, addr, PWR_MGMT_1, 0x00) < 0) return -1;

    // Sleep 50ms
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 50000000 };
    nanosleep(&ts, NULL);

    // Sample rate: 1kHz / (1 + 7) = 125 Hz
    if (i2c_write_reg8(fd, addr, SMPLRT_DIV, 0x07) < 0) return -1;
    // DLPF ~ 44 Hz
    if (i2c_write_reg8(fd, addr, CONFIG_REG, 0x03) < 0) return -1;
    // +/- 2g
    if (i2c_write_reg8(fd, addr, ACCEL_CONFIG, 0x00) < 0) return -1;

    return 0;
}

static inline int read_accel_ms2(int fd, uint8_t addr, double *ax, double *ay, double *az) {
    uint8_t b[6];
    if (i2c_read_block(fd, addr, ACCEL_XOUT_H, b, 6) < 0) {
        return -1;
    }
    // Direct conversion with precomputed scale
    *ax = (double)s16(b[0], b[1]) * ACCEL_SCALE;
    *ay = (double)s16(b[2], b[3]) * ACCEL_SCALE;
    *az = (double)s16(b[4], b[5]) * ACCEL_SCALE;
    return 0;
}

int main(void) {
    // Handle Ctrl+C
    signal(SIGINT, handle_sigint);

    // Open I2C bus
    int i2c_fd = open(I2C_DEV, O_RDWR);
    if (i2c_fd < 0) {
        perror("open(/dev/i2c-1)");
        return 1;
    }

    // Find address (0x68/0x69)
    uint8_t addr = 0;
    if (find_addr(i2c_fd, &addr) < 0) {
        fprintf(stderr, "MPU-6050 not found at 0x68/0x69\n");
        close(i2c_fd);
        return 1;
    }

    // Optional: bind address for legacy ioctls (not strictly needed for I2C_RDWR)
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        perror("ioctl(I2C_SLAVE)");
        close(i2c_fd);
        return 1;
    }

    if (init_mpu(i2c_fd, addr) < 0) {
        fprintf(stderr, "Failed to initialize MPU-6050\n");
        close(i2c_fd);
        return 1;
    }

    printf("MPU-6050 @ 0x%02X ready\n", addr);

    // Set up UDP socket (open once)
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        close(i2c_fd);
        return 1;
    }

    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT)
    };
    if (inet_pton(AF_INET, PC_IP, &dst.sin_addr) != 1) {
        fprintf(stderr, "Invalid PC_IP: %s\n", PC_IP);
        close(sock);
        close(i2c_fd);
        return 1;
    }

    struct timespec sleep_ts = { .tv_sec = 0, .tv_nsec = 500000000 }; // 500 ms
    char line[128];
    char tsbuf[32];
    struct timespec ts;

    while (g_running) {
        double ax, ay, az;
        if (read_accel_ms2(i2c_fd, addr, &ax, &ay, &az) == 0) {
            clock_gettime(CLOCK_REALTIME, &ts);
            iso8601_utc_ms(tsbuf, &ts);

            // Manual string construction for speed
            int len = 0;
            line[len++] = 't'; line[len++] = 's'; line[len++] = '=';
            
            // Copy timestamp
            char *p = tsbuf;
            while (*p) line[len++] = *p++;
            
            line[len++] = ' '; line[len++] = 'X'; line[len++] = '=';
            len += snprintf(line + len, 16, "%.3f", ax);
            line[len++] = ' '; line[len++] = 'Y'; line[len++] = '=';
            len += snprintf(line + len, 16, "%.3f", ay);
            line[len++] = ' '; line[len++] = 'Z'; line[len++] = '=';
            len += snprintf(line + len, 16, "%.3f", az);
            line[len] = '\0';

            write(STDOUT_FILENO, line, len);
            write(STDOUT_FILENO, "\n", 1);

            sendto(sock, line, len, 0, (struct sockaddr *)&dst, sizeof(dst));
        }

        nanosleep(&sleep_ts, NULL);
    }

    close(sock);
    close(i2c_fd);
    return 0;
}
