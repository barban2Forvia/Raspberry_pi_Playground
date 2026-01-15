
// mpu6050_udp_nosmbus.c
// MPU-6050 I2C accelerometer reader for Raspberry Pi + UDP sender (no i2c/smbus.h)
// Build: gcc -O2 mpu6050_udp_nosmbus.c -o mpu6050_udp
// Run  : ./mpu6050_udp

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <math.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

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
#define LSB_PER_G  16384.0   // +/-2g scale
#define G_TO_MS2   9.80665

// ======== GLOBALS ========
static volatile sig_atomic_t g_running = 1;

// ======== SIGNAL HANDLER ========
static void handle_sigint(int sig) {
    (void)sig;
    g_running = 0;
}

// ======== UTILS ========

// Format ISO-8601 UTC with millisecond precision: YYYY-MM-DDTHH:MM:SS.sss+00:00
static void iso8601_utc_ms(char *buf, size_t buflen) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    time_t sec = ts.tv_sec;
    struct tm tm_utc;
    gmtime_r(&sec, &tm_utc);

    char tmp[32];
    strftime(tmp, sizeof(tmp), "%Y-%m-%dT%H:%M:%S", &tm_utc);

    long ms = ts.tv_nsec / 1000000L;
    snprintf(buf, buflen, "%s.%03ld+00:00", tmp, ms);
}

// Write 1 byte to register using I2C_RDWR
static int i2c_write_reg8(int fd, uint8_t addr, uint8_t reg, uint8_t val) {
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

static int16_t s16(uint8_t msb, uint8_t lsb) {
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
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 50 * 1000 * 1000 };
    nanosleep(&ts, NULL);

    // Sample rate: 1kHz / (1 + 7) = 125 Hz
    if (i2c_write_reg8(fd, addr, SMPLRT_DIV, 0x07) < 0) return -1;
    // DLPF ~ 44 Hz
    if (i2c_write_reg8(fd, addr, CONFIG_REG, 0x03) < 0) return -1;
    // +/- 2g
    if (i2c_write_reg8(fd, addr, ACCEL_CONFIG, 0x00) < 0) return -1;

    return 0;
}

static int read_accel_ms2(int fd, uint8_t addr, double *ax, double *ay, double *az) {
    uint8_t b[6];
    if (i2c_read_block(fd, addr, ACCEL_XOUT_H, b, 6) < 0) {
        return -1;
    }
    int16_t raw_x = s16(b[0], b[1]);
    int16_t raw_y = s16(b[2], b[3]);
    int16_t raw_z = s16(b[4], b[5]);

    *ax = (double)raw_x / LSB_PER_G * G_TO_MS2;
    *ay = (double)raw_y / LSB_PER_G * G_TO_MS2;
    *az = (double)raw_z / LSB_PER_G * G_TO_MS2;
    return 0;
}

int main(void) {
    // Handle Ctrl+C
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_sigint;
    sigaction(SIGINT, &sa, NULL);

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

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(PORT);
    if (inet_pton(AF_INET, PC_IP, &dst.sin_addr) != 1) {
        fprintf(stderr, "Invalid PC_IP: %s\n", PC_IP);
        close(sock);
        close(i2c_fd);
        return 1;
    }

    struct timespec sleep_ts = { .tv_sec = 0, .tv_nsec = 500 * 1000 * 1000 }; // 500 ms

    while (g_running) {
        double ax, ay, az;
        if (read_accel_ms2(i2c_fd, addr, &ax, &ay, &az) == 0) {
            char tsbuf[40];
            iso8601_utc_ms(tsbuf, sizeof(tsbuf));

            char line[128];
            snprintf(line, sizeof(line), "ts=%s X=%.3f Y=%.3f Z=%.3f", tsbuf, ax, ay, az);

            printf("%s\n", line);

            ssize_t sent = sendto(sock, line, strlen(line), 0,
                                  (struct sockaddr *)&dst, sizeof(dst));
            if (sent < 0) {
                perror("sendto");
            }
        } else {
            fprintf(stderr, "Failed to read accelerometer data\n");
        }

        nanosleep(&sleep_ts, NULL);
    }

    close(sock);
    close(i2c_fd);
    return 0;
}
