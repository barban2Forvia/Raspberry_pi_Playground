
// udp_latency_log_xyz.c
// Listens on 0.0.0.0:5005, parses "ts=<ISO8601>" and X/Y/Z values,
// prints them and logs everything to a .txt (TSV) file.
//
// Build:
//   Linux/macOS: cc -O3 -march=native -flto -o udp_latency_log udp_latency_log_xyz.c
//   Windows (MSVC): cl /W4 /O2 udp_latency_log_xyz.c ws2_32.lib
//   Windows (MinGW): gcc -O3 -march=native -flto -o udp_latency_log.exe udp_latency_log_xyz.c -lws2_32
//
// Run:
//   ./udp_latency_log [bind_ip] [port] [log_file]
//   e.g., ./udp_latency_log 0.0.0.0 5005 udp_log.txt

#include <stdio.h>
#include <stdint.h>
#include <time.h>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #pragma comment(lib, "Ws2_32.lib")
  #define CLOSESOCK closesocket
  #define sock_perror(msg) fprintf(stderr, "%s: WSA error %ld\n", msg, WSAGetLastError())
  #ifndef _TIMESPEC_DEFINED
  struct timespec { time_t tv_sec; long tv_nsec; };
  #endif
#else
  #include <unistd.h>
  #include <arpa/inet.h>
  #include <sys/socket.h>
  #define CLOSESOCK close
  #define sock_perror(msg) perror(msg)
#endif

#define DEFAULT_IP   "0.0.0.0"
#define DEFAULT_PORT 5005
#define DEFAULT_LOG  "udp_log.txt"
#define MAX_UDP      65535

// -------- Time utilities --------

#ifdef _WIN32
static void now_utc_timespec(struct timespec* ts) {
    static BOOL has_precise = TRUE;
    FILETIME ft;
    ULARGE_INTEGER uli;
    if (has_precise) {
        static BOOL resolved = FALSE;
        static void (WINAPI *pGetSystemTimePreciseAsFileTime)(LPFILETIME) = NULL;
        if (!resolved) {
            HMODULE h = GetModuleHandleA("kernel32.dll");
            pGetSystemTimePreciseAsFileTime =
                (void (WINAPI *)(LPFILETIME))GetProcAddress(h, "GetSystemTimePreciseAsFileTime");
            resolved = TRUE;
            if (!pGetSystemTimePreciseAsFileTime) has_precise = FALSE;
        }
        if (pGetSystemTimePreciseAsFileTime) pGetSystemTimePreciseAsFileTime(&ft);
        else GetSystemTimeAsFileTime(&ft);
    } else {
        GetSystemTimeAsFileTime(&ft);
    }
    uli.LowPart  = ft.dwLowDateTime;
    uli.HighPart = ft.dwHighDateTime;
    const uint64_t EPOCH_DIFF_100NS = 11644473600ULL * 10000000ULL; // 1601->1970
    uint64_t t100 = uli.QuadPart - EPOCH_DIFF_100NS;
    ts->tv_sec  = (time_t)(t100 / 10000000ULL);
    ts->tv_nsec = (long)((t100 % 10000000ULL) * 100);
}
#else
static void now_utc_timespec(struct timespec* ts) { clock_gettime(CLOCK_REALTIME, ts); }
#endif

// Portable UTC 'timegm'
static time_t timegm_portable(struct tm* tm_utc) {
#ifdef _WIN32
    return _mkgmtime(tm_utc);
#else
    return timegm(tm_utc);
#endif
}

// Convert timespec (UTC) to ISO8601 "YYYY-MM-DDThh:mm:ss.mmmZ"
static inline void iso8601_utc_ms_from_timespec(const struct timespec* ts, char* out) {
    struct tm tm_utc;
#ifdef _WIN32
    gmtime_s(&tm_utc, &ts->tv_sec);
#else
    gmtime_r(&ts->tv_sec, &tm_utc);
#endif
    int ms = (int)(ts->tv_nsec / 1000000L);
    int year = tm_utc.tm_year + 1900;
    register int pos = 0;
    out[pos++] = '0' + year / 1000;
    out[pos++] = '0' + (year / 100) % 10;
    out[pos++] = '0' + (year / 10) % 10;
    out[pos++] = '0' + year % 10;
    out[pos++] = '-';
    out[pos++] = '0' + (tm_utc.tm_mon + 1) / 10;
    out[pos++] = '0' + (tm_utc.tm_mon + 1) % 10;
    out[pos++] = '-';
    out[pos++] = '0' + tm_utc.tm_mday / 10;
    out[pos++] = '0' + tm_utc.tm_mday % 10;
    out[pos++] = 'T';
    out[pos++] = '0' + tm_utc.tm_hour / 10;
    out[pos++] = '0' + tm_utc.tm_hour % 10;
    out[pos++] = ':';
    out[pos++] = '0' + tm_utc.tm_min / 10;
    out[pos++] = '0' + tm_utc.tm_min % 10;
    out[pos++] = ':';
    out[pos++] = '0' + tm_utc.tm_sec / 10;
    out[pos++] = '0' + tm_utc.tm_sec % 10;
    out[pos++] = '.';
    out[pos++] = '0' + ms / 100;
    out[pos++] = '0' + (ms / 10) % 10;
    out[pos++] = '0' + ms % 10;
    out[pos++] = 'Z';
    out[pos] = '\0';
}

// Compute (a - b) in milliseconds
static inline double timespec_diff_ms(const struct timespec* a, const struct timespec* b) {
    return (double)(a->tv_sec - b->tv_sec) * 1000.0 + (double)(a->tv_nsec - b->tv_nsec) * 1e-6;
}

// -------- Payload parsing --------

// Extract the token after "ts=" (up to first whitespace). Returns 0 on success.
static inline int extract_ts_token(const char* payload, char* out, size_t outsz) {
    const char* p = payload;
    // Skip whitespace manually
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    // Check for "ts="
    if (p[0] != 't' || p[1] != 's' || p[2] != '=') return -1;
    p += 3;
    size_t i = 0;
    // Copy until whitespace
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
        if (i + 1 < outsz) out[i++] = *p;
        ++p;
    }
    out[i] = '\0';
    return (i > 0) ? 0 : -1;
}

// Parse ISO8601 with optional fractional seconds and timezone offset.
// Supports: Z, +HH[:MM], +HHMM, +HH (and negatives).
// Returns 0 on success. Result is UTC.
static int parse_iso8601_with_offset_to_timespec(const char* iso, struct timespec* out) {
    if (!iso || !out) return -1;

    // Parse core datetime: YYYY-MM-DDThh:mm:ss
    int year, mon, day, hour, min, sec;
    if (sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &mon, &day, &hour, &min, &sec) != 6)
        return -1;

    const char* tptr = iso;
    while (*tptr && *tptr != 'T') tptr++;
    if (!*tptr) return -1;
    const char* p = tptr + 1;
    // Move past "hh:mm:ss" - trust format from our sender
    if (p[2] != ':' || p[5] != ':') return -1;
    p += 8;

    long nanos = 0;
    // Fractional seconds
    if (*p == '.') {
        ++p;
        int digits = 0;
        long long frac_val = 0;
        while (*p >= '0' && *p <= '9' && digits < 9) {
            frac_val = frac_val * 10 + (*p - '0');
            ++p; ++digits;
        }
        for (int i = digits; i < 9; ++i) frac_val *= 10;
        nanos = (long)frac_val;
        while (*p >= '0' && *p <= '9') ++p;
    }

    int tz_sign = 0, tz_h = 0, tz_m = 0;
    if (*p == 'Z' || *p == 'z') { tz_sign = 0; ++p; }
    else if (*p == '+' || *p == '-') {
        tz_sign = (*p == '-') ? -1 : 1;
        ++p;
        if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return -1;
        tz_h = (p[0]-'0')*10 + (p[1]-'0'); p += 2;
        if (*p == ':') {
            ++p;
            if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return -1;
            tz_m = (p[0]-'0')*10 + (p[1]-'0'); p += 2;
        } else if (p[0] >= '0' && p[0] <= '9' && p[1] >= '0' && p[1] <= '9') {
            tz_m = (p[0]-'0')*10 + (p[1]-'0'); p += 2;
        }
    } else {
        return -1;
    }

    struct tm tm_utc = {0};
    tm_utc.tm_year = year - 1900;
    tm_utc.tm_mon  = mon - 1;
    tm_utc.tm_mday = day;
    tm_utc.tm_hour = hour;
    tm_utc.tm_min  = min;
    tm_utc.tm_sec  = sec;
    tm_utc.tm_isdst = 0;

    time_t base = timegm_portable(&tm_utc);
    if (base == (time_t)-1) return -1;

    long tz_offset_sec = (long)(tz_h * 3600 + tz_m * 60);
    if (tz_sign != 0) base -= (tz_sign * tz_offset_sec);

    out->tv_sec  = base;
    out->tv_nsec = nanos;
    return 0;
}

// Manual double parser (simplified for performance)
static inline double parse_double(const char* str, const char** endptr) {
    double result = 0.0, sign = 1.0;
    const char* p = str;
    if (*p == '-') { sign = -1.0; p++; }
    else if (*p == '+') p++;
    
    while (*p >= '0' && *p <= '9') {
        result = result * 10.0 + (*p - '0');
        p++;
    }
    
    if (*p == '.') {
        p++;
        double frac = 0.0, divisor = 10.0;
        while (*p >= '0' && *p <= '9') {
            frac += (*p - '0') / divisor;
            divisor *= 10.0;
            p++;
        }
        result += frac;
    }
    
    if (endptr) *endptr = p;
    return result * sign;
}

// Parse X/Y/Z from payload irrespective of order.
// Returns count of successfully parsed components (0..3).
static inline int parse_xyz_values(const char* payload, double* X, double* Y, double* Z) {
    int cnt = 0;
    const char* p = payload;
    
    // Scan through once instead of multiple strstr calls
    while (*p) {
        if (p[0] == 'X' && p[1] == '=') {
            const char* end;
            *X = parse_double(p + 2, &end);
            if (end != p + 2) cnt++;
            p = end;
        } else if (p[0] == 'Y' && p[1] == '=') {
            const char* end;
            *Y = parse_double(p + 2, &end);
            if (end != p + 2) cnt++;
            p = end;
        } else if (p[0] == 'Z' && p[1] == '=') {
            const char* end;
            *Z = parse_double(p + 2, &end);
            if (end != p + 2) cnt++;
            p = end;
        } else {
            p++;
        }
    }
    return cnt;
}

// -------- Main program --------

// Simple atoi replacement
static inline int parse_int(const char* s) {
    int result = 0;
    while (*s >= '0' && *s <= '9') {
        result = result * 10 + (*s - '0');
        s++;
    }
    return result;
}

int main(int argc, char* argv[]) {
    const char* ip_str  = (argc >= 2) ? argv[1] : DEFAULT_IP;
    uint16_t    port    = (argc >= 3) ? (uint16_t)parse_int(argv[2]) : DEFAULT_PORT;
    const char* logpath = (argc >= 4) ? argv[3] : DEFAULT_LOG;

    // Prepare log file (append). Write header if file doesn't exist.
    FILE* fcheck = fopen(logpath, "r");
    int write_header = (fcheck == NULL);
    if (fcheck) fclose(fcheck);
    
    FILE* logf = fopen(logpath, "a");
    if (!logf) {
        fprintf(stderr, "ERROR: cannot open log file '%s' for append\n", logpath);
    } else {
        setvbuf(logf, NULL, _IOLBF, 0);
        if (write_header) {
            fprintf(logf, "local_utc\tdelta_ms\tremote_ts\tX\tY\tZ\tsrc_ip\tsrc_port\n");
        }
    }

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        sock_perror("WSAStartup failed");
        if (logf) fclose(logf);
        return EXIT_FAILURE;
    }
#endif

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        sock_perror("socket");
#ifdef _WIN32
        WSACleanup();
#endif
        if (logf) fclose(logf);
        return EXIT_FAILURE;
    }

    int yes = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    // Check for "0.0.0.0" manually
    if (ip_str[0] == '0' && ip_str[1] == '.' && ip_str[2] == '0' && ip_str[3] == '.' && 
        ip_str[4] == '0' && ip_str[5] == '.' && ip_str[6] == '0' && ip_str[7] == '\0') {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, ip_str, &addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP address: %s\n", ip_str);
        CLOSESOCK(sockfd);
#ifdef _WIN32
        WSACleanup();
#endif
        if (logf) fclose(logf);
        return EXIT_FAILURE;
    }

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        sock_perror("bind");
        CLOSESOCK(sockfd);
#ifdef _WIN32
        WSACleanup();
#endif
        if (logf) fclose(logf);
        return EXIT_FAILURE;
    }

    printf("Listening UDP on %s:%u ...  Logging to: %s\n", ip_str, (unsigned)port, logpath);

    unsigned char buf[MAX_UDP + 1];

    while (1) {
        struct sockaddr_in src; socklen_t srclen = sizeof(src);

#ifdef _WIN32
        int n = recvfrom(sockfd, (char*)buf, MAX_UDP, 0, (struct sockaddr*)&src, &srclen);
#else
        ssize_t n = recvfrom(sockfd, buf, MAX_UDP, 0, (struct sockaddr*)&src, &srclen);
#endif
        if (n < 0) {
            sock_perror("recvfrom");
            continue;
        }

        // Local receive time immediately after recv
        struct timespec local_ts; now_utc_timespec(&local_ts);

        size_t len = (size_t)n;
        if (len > MAX_UDP) len = MAX_UDP;
        buf[len] = '\0';

        char src_ip[INET_ADDRSTRLEN] = {0};
        if (!inet_ntop(AF_INET, &src.sin_addr, src_ip, sizeof(src_ip))) {
            strncpy(src_ip, "(unknown)", sizeof(src_ip) - 1);
        }
        unsigned src_port = ntohs(src.sin_port);

        // Extract and parse remote ts
        char ts_token[64] = {0};
        struct timespec remote_ts; int have_ts = 0;
        if (extract_ts_token((const char*)buf, ts_token, sizeof(ts_token)) == 0) {
            if (parse_iso8601_with_offset_to_timespec(ts_token, &remote_ts) == 0) {
                have_ts = 1;
            }
        }

        // Parse X/Y/Z (any order)
        double X=0.0, Y=0.0, Z=0.0; 
        int xyz_count = parse_xyz_values((const char*)buf, &X, &Y, &Z);

        // Compute delta if ts ok
        double delta_ms = 0.0;
        if (have_ts) delta_ms = timespec_diff_ms(&local_ts, &remote_ts);

        // Prepare local ISO8601 for logs
        char local_iso[32]; 
        iso8601_utc_ms_from_timespec(&local_ts, local_iso);

        // Console output
        printf("From %s:%u — %zu bytes\n", src_ip, src_port, len);
        if (have_ts) {
            printf("Δt = %.3f ms", delta_ms);
            if (xyz_count > 0) printf(" | X=%.3f Y=%.3f Z=%.3f", X, Y, Z);
            printf("\n");
        } else {
            printf("Warning: Could not parse 'ts=' ISO8601 from payload start.\n");
            if (xyz_count > 0) printf("X=%.3f Y=%.3f Z=%.3f\n", X, Y, Z);
        }

        // Log to file (TSV)
        if (logf) {
            if (have_ts) {
                fprintf(logf, "%s\t%.3f\t%s\t", local_iso, delta_ms, ts_token);
            } else {
                fprintf(logf, "%s\t\t\t", local_iso);
            }
            if (xyz_count > 0) fprintf(logf, "%.6f\t%.6f\t%.6f\t", X, Y, Z);
            else               fprintf(logf, "\t\t\t");
            fprintf(logf, "%s\t%u\n", src_ip, src_port);
        }

        fflush(stdout);
    }

    // Unreachable in normal use
    // (If you add a signal handler, close resources here.)
    // CLOSESOCK(sockfd); if (logf) fclose(logf);
#ifdef _WIN32
    WSACleanup();
#endif
    return EXIT_SUCCESS;
}
