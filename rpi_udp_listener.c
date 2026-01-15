
// udp_latency_log_xyz.c
// Listens on 0.0.0.0:5005, parses "ts=<ISO8601>" and X/Y/Z values,
// prints them and logs everything to a .txt (TSV) file.
//
// Build:
//   Linux/macOS: cc -O2 -Wall -Wextra -o udp_latency_log udp_latency_log_xyz.c
//   Windows (MSVC): cl /W4 /O2 udp_latency_log_xyz.c ws2_32.lib
//   Windows (MinGW): gcc -O2 -Wall -Wextra -o udp_latency_log.exe udp_latency_log_xyz.c -lws2_32
//
// Run:
//   ./udp_latency_log [bind_ip] [port] [log_file]
//   e.g., ./udp_latency_log 0.0.0.0 5005 udp_log.txt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <time.h>
#include <errno.h>
#include <locale.h>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #include <sys/stat.h>
  #pragma comment(lib, "Ws2_32.lib")
  #define CLOSESOCK closesocket
  static void sock_perror(const char* msg) {
      fprintf(stderr, "%s: WSA error %ld\n", msg, WSAGetLastError());
  }
  #ifndef _TIMESPEC_DEFINED
  #define _TIMESPEC_DEFINED
  struct timespec { time_t tv_sec; long tv_nsec; };
  #endif
#else
  #include <unistd.h>
  #include <errno.h>
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <sys/stat.h>
  #define CLOSESOCK close
  static void sock_perror(const char* msg) { perror(msg); }
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
static void iso8601_utc_ms_from_timespec(const struct timespec* ts, char* out, size_t outsz) {
    struct tm tm_utc;
#ifdef _WIN32
    gmtime_s(&tm_utc, &ts->tv_sec);
#else
    gmtime_r(&ts->tv_sec, &tm_utc);
#endif
    int ms = (int)(ts->tv_nsec / 1000000L);
    // Ensure buffer large enough
    // "YYYY-MM-DDThh:mm:ss.mmmZ" = 24 chars + NUL
    snprintf(out, outsz, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
             tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday,
             tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec, ms);
}

// Compute (a - b) normalized
static void timespec_diff(const struct timespec* a, const struct timespec* b, struct timespec* d) {
    d->tv_sec  = a->tv_sec  - b->tv_sec;
    d->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (d->tv_nsec < 0) {
        d->tv_sec -= 1;
        d->tv_nsec += 1000000000L;
    }
}

// -------- Payload parsing --------

// Extract the token after "ts=" (up to first whitespace). Returns 0 on success.
static int extract_ts_token(const char* payload, char* out, size_t outsz) {
    if (!payload || !out || outsz == 0) return -1;
    const char* p = payload;
    while (*p && isspace((unsigned char)*p)) ++p;
    if (strncmp(p, "ts=", 3) != 0) return -1;
    p += 3;
    size_t i = 0;
    while (*p && !isspace((unsigned char)*p)) {
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

    const char* tptr = strchr(iso, 'T');
    if (!tptr) return -1;
    const char* p = tptr + 1; // hh...
    // Move past "hh:mm:ss"
    if (!isdigit((unsigned char)p[0]) || !isdigit((unsigned char)p[1]) ||
        p[2] != ':' || !isdigit((unsigned char)p[3]) || !isdigit((unsigned char)p[4]) ||
        p[5] != ':' || !isdigit((unsigned char)p[6]) || !isdigit((unsigned char)p[7])) return -1;
    p += 8;

    long nanos = 0;
    // Fractional seconds
    if (*p == '.') {
        ++p;
        int digits = 0;
        long long frac_val = 0;
        while (isdigit((unsigned char)*p) && digits < 9) {
            frac_val = frac_val * 10 + (*p - '0');
            ++p; ++digits;
        }
        for (int i = digits; i < 9; ++i) frac_val *= 10; // pad
        nanos = (long)frac_val;
        while (isdigit((unsigned char)*p)) ++p; // skip extra digits
    }

    int tz_sign = 0, tz_h = 0, tz_m = 0;
    if (*p == 'Z' || *p == 'z') { tz_sign = 0; ++p; }
    else if (*p == '+' || *p == '-') {
        tz_sign = (*p == '-') ? -1 : 1;
        ++p;
        if (!isdigit((unsigned char)p[0]) || !isdigit((unsigned char)p[1])) return -1;
        tz_h = (p[0]-'0')*10 + (p[1]-'0'); p += 2;
        if (*p == ':') {
            ++p;
            if (!isdigit((unsigned char)p[0]) || !isdigit((unsigned char)p[1])) return -1;
            tz_m = (p[0]-'0')*10 + (p[1]-'0'); p += 2;
        } else if (isdigit((unsigned char)p[0]) && isdigit((unsigned char)p[1])) {
            tz_m = (p[0]-'0')*10 + (p[1]-'0'); p += 2;
        } else {
            tz_m = 0;
        }
    } else {
        return -1;
    }

    struct tm tm_utc; memset(&tm_utc, 0, sizeof(tm_utc));
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

// Parse X/Y/Z from payload irrespective of order.
// Returns count of successfully parsed components (0..3).
static int parse_xyz_values(const char* payload, double* X, double* Y, double* Z) {
    int cnt = 0;
    const char* p;

    if (X) {
        p = strstr(payload, "X=");
        if (p) {
            char* endptr = NULL;
            double v = strtod(p + 2, &endptr);
            if (endptr && endptr != p + 2) { *X = v; cnt++; }
        }
    }
    if (Y) {
        p = strstr(payload, "Y=");
        if (p) {
            char* endptr = NULL;
            double v = strtod(p + 2, &endptr);
            if (endptr && endptr != p + 2) { *Y = v; cnt++; }
        }
    }
    if (Z) {
        p = strstr(payload, "Z=");
        if (p) {
            char* endptr = NULL;
            double v = strtod(p + 2, &endptr);
            if (endptr && endptr != p + 2) { *Z = v; cnt++; }
        }
    }
    return cnt;
}

// -------- Main program --------

int main(int argc, char* argv[]) {
    // Ensure decimal point is '.' (robust across locales)
    setlocale(LC_NUMERIC, "C");

    const char* ip_str  = (argc >= 2) ? argv[1] : DEFAULT_IP;
    uint16_t    port    = (argc >= 3) ? (uint16_t)atoi(argv[2]) : DEFAULT_PORT;
    const char* logpath = (argc >= 4) ? argv[3] : DEFAULT_LOG;

    // Prepare log file (append). Write header if file doesn't exist.
    int write_header = 0;
    {
        FILE* fcheck = fopen(logpath, "r");
        if (!fcheck) write_header = 1;
        else fclose(fcheck);
    }
    FILE* logf = fopen(logpath, "a");
    if (!logf) {
        fprintf(stderr, "ERROR: cannot open log file '%s' for append: %s\n", logpath, strerror(errno));
        // Not fatal—continue without logging.
    } else {
        // Line-buffered logs for immediate flush on newline
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

    struct sockaddr_in addr; memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (strcmp(ip_str, "0.0.0.0") == 0) addr.sin_addr.s_addr = htonl(INADDR_ANY);
    else if (inet_pton(AF_INET, ip_str, &addr.sin_addr) != 1) {
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
        double X=0.0, Y=0.0, Z=0.0; int xyz_count = parse_xyz_values((const char*)buf, &X, &Y, &Z);

        // Compute and show delta if ts ok
        double delta_ms = 0.0;
        if (have_ts) {
            struct timespec diff; timespec_diff(&local_ts, &remote_ts, &diff);
            delta_ms = (double)diff.tv_sec * 1000.0 + (double)diff.tv_nsec / 1e6; // <-- small mistake fixed below
        }

        // Prepare local ISO8601 for logs
        char local_iso[32]; iso8601_utc_ms_from_timespec(&local_ts, local_iso, sizeof(local_iso));

        // Console output
        printf("From %s:%u — %zu bytes\n", src_ip, src_port, len);
        if (have_ts) {
            // Print Δt and XYZ
            struct timespec diff; timespec_diff(&local_ts, &remote_ts, &diff);
            double ms = (double)diff.tv_sec * 1000.0 + (double)diff.tv_nsec / 1e6;
            printf("Δt = %.3f ms", ms);
            if (xyz_count > 0) printf(" | X=%.3f Y=%.3f Z=%.3f", X, Y, Z);
            printf("\n");
        } else {
            printf("Warning: Could not parse 'ts=' ISO8601 from payload start.\n");
            if (xyz_count > 0) printf("X=%.3f Y=%.3f Z=%.3f\n", X, Y, Z);
        }

        // Log to file (TSV): local_utc, delta_ms, remote_ts, X, Y, Z, src_ip, src_port
        if (logf) {
            if (have_ts) {
                fprintf(logf, "%s\t%.3f\t%s\t", local_iso, // local time
                        // recompute ms reliably to avoid any mismatch
                        (double)((local_ts.tv_sec - remote_ts.tv_sec) * 1000.0) +
                        (double)(local_ts.tv_nsec - remote_ts.tv_nsec) / 1e6,
                        ts_token);
            } else {
                fprintf(logf, "%s\t\t\t", local_iso); // leave delta_ms and remote_ts empty
            }
            if (xyz_count > 0) fprintf(logf, "%.6f\t%.6f\t%.6f\t", X, Y, Z);
            else               fprintf(logf, "\t\t\t");
            fprintf(logf, "%s\t%u\n", src_ip, src_port);
            // log is line-buffered; flushed automatically
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
