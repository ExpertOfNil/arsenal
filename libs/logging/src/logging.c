#include "logging.h"

#include <dirent.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <limits.h>

static pthread_mutex_t g_log_mutex = PTHREAD_MUTEX_INITIALIZER;
static const char LOG_DATE_FORMAT[] = "%Y-%m-%d";
static const char LOG_TIMESTAMP_FORMAT[] = "%Y-%m-%d %H:%M:%S";
static const char LOG_FILE_EXTENSION[] = ".log";
static const char LOG_FILE_SEPARATOR = '-';

static FILE* g_log_file = NULL;
static char g_log_date[LOG_DATE_SIZE] = {0};
static char g_log_dir[LOG_DIR_SIZE] = {0};
static char g_log_basename[LOG_BASENAME_SIZE] = {0};
static size_t g_log_retention_days = 0;

static int format_timestamp(char* buf, size_t len) {
    time_t now = time(NULL);
    struct tm tm_buf;

    if (now == (time_t)-1 || localtime_r(&now, &tm_buf) == NULL) {
        snprintf(buf, len, "0000-00-00 00:00:00");
        return -1;
    }

    if (strftime(buf, len, LOG_TIMESTAMP_FORMAT, &tm_buf) == 0) {
        snprintf(buf, len, "0000-00-00 00:00:00");
        return -1;
    }

    return 0;
}

static int format_date(char* buf, size_t len) {
    time_t now = time(NULL);
    struct tm tm_buf;

    if (now == (time_t)-1 || localtime_r(&now, &tm_buf) == NULL) {
        snprintf(buf, len, "0000-00-00");
        return -1;
    }

    if (strftime(buf, len, LOG_DATE_FORMAT, &tm_buf) == 0) {
        snprintf(buf, len, "0000-00-00");
        return -1;
    }

    return 0;
}

static const char* LogLevel_to_string(LogLevel level) {
    switch (level) {
        case LOG_LEVEL_TRACE:
            return "TRACE";
        case LOG_LEVEL_DEBUG:
            return "DEBUG";
        case LOG_LEVEL_INFO:
            return "INFO";
        case LOG_LEVEL_WARN:
            return "WARN";
        case LOG_LEVEL_ERROR:
            return "ERROR";
    }
    return "UNKNOWN";
}

static int is_digit_char(char c) { return '0' <= c && c <= '9'; }

static int is_log_date_str(const char* s) {
    return is_digit_char(s[0]) && is_digit_char(s[1]) && is_digit_char(s[2]) &&
           is_digit_char(s[3]) && s[4] == '-' && is_digit_char(s[5]) &&
           is_digit_char(s[6]) && s[7] == '-' && is_digit_char(s[8]) &&
           is_digit_char(s[9]);
}

/* Returns a date string from matching log file names
 */
static const char* match_log_filename_date(const char* filename) {
    size_t filename_len = strlen(filename);
    size_t basename_len = strlen(g_log_basename);
    size_t ext_len = strlen(LOG_FILE_EXTENSION);
    size_t expected_len = basename_len + 1 + (LOG_DATE_SIZE - 1) + ext_len;
    // Bounds check
    if (filename_len != expected_len) return NULL;
    // Must start with a basename
    if (basename_len == 0) return NULL;
    if (strncmp(filename, g_log_basename, basename_len) != 0) return NULL;
    // Must have a separator after basename
    if (filename[basename_len] != LOG_FILE_SEPARATOR) return NULL;

    const char* date = filename + basename_len + 1;

    // Must have the exact format
    if (!is_log_date_str(date)) return NULL;

    // Must have the correct extension after date
    const char* ext = date + LOG_DATE_SIZE - 1;
    if (strcmp(ext, LOG_FILE_EXTENSION) != 0) return NULL;

    return date;
}

/* Convert 4 characters of a string into an integer */
static int parse_4_digits(const char* s) {
    return (s[0] - '0') * 1000 + (s[1] - '0') * 100 + (s[2] - '0') * 10 +
           (s[3] - '0');
}

/* Convert 2 characters of a string into an integer */
static int parse_2_digits(const char* s) {
    return (s[0] - '0') * 10 + (s[1] - '0');
}

/* Create an integer key from year, month, and day */
static int date_key_from_ymd(int year, int month, int day) {
    return year * 10000 + month * 100 + day;
}

/* Create an integer key from a date string
 *
 * Assumes %Y-%m-%d format
 */
static int log_date_key(const char* date, int* out) {
    if (!date || !out) return -1;
    // TODO (mmckenna): Assert format?
    if (!is_log_date_str(date)) return -1;

    int year = parse_4_digits(date);
    int month = parse_2_digits(date + 5);
    int day = parse_2_digits(date + 8);

    if (month < 1 || month > 12) return -1;
    if (day < 1 || day > 31) return -1;

    struct tm tm_buf = {0};
    tm_buf.tm_year = year - 1900;
    tm_buf.tm_mon = month - 1;
    tm_buf.tm_mday = day;
    tm_buf.tm_isdst = -1;

    time_t t = mktime(&tm_buf);
    if (t == (time_t)-1) return -1;

    if (tm_buf.tm_year != year - 1900 || tm_buf.tm_mon != month - 1 ||
        tm_buf.tm_mday != day) {
        return -1;
    }

    *out = date_key_from_ymd(year, month, day);
    return 0;
}

/* Create an integer key from the current date and number of retention days
 *
 * Assumes %Y-%m-%d format
 */
static int cutoff_date_key(size_t retention_days, int* out) {
    if (retention_days > INT_MAX) return -1;
    // TODO (mmckenna): Assert format?
    time_t now = time(NULL);
    if (now == (time_t)-1 || !out) return -1;

    struct tm tm_buf = {0};
    if (localtime_r(&now, &tm_buf) == NULL) return -1;
    tm_buf.tm_mday -= (int)retention_days;
    tm_buf.tm_isdst = -1;

    if (mktime(&tm_buf) == (time_t)-1) return -1;

    int year = tm_buf.tm_year + 1900;
    int month = tm_buf.tm_mon + 1;
    int day = tm_buf.tm_mday;

    *out = date_key_from_ymd(year, month, day);
    return 0;
}

/* Remove files that are outside the specified retention window
 *
 * Age is determined by parsing the file name, not by its metadata
 */
static int purge_old_log_files_locked(void) {
    if (g_log_retention_days == 0) return 0;
    if (g_log_dir[0] == '\0' || g_log_basename[0] == '\0') return 0;

    int cutoff_key;
    if (cutoff_date_key(g_log_retention_days, &cutoff_key) != 0) return -1;

    DIR* dir = opendir(g_log_dir);
    if (!dir) return -1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        const char* name = entry->d_name;

        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
            continue;
        }

        const char* date = match_log_filename_date(name);
        if (!date) continue;
        int file_key;
        if (log_date_key(date, &file_key) != 0) continue;
        if (file_key < cutoff_key) {
            char path[LOG_PATH_SIZE] = {0};
            int n = snprintf(path, sizeof(path), "%s/%s", g_log_dir, name);
            if (n < 0 || (size_t)n >= sizeof(path)) continue;
            (void)unlink(path);
        }
    }

    closedir(dir);
    return 0;
}

static int open_log_file_locked(void) {
    char date[LOG_DATE_SIZE] = {0};
    char path[LOG_PATH_SIZE] = {0};

    if (format_date(date, sizeof(date)) != 0) return -1;
    int n = snprintf(
        path, sizeof(path), "%s/%s-%s.log", g_log_dir, g_log_basename, date
    );
    if (n < 0 || (size_t)n >= sizeof(path)) return -1;

    FILE* fp = fopen(path, "a");
    if (!fp) return -1;
    if (g_log_file) {
        fclose(g_log_file);
    }
    g_log_file = fp;
    memcpy(g_log_date, date, sizeof(g_log_date));

    purge_old_log_files_locked();
    return 0;
}

static void clear_log_file_locked(void) {
    if (g_log_file) {
        fclose(g_log_file);
        g_log_file = NULL;
    }
    g_log_dir[0] = '\0';
    g_log_basename[0] = '\0';
    g_log_date[0] = '\0';
    g_log_retention_days = 0;
}

int log_enable_file(
    const char* directory_path, const char* basename, size_t retention_days
) {
    // TODO (mmckenna): add logging esp. for failures
    if (!directory_path || !basename) return -1;
    size_t dir_path_len = strlen(directory_path);
    size_t basename_len = strlen(basename);
    if (dir_path_len == 0 || basename_len == 0) return -1;
    if (dir_path_len >= LOG_DIR_SIZE || basename_len >= LOG_BASENAME_SIZE) {
        return -1;
    }

    pthread_mutex_lock(&g_log_mutex);
    memcpy(g_log_dir, directory_path, dir_path_len + 1);
    memcpy(g_log_basename, basename, basename_len + 1);
    g_log_retention_days = retention_days;

    int result = open_log_file_locked();

    if (result != 0) {
        clear_log_file_locked();
    }
    pthread_mutex_unlock(&g_log_mutex);

    return result;
}

void log_disable_file(void) {
    pthread_mutex_lock(&g_log_mutex);
    clear_log_file_locked();
    pthread_mutex_unlock(&g_log_mutex);
}

static void vlog_message(
    FILE* fp,
    LogLevel level,
    int mirror_to_global_file,
    const char* format,
    va_list args
) {
    static const char LOG_LINE_PREFIX_FORMAT[] = "[%s] %s: ";
    const int write_console = fp && LOG_LEVEL <= level;

    pthread_mutex_lock(&g_log_mutex);
    // Rotate log file if needed
    if (mirror_to_global_file && g_log_file) {
        char date[LOG_DATE_SIZE] = {0};
        if (format_date(date, sizeof(date)) != 0) {
            clear_log_file_locked();
        } else if (strncmp(g_log_date, date, sizeof(g_log_date)) != 0) {
            if (open_log_file_locked() != 0) {
                clear_log_file_locked();
            }
        }
    }

    const int write_file =
        mirror_to_global_file && g_log_file && LOG_FILE_LEVEL <= level;
    if (write_console || write_file) {
        char timestamp[LOG_TIMESTAMP_SIZE];

        format_timestamp(timestamp, sizeof(timestamp));

        if (write_console) {
            va_list console_args;
            va_copy(console_args, args);
            fprintf(
                fp, LOG_LINE_PREFIX_FORMAT, timestamp, LogLevel_to_string(level)
            );
            vfprintf(fp, format, console_args);
            fflush(fp);
            va_end(console_args);
        }

        if (write_file) {
            va_list file_args;
            va_copy(file_args, args);
            fprintf(
                g_log_file,
                LOG_LINE_PREFIX_FORMAT,
                timestamp,
                LogLevel_to_string(level)
            );
            vfprintf(g_log_file, format, file_args);
            fflush(g_log_file);
            va_end(file_args);
        }
    }
    pthread_mutex_unlock(&g_log_mutex);

    return;
}

void log_trace(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(stdout, LOG_LEVEL_TRACE, 1, format, args);
    va_end(args);
}
void log_debug(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(stdout, LOG_LEVEL_DEBUG, 1, format, args);
    va_end(args);
}
void log_info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(stdout, LOG_LEVEL_INFO, 1, format, args);
    va_end(args);
}
void log_warn(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(stdout, LOG_LEVEL_WARN, 1, format, args);
    va_end(args);
}
void log_error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(stderr, LOG_LEVEL_ERROR, 1, format, args);
    va_end(args);
}

void logf_trace(FILE* fp, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(fp, LOG_LEVEL_TRACE, 0, format, args);
    va_end(args);
}
void logf_debug(FILE* fp, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(fp, LOG_LEVEL_DEBUG, 0, format, args);
    va_end(args);
}
void logf_info(FILE* fp, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(fp, LOG_LEVEL_INFO, 0, format, args);
    va_end(args);
}
void logf_warn(FILE* fp, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(fp, LOG_LEVEL_WARN, 0, format, args);
    va_end(args);
}
void logf_error(FILE* fp, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog_message(fp, LOG_LEVEL_ERROR, 0, format, args);
    va_end(args);
}
