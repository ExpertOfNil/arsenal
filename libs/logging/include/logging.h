#ifndef LOGGING_H
#define LOGGING_H

#include <stddef.h>
#include <stdio.h>

#define LOG_TIMESTAMP_SIZE 20
#define LOG_DATE_SIZE 11
#define LOG_DIR_SIZE 512
#define LOG_BASENAME_SIZE 128
#define LOG_PATH_SIZE 1024

#if defined(__GNUC__) || defined(__clang__)
#define LOG_PRINTF(fmt_idx, arg_idx) \
    __attribute__((format(printf, fmt_idx, arg_idx)))
#else
#define LOG_PRINTF(fmt_idx, arg_idx)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_LEVEL_TRACE,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} LogLevel;
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG
#endif
#ifndef LOG_FILE_LEVEL
#define LOG_FILE_LEVEL LOG_LEVEL_DEBUG
#endif

int log_enable_file(
    const char* directory_path, const char* basename, size_t retention_days
);
void log_disable_file(void);

void log_trace(const char* format, ...) LOG_PRINTF(1, 2);
void log_debug(const char* format, ...) LOG_PRINTF(1, 2);
void log_info(const char* format, ...) LOG_PRINTF(1, 2);
void log_warn(const char* format, ...) LOG_PRINTF(1, 2);
void log_error(const char* format, ...) LOG_PRINTF(1, 2);

void logf_trace(FILE* fp, const char* format, ...) LOG_PRINTF(2, 3);
void logf_debug(FILE* fp, const char* format, ...) LOG_PRINTF(2, 3);
void logf_info(FILE* fp, const char* format, ...) LOG_PRINTF(2, 3);
void logf_warn(FILE* fp, const char* format, ...) LOG_PRINTF(2, 3);
void logf_error(FILE* fp, const char* format, ...) LOG_PRINTF(2, 3);

#ifdef __cplusplus
}
#endif

#undef LOG_PRINTF

#endif /* LOGGING_H */
