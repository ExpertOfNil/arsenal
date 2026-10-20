#include "point_cloud.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

point_cloud_status point_cloud_reserve(point_cloud* cloud, size_t size) {
    if (!cloud) return POINT_CLOUD_INVALID_ARGUMENT;
    if (size <= cloud->capacity) return POINT_CLOUD_OK;
    size_t capacity =
        cloud->capacity ? cloud->capacity : POINT_CLOUD_DEFAULT_CAPACITY;
    while (capacity < size) {
        if (capacity > SIZE_MAX / 2) return POINT_CLOUD_ALLOCATION_ERROR;
        capacity *= 2;
    }
    if (capacity > SIZE_MAX / sizeof(*cloud->points)) {
        return POINT_CLOUD_ALLOCATION_ERROR;
    }
    point3d* points = realloc(cloud->points, capacity * sizeof(*cloud->points));
    if (!points) return POINT_CLOUD_ALLOCATION_ERROR;
    cloud->points = points;
    cloud->capacity = capacity;
    return POINT_CLOUD_OK;
}

point_cloud_status point_cloud_push(const point3d* pt, point_cloud* cloud) {
    if (!pt || !cloud) return POINT_CLOUD_INVALID_ARGUMENT;
    if (cloud->count == SIZE_MAX) return POINT_CLOUD_ALLOCATION_ERROR;
    point_cloud_status result = point_cloud_reserve(cloud, cloud->count + 1);
    if (result != POINT_CLOUD_OK) return result;
    cloud->points[cloud->count] = *pt;
    ++cloud->count;
    return POINT_CLOUD_OK;
}

void point_cloud_reset(point_cloud* cloud) {
    if (!cloud) return;
    cloud->count = 0;
}

static void skip_spaces(const char** cursor) {
    while (**cursor == ' ' || **cursor == '\t') {
        ++(*cursor);
    }
}

static void strip_line_endings(char* line, ssize_t* len) {
    ssize_t n = *len;
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) {
        line[--n] = '\0';
    }
    *len = n;
}

static point_cloud_status parse_header(const char* line) {
    if (!line) return POINT_CLOUD_INVALID_ARGUMENT;
    const char* cursor = line;

    skip_spaces(&cursor);
    if (strncmp(cursor, "id", 2) != 0) return POINT_CLOUD_FORMAT_ERROR;
    cursor += 2;

    skip_spaces(&cursor);
    if (*cursor != ',') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;

    skip_spaces(&cursor);
    if (*cursor != 'x') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;

    skip_spaces(&cursor);
    if (*cursor != ',') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;

    skip_spaces(&cursor);
    if (*cursor != 'y') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;

    skip_spaces(&cursor);
    if (*cursor != ',') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;

    skip_spaces(&cursor);
    if (*cursor != 'z') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;

    skip_spaces(&cursor);
    if (*cursor != '\0') return POINT_CLOUD_FORMAT_ERROR;

    return POINT_CLOUD_OK;
}

static point_cloud_status parse_line(const char* line, point3d* out) {
    if (!line || !out) return POINT_CLOUD_INVALID_ARGUMENT;
    const char* cursor = line;

    skip_spaces(&cursor);
    if (*cursor == '+' || *cursor == '-') return POINT_CLOUD_FORMAT_ERROR;

    // Parse ID
    errno = 0;
    char* end;
    uintmax_t value = strtoumax(cursor, &end, 10);
    if (end == cursor) return POINT_CLOUD_FORMAT_ERROR;
    if (errno == ERANGE || value > UINT64_MAX) return POINT_CLOUD_RANGE_ERROR;

    cursor = end;
    uint64_t id = (uint64_t)value;

    // Parse x-coordinate
    skip_spaces(&cursor);
    if (*cursor != ',') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;
    skip_spaces(&cursor);
    errno = 0;
    double x = strtod(cursor, &end);
    if (end == cursor) return POINT_CLOUD_FORMAT_ERROR;
    if (errno == ERANGE || !isfinite(x)) return POINT_CLOUD_RANGE_ERROR;
    cursor = end;

    // Parse y-coordinate
    skip_spaces(&cursor);
    if (*cursor != ',') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;
    skip_spaces(&cursor);
    errno = 0;
    double y = strtod(cursor, &end);
    if (end == cursor) return POINT_CLOUD_FORMAT_ERROR;
    if (errno == ERANGE || !isfinite(y)) return POINT_CLOUD_RANGE_ERROR;
    cursor = end;

    // Parse z-coordinate
    skip_spaces(&cursor);
    if (*cursor != ',') return POINT_CLOUD_FORMAT_ERROR;
    ++cursor;
    skip_spaces(&cursor);
    errno = 0;
    double z = strtod(cursor, &end);
    if (end == cursor) return POINT_CLOUD_FORMAT_ERROR;
    if (errno == ERANGE || !isfinite(z)) return POINT_CLOUD_RANGE_ERROR;
    cursor = end;

    // Skip trailing spaces
    skip_spaces(&cursor);
    if (*cursor != '\0') return POINT_CLOUD_FORMAT_ERROR;

    out->id = id;
    out->position[0] = x;
    out->position[1] = y;
    out->position[2] = z;
    return POINT_CLOUD_OK;
}

point_cloud_status point_cloud_from_csv(
    const char* path, point_cloud* cloud, size_t* err_line
) {
    if (err_line) *err_line = 0;
    if (!path || !cloud) return POINT_CLOUD_INVALID_ARGUMENT;

    FILE* fp = fopen(path, "r");
    if (!fp) return POINT_CLOUD_IO_ERROR;

    point_cloud tmp = {0};
    point_cloud_status result = POINT_CLOUD_OK;

    char* line = NULL;
    size_t line_capacity = 0;
    size_t line_number = 0;

    ssize_t len = getline(&line, &line_capacity, fp);
    if (len <= 0) {
        result = POINT_CLOUD_FORMAT_ERROR;
        goto cleanup;
    }
    line_number++;
    strip_line_endings(line, &len);
    result = parse_header(line);
    if (result != POINT_CLOUD_OK) {
        if (err_line) *err_line = line_number;
        goto cleanup;
    }

    while ((len = getline(&line, &line_capacity, fp)) != -1) {
        ++line_number;
        strip_line_endings(line, &len);

        point3d pt = {0};
        result = parse_line(line, &pt);
        if (result != POINT_CLOUD_OK) {
            if (err_line) *err_line = line_number;
            goto cleanup;
        }
        result = point_cloud_push(&pt, &tmp);
        if (result != POINT_CLOUD_OK) {
            if (err_line) *err_line = line_number;
            goto cleanup;
        }
    }

cleanup:
    if (ferror(fp)) result = POINT_CLOUD_IO_ERROR;
    free(line);
    if (fclose(fp) != 0 && result == POINT_CLOUD_OK) {
        result = POINT_CLOUD_IO_ERROR;
    }
    if (result != POINT_CLOUD_OK) {
        point_cloud_destroy(&tmp);
        return result;
    }

    point_cloud_destroy(cloud);
    *cloud = tmp;
    return POINT_CLOUD_OK;
}

void point_cloud_destroy(point_cloud* cloud) {
    if (!cloud) return;
    if (cloud->points) free(cloud->points);
    cloud->points = NULL;
    cloud->count = 0;
    cloud->capacity = 0;
}
