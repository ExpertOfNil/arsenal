#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <stdint.h>
#include <stddef.h>

#include "glmath.h"

#define POINT_CLOUD_DEFAULT_CAPACITY 256

typedef struct point3d {
    uint64_t id;
    vec3d position;
} point3d;

typedef struct point_cloud {
    point3d* points;
    size_t count;
    size_t capacity;
} point_cloud;

typedef enum {
    POINT_CLOUD_OK = 0,
    POINT_CLOUD_INVALID_ARGUMENT,
    POINT_CLOUD_IO_ERROR,
    POINT_CLOUD_FORMAT_ERROR,
    POINT_CLOUD_RANGE_ERROR,
    POINT_CLOUD_ALLOCATION_ERROR
} point_cloud_status;

point_cloud_status point_cloud_reserve(point_cloud* cloud, size_t size);
point_cloud_status point_cloud_push(const point3d* pt, point_cloud* cloud);
point_cloud_status point_cloud_from_csv(
    const char* path, point_cloud* cloud, size_t* err_line
);
void point_cloud_reset(point_cloud* cloud);
void point_cloud_destroy(point_cloud* cloud);

#endif /* POINT_CLOUD_H */
