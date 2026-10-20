#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <stddef.h>
#include <stdint.h>

#include "glmath.h"

#define POINT_CLOUD_DEFAULT_CAPACITY 256

typedef struct point3f {
    uint64_t id;
    vec3f position;
} point3f;
typedef struct point3d {
    uint64_t id;
    vec3d position;
} point3d;

typedef struct point_cloudf {
    point3f* points;
    size_t count;
    size_t capacity;
} point_cloudf;
typedef struct point_cloudd {
    point3d* points;
    size_t count;
    size_t capacity;
} point_cloudd;

typedef enum {
    POINT_CLOUD_OK = 0,
    POINT_CLOUD_INVALID_ARGUMENT,
    POINT_CLOUD_IO_ERROR,
    POINT_CLOUD_FORMAT_ERROR,
    POINT_CLOUD_RANGE_ERROR,
    POINT_CLOUD_ALLOCATION_ERROR
} point_cloud_status;

point_cloud_status point_cloudf_reserve(point_cloudf* cloud, size_t size);
point_cloud_status point_cloudf_push(const point3f* pt, point_cloudf* cloud);
point_cloud_status point_cloudf_from_csv(
    const char* path, point_cloudf* cloud, size_t* err_line
);
void point_cloudf_reset(point_cloudf* cloud);
void point_cloudf_destroy(point_cloudf* cloud);

point_cloud_status point_cloudd_reserve(point_cloudd* cloud, size_t size);
point_cloud_status point_cloudd_push(const point3d* pt, point_cloudd* cloud);
point_cloud_status point_cloudd_from_csv(
    const char* path, point_cloudd* cloud, size_t* err_line
);
void point_cloudd_reset(point_cloudd* cloud);
void point_cloudd_destroy(point_cloudd* cloud);

#endif /* POINT_CLOUD_H */
