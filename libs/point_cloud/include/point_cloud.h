#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <stddef.h>
#include <stdint.h>

#include "glmath.h"

#define POINT_CLOUD_DEFAULT_CAPACITY 256

typedef struct Point3f {
    uint64_t id;
    vec3f position;
} Point3f;
typedef struct Point3d {
    uint64_t id;
    vec3d position;
} Point3d;

typedef struct PointCloudf {
    Point3f* points;
    size_t count;
    size_t capacity;
} PointCloudf;
typedef struct PointCloudd {
    Point3d* points;
    size_t count;
    size_t capacity;
} PointCloudd;

typedef enum {
    POINT_CLOUD_OK = 0,
    POINT_CLOUD_INVALID_ARGUMENT,
    POINT_CLOUD_IO_ERROR,
    POINT_CLOUD_FORMAT_ERROR,
    POINT_CLOUD_RANGE_ERROR,
    POINT_CLOUD_ALLOCATION_ERROR
} PointCloudStatus;

PointCloudStatus point_cloud_d_to_f(const PointCloudd* src, PointCloudf* dst);
PointCloudStatus point_cloud_f_to_d(const PointCloudf* src, PointCloudd* dst);

PointCloudStatus point_cloudf_init(PointCloudf* cloud);
PointCloudStatus point_cloudf_reserve(PointCloudf* cloud, size_t size);
PointCloudStatus point_cloudf_push(const Point3f* pt, PointCloudf* cloud);
PointCloudStatus point_cloudf_from_csv(
    const char* path, PointCloudf* cloud, size_t* err_line
);
PointCloudStatus point_cloudf_to_csv(
    const char* path, const PointCloudf* cloud
);
void point_cloudf_reset(PointCloudf* cloud);
void point_cloudf_destroy(PointCloudf* cloud);

PointCloudStatus point_cloudd_init(PointCloudd* cloud);
PointCloudStatus point_cloudd_reserve(PointCloudd* cloud, size_t size);
PointCloudStatus point_cloudd_push(const Point3d* pt, PointCloudd* cloud);
PointCloudStatus point_cloudd_from_csv(
    const char* path, PointCloudd* cloud, size_t* err_line
);
PointCloudStatus point_cloudd_to_csv(
    const char* path, const PointCloudd* cloud
);
void point_cloudd_reset(PointCloudd* cloud);
void point_cloudd_destroy(PointCloudd* cloud);

#endif /* POINT_CLOUD_H */
