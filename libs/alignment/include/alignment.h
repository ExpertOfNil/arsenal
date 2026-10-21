#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <stddef.h>

#include "glmath.h"
#include "point_cloud.h"

typedef struct AlignmentTransform {
    mat3d rotation;
    vec3d translation;
} AlignmentTransform;

typedef enum AlignmentStatus {
    ALIGNMENT_OK = 0,
    ALIGNMENT_INVALID_ARGUMENT,
    ALIGNMENT_COUNT_MISMATCH,
    ALIGNMENT_ID_MISMATCH,
    ALIGNMENT_INSUFFICIENT_POINTS,
    ALIGNMENT_DEGENERATE_CONFIGURATION,
    ALIGNMENT_NUMERICAL_FAILURE,
} AlignmentStatus;

typedef struct AlignmentFitMetrics {
    size_t correspondence_count;
    double rmse;
    double orthogonality_error;
    double determinant;
} AlignmentFitMetrics;

typedef struct AlignmentTransformError {
    double rotation_radians;
    double translation;
} AlignmentTransformError;

AlignmentStatus point_cloud_align_kabsch(
    const PointCloud* source, const PointCloud* target, AlignmentTransform* out
);

AlignmentStatus point_cloud_align_normal_equations(
    const PointCloud* source, const PointCloud* target, AlignmentTransform* out
);

AlignmentStatus point_cloud_assess_alignment(
    const PointCloud* source,
    const PointCloud* target,
    const AlignmentTransform* transform,
    AlignmentFitMetrics* out
);

AlignmentStatus alignment_compare_transforms(
    const AlignmentTransform* estimated,
    const AlignmentTransform* expected,
    AlignmentTransformError* out
);

#endif /* ALIGNMENT_H */
