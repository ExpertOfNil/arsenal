#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <stddef.h>

#include "glmath.h"
#include "optimization.h"
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
    ALIGNMENT_ALLOCATION_FAILURE,
    ALIGNMENT_OBJECTIVE_FAILURE,
    ALIGNMENT_ITERATION_LIMIT,
    ALIGNMENT_EVALUATION_LIMIT,
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

typedef OptimizationEval (*AlignmentObjective)(
    const PointCloudd* source,
    const PointCloudd* target,
    const AlignmentTransform* transform,
    void* user_data,
    double* out_value
);

typedef struct AlignmentNelderMeadOptions {
    NelderMeadOptions optimizer;
    vec3d rotation_steps;
    vec3d translation_steps;
} AlignmentNelderMeadOptions;

AlignmentStatus point_cloud_align_kabsch(
    const PointCloudd* source,
    const PointCloudd* target,
    AlignmentTransform* out
);

AlignmentStatus point_cloud_align_normal_equations(
    const PointCloudd* source,
    const PointCloudd* target,
    AlignmentTransform* out
);

AlignmentNelderMeadOptions alignment_nelder_mead_options_default(void);

AlignmentStatus point_cloud_align_nelder_mead(
    const PointCloudd* source,
    const PointCloudd* target,
    const AlignmentTransform* initial_transform,
    AlignmentObjective objective,
    void* user_data,
    const AlignmentNelderMeadOptions* options,
    AlignmentTransform* out,
    NelderMeadResult* out_result
);

AlignmentStatus point_cloud_assess_alignment(
    const PointCloudd* source,
    const PointCloudd* target,
    const AlignmentTransform* transform,
    AlignmentFitMetrics* out
);

AlignmentStatus alignment_compare_transforms(
    const AlignmentTransform* estimated,
    const AlignmentTransform* expected,
    AlignmentTransformError* out
);

#endif /* ALIGNMENT_H */
