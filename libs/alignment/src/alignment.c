#include "alignment.h"

#include <float.h>
#include <math.h>

#include "glmath.h"
#include "optimization.h"

typedef struct AlignmentNelderMeadContext {
    const PointCloudd* source;
    const PointCloudd* target;

    AlignmentTransform initial;

    AlignmentObjective objective;
    void* user_data;

    AlignmentStatus status;
} AlignmentNelderMeadContext;

static int point_is_finite(const Point3d* point) {
    return isfinite(point->position[0]) && isfinite(point->position[1]) &&
           isfinite(point->position[2]);
}

static AlignmentStatus validate_correspondences(
    const PointCloudd* source, const PointCloudd* target, size_t min_count
) {
    if (source == NULL || target == NULL) return ALIGNMENT_INVALID_ARGUMENT;
    if ((source->count > 0 && source->points == NULL) ||
        (target->count > 0 && target->points == NULL)) {
        return ALIGNMENT_INVALID_ARGUMENT;
    }
    if (source->count > source->capacity || target->count > target->capacity) {
        return ALIGNMENT_INVALID_ARGUMENT;
    }
    if (source->count != target->count) return ALIGNMENT_COUNT_MISMATCH;
    if (source->count < min_count) return ALIGNMENT_INSUFFICIENT_POINTS;

    for (size_t i = 0; i < source->count; ++i) {
        if (source->points[i].id != target->points[i].id) {
            return ALIGNMENT_ID_MISMATCH;
        }

        if (!point_is_finite(&source->points[i]) ||
            !point_is_finite(&target->points[i])) {
            return ALIGNMENT_INVALID_ARGUMENT;
        }
    }
    return ALIGNMENT_OK;
}

static void calculate_centroid(const PointCloudd* cloud, vec3d centroid) {
    centroid[0] = 0.0;
    centroid[1] = 0.0;
    centroid[2] = 0.0;

    for (size_t i = 0; i < cloud->count; ++i) {
        centroid[0] += cloud->points[i].position[0];
        centroid[1] += cloud->points[i].position[1];
        centroid[2] += cloud->points[i].position[2];
    }

    const double inverse_count = 1.0 / (double)cloud->count;
    centroid[0] *= inverse_count;
    centroid[1] *= inverse_count;
    centroid[2] *= inverse_count;
}

static void calculate_cross_covariance(
    const PointCloudd* source,
    const PointCloudd* target,
    const vec3d source_centroid,
    const vec3d target_centroid,
    mat3d covariance
) {
    for (size_t i = 0; i < 9; ++i) covariance[i] = 0.0;

    for (size_t i = 0; i < source->count; ++i) {
        const double* source_pos = source->points[i].position;
        const double* target_pos = target->points[i].position;

        for (size_t col = 0; col < 3; ++col) {
            const double source_coord = source_pos[col] - source_centroid[col];

            for (size_t row = 0; row < 3; ++row) {
                const double target_coord =
                    target_pos[row] - target_centroid[row];

                covariance[row + 3 * col] += target_coord * source_coord;
            }
        }
    }
}

static int calculate_regression_matrices(
    const PointCloudd* source, const PointCloudd* target, mat4d ptp, mat4d ptq
) {
    if (source == NULL || target == NULL || ptp == NULL || ptq == NULL) {
        return -1;
    }
    for (size_t i = 0; i < 16; ++i) {
        ptp[i] = 0.0;
        ptq[i] = 0.0;
    }

    for (size_t i = 0; i < source->count; ++i) {
        const vec4d p = {
            source->points[i].position[0],
            source->points[i].position[1],
            source->points[i].position[2],
            1.0
        };
        const vec4d q = {
            target->points[i].position[0],
            target->points[i].position[1],
            target->points[i].position[2],
            1.0
        };
        for (size_t col = 0; col < 4; ++col) {
            for (size_t row = 0; row < 4; ++row) {
                const size_t index = row + 4 * col;
                ptp[index] += p[row] * p[col];
                ptq[index] += p[row] * q[col];
            }
        }
    }

    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(ptp[i]) || !isfinite(ptq[i])) {
            return -1;
        }
    }

    return 0;
}

static int accumulate_scaled_square(
    double value, double* scale, double* sum_of_squares
) {
    if (!isfinite(value)) return -1;

    const double mag = fabs(value);
    if (mag == 0.0) return 0;

    if (*scale < mag) {
        const double ratio = *scale / mag;
        *sum_of_squares = 1.0 + *sum_of_squares * ratio * ratio;
        *scale = mag;
    } else {
        const double ratio = mag / *scale;
        *sum_of_squares += ratio * ratio;
    }

    return 0;
}

static AlignmentStatus alignment_transform_from_params(
    const double params[6],
    const AlignmentTransform* initial,
    AlignmentTransform* out
) {
    for (size_t i = 0; i < 6; ++i) {
        if (!isfinite(params[i])) {
            return ALIGNMENT_NUMERICAL_FAILURE;
        }
    }

    const vec3d omega = {params[0], params[1], params[2]};

    mat3d delta_rotation;
    if (mat3d_from_rotation_vector(omega, delta_rotation) != 0) {
        return ALIGNMENT_NUMERICAL_FAILURE;
    }

    AlignmentTransform result;
    mat3d_mul(delta_rotation, initial->rotation, result.rotation);

    for (size_t i = 0; i < 3; ++i) {
        result.translation[i] = initial->translation[i] + params[3 + i];
        if (!isfinite(result.translation[i])) {
            return ALIGNMENT_NUMERICAL_FAILURE;
        }
    }

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(result.rotation[i])) {
            return ALIGNMENT_NUMERICAL_FAILURE;
        }
    }

    *out = result;
    return ALIGNMENT_OK;
}

static AlignmentStatus alignment_prepare_initial_transform(
    const AlignmentTransform* initial, AlignmentTransform* out
) {
    AlignmentTransform result;
    if (initial == NULL) {
        mat3d_identity(result.rotation);
        for (size_t i = 0; i < 3; ++i) {
            result.translation[i] = 0.0;
        }

        *out = result;
        return ALIGNMENT_OK;
    }

    result = *initial;
    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(result.rotation[i])) {
            return ALIGNMENT_INVALID_ARGUMENT;
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        if (!isfinite(result.translation[i])) {
            return ALIGNMENT_INVALID_ARGUMENT;
        }
    }

    /*
     * Absolute, nondimensional tolerance for double-precision rotation
     * matrices. This is input validation not optimization stopping tolerance.
     */
    const double rotation_tolerance = 1e-10;
    mat3d transpose;
    mat3d gram;

    mat3d_transpose(result.rotation, transpose);
    mat3d_mul(transpose, result.rotation, gram);

    for (size_t col = 0; col < 3; ++col) {
        for (size_t row = 0; row < 3; ++row) {
            const size_t index = row + 3 * col;
            const double expected = row == col ? 1.0 : 0.0;

            if (!isfinite(gram[index]) ||
                fabs(gram[index] - expected) > rotation_tolerance) {
                return ALIGNMENT_INVALID_ARGUMENT;
            }
        }
    }

    const double determinant = mat3d_det(result.rotation);

    if (!isfinite(determinant) ||
        fabs(determinant - 1.0) > rotation_tolerance) {
        return ALIGNMENT_INVALID_ARGUMENT;
    }

    *out = result;
    return ALIGNMENT_OK;
}

static OptimizationEval alignment_nelder_mead_evaluate(
    const double* params, size_t param_count, void* user_data, double* out_value
) {
    AlignmentNelderMeadContext* context = user_data;

    if (param_count != 6) {
        context->status = ALIGNMENT_NUMERICAL_FAILURE;
        return OPTIMIZATION_EVAL_FAILED;
    }

    AlignmentTransform transform;

    context->status =
        alignment_transform_from_params(params, &context->initial, &transform);
    if (context->status != ALIGNMENT_OK) {
        return OPTIMIZATION_EVAL_FAILED;
    }

    return context->objective(
        context->source,
        context->target,
        &transform,
        context->user_data,
        out_value
    );
}

static AlignmentStatus alignment_status_from_optimization(
    OptimizationStatus status
) {
    switch (status) {
        case OPTIMIZATION_OK:
            return ALIGNMENT_OK;
        case OPTIMIZATION_INVALID_ARGUMENT:
            return ALIGNMENT_INVALID_ARGUMENT;
        case OPTIMIZATION_ALLOCATION_FAILURE:
            return ALIGNMENT_ALLOCATION_FAILURE;
        case OPTIMIZATION_OBJECTIVE_FAILURE:
            return ALIGNMENT_OBJECTIVE_FAILURE;
        case OPTIMIZATION_NUMERICAL_FAILURE:
            return ALIGNMENT_NUMERICAL_FAILURE;
        case OPTIMIZATION_ITERATION_LIMIT:
            return ALIGNMENT_ITERATION_LIMIT;
        case OPTIMIZATION_EVALUATION_LIMIT:
            return ALIGNMENT_EVALUATION_LIMIT;
        default:
            return ALIGNMENT_NUMERICAL_FAILURE;
    }
}

/* Public Functions */

AlignmentStatus point_cloud_align_kabsch(
    const PointCloudd* source,
    const PointCloudd* target,
    AlignmentTransform* out
) {
    if (out == NULL) return ALIGNMENT_INVALID_ARGUMENT;

    AlignmentStatus status = validate_correspondences(source, target, 3);
    if (status != ALIGNMENT_OK) return status;

    vec3d source_centroid;
    vec3d target_centroid;
    mat3d covariance;

    calculate_centroid(source, source_centroid);
    calculate_centroid(target, target_centroid);
    calculate_cross_covariance(
        source, target, source_centroid, target_centroid, covariance
    );

    mat3d u;
    mat3d vt;
    vec3d singular_values;

    int svd_status = mat3d_svd(covariance, u, singular_values, vt);
    if (svd_status != 0) return ALIGNMENT_NUMERICAL_FAILURE;

    if (!isfinite(singular_values[0]) || !isfinite(singular_values[1]) ||
        !isfinite(singular_values[2])) {
        return ALIGNMENT_NUMERICAL_FAILURE;
    }

    // Kabsch requires rank >= 2
    if (singular_values[0] <= 0.0) return ALIGNMENT_DEGENERATE_CONFIGURATION;
    const double rank_tolerance = 3.0 * DBL_EPSILON * singular_values[0];

    if (singular_values[1] <= rank_tolerance) {
        return ALIGNMENT_DEGENERATE_CONFIGURATION;
    }

    // Check for reflection
    const double orientation = mat3d_det(u) * mat3d_det(vt);
    if (!isfinite(orientation)) return ALIGNMENT_NUMERICAL_FAILURE;

    if (orientation < 0.0) {
        u[6] = -u[6];
        u[7] = -u[7];
        u[8] = -u[8];
    }

    AlignmentTransform result;
    mat3d_mul(u, vt, result.rotation);

    // target_centroid = R * source_centroid + translation
    vec3d rotated_source_centroid;
    mat3d_mul_vec3d(result.rotation, source_centroid, rotated_source_centroid);
    vec3d_sub(target_centroid, rotated_source_centroid, result.translation);

    *out = result;
    return ALIGNMENT_OK;
}

AlignmentStatus point_cloud_align_normal_equations(
    const PointCloudd* source,
    const PointCloudd* target,
    AlignmentTransform* out
) {
    if (out == NULL) return ALIGNMENT_INVALID_ARGUMENT;

    AlignmentStatus status = validate_correspondences(source, target, 4);
    if (status != ALIGNMENT_OK) return status;

    mat4d ptp;
    mat4d ptq;
    if (calculate_regression_matrices(source, target, ptp, ptq) != 0) {
        return ALIGNMENT_NUMERICAL_FAILURE;
    }

    // Calculate (P^TP)^{-1}
    mat4d inverse_ptp;
    const int inverse_status = mat4d_inverse_cholesky(ptp, inverse_ptp);
    if (inverse_status > 0) return ALIGNMENT_DEGENERATE_CONFIGURATION;
    if (inverse_status != 0) return ALIGNMENT_NUMERICAL_FAILURE;

    // Calculate B = (P^TP)^{-1} P^TQ
    mat4d beta;
    mat4d_mul(inverse_ptp, ptq, beta);
    for (size_t i = 0; i < 16; ++i) {
        if (!isfinite(beta[i])) return ALIGNMENT_NUMERICAL_FAILURE;
    }

    /* Convert from row-observation regression coefficients to column-vector
     * transform convention:
     *
     * Q ~= PB
     * q ~= B^Tp
     */
    mat4d raw_transform;
    mat4d_transpose(beta, raw_transform);

    // Apply Gram-Schmidt
    mat4d rigid_transform;
    if (mat4d_orthonormalize(raw_transform, rigid_transform) != 0) {
        return ALIGNMENT_DEGENERATE_CONFIGURATION;
    }

    AlignmentTransform result;
    result.rotation[0] = rigid_transform[0];
    result.rotation[1] = rigid_transform[1];
    result.rotation[2] = rigid_transform[2];

    result.rotation[3] = rigid_transform[4];
    result.rotation[4] = rigid_transform[5];
    result.rotation[5] = rigid_transform[6];

    result.rotation[6] = rigid_transform[8];
    result.rotation[7] = rigid_transform[9];
    result.rotation[8] = rigid_transform[10];

    result.translation[0] = rigid_transform[12];
    result.translation[1] = rigid_transform[13];
    result.translation[2] = rigid_transform[14];

    *out = result;
    return ALIGNMENT_OK;
}

AlignmentStatus point_cloud_assess_alignment(
    const PointCloudd* source,
    const PointCloudd* target,
    const AlignmentTransform* transform,
    AlignmentFitMetrics* out
) {
    if (transform == NULL || out == NULL) return ALIGNMENT_INVALID_ARGUMENT;

    AlignmentStatus status = validate_correspondences(source, target, 1);
    if (status != ALIGNMENT_OK) return status;

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(transform->rotation[i])) {
            return ALIGNMENT_INVALID_ARGUMENT;
        }
    }
    for (size_t i = 0; i < 3; ++i) {
        if (!isfinite(transform->translation[i])) {
            return ALIGNMENT_INVALID_ARGUMENT;
        }
    }

    double residual_scale = 0.0;
    double residual_sum_of_squares = 1.0;

    for (size_t i = 0; i < source->count; ++i) {
        vec3d xformed_position;

        mat3d_mul_vec3d(
            transform->rotation, source->points[i].position, xformed_position
        );

        for (size_t axis = 0; axis < 3; ++axis) {
            xformed_position[axis] += transform->translation[axis];

            const double residual =
                xformed_position[axis] - target->points[i].position[axis];
            int result = accumulate_scaled_square(
                residual, &residual_scale, &residual_sum_of_squares
            );
            if (result != 0) return ALIGNMENT_NUMERICAL_FAILURE;
        }
    }

    double rmse = 0.0;

    if (residual_scale != 0.0) {
        rmse = residual_scale *
               sqrt(residual_sum_of_squares / (double)source->count);
        if (!isfinite(rmse)) return ALIGNMENT_NUMERICAL_FAILURE;
    }

    mat3d rotation_transpose;
    mat3d rotation_gram;

    mat3d_transpose(transform->rotation, rotation_transpose);
    mat3d_mul(rotation_transpose, transform->rotation, rotation_gram);

    double ortho_scale = 0.0;
    double ortho_sum_of_squares = 1.0;

    for (size_t col = 0; col < 3; ++col) {
        for (size_t row = 0; row < 3; ++row) {
            const double expected = row == col ? 1.0 : 0.0;
            const double diff = rotation_gram[row + 3 * col] - expected;

            int result = accumulate_scaled_square(
                diff, &ortho_scale, &ortho_sum_of_squares
            );
            if (result != 0) return ALIGNMENT_NUMERICAL_FAILURE;
        }
    }

    double ortho_error = 0.0;
    if (ortho_scale != 0.0) {
        ortho_error = ortho_scale * sqrt(ortho_sum_of_squares);
        if (!isfinite(ortho_error)) return ALIGNMENT_NUMERICAL_FAILURE;
    }

    const double det = mat3d_det(transform->rotation);
    if (!isfinite(det)) return ALIGNMENT_NUMERICAL_FAILURE;

    const AlignmentFitMetrics result = {
        .correspondence_count = source->count,
        .rmse = rmse,
        .orthogonality_error = ortho_error,
        .determinant = det,
    };

    *out = result;
    return ALIGNMENT_OK;
}

AlignmentStatus alignment_compare_transforms(
    const AlignmentTransform* estimated,
    const AlignmentTransform* expected,
    AlignmentTransformError* out
) {
    if (estimated == NULL || expected == NULL || out == NULL) {
        return ALIGNMENT_INVALID_ARGUMENT;
    }

    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(estimated->rotation[i]) ||
            !isfinite(expected->rotation[i])) {
            return ALIGNMENT_INVALID_ARGUMENT;
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        if (!isfinite(estimated->translation[i]) ||
            !isfinite(expected->translation[i])) {
            return ALIGNMENT_INVALID_ARGUMENT;
        }
    }

    // Translation error: ||t_estimated - t_expected||
    double t_scale = 0.0;
    double t_sum_of_squares = 1.0;
    for (size_t axis = 0; axis < 3; ++axis) {
        const double diff =
            estimated->translation[axis] - expected->translation[axis];
        int result =
            accumulate_scaled_square(diff, &t_scale, &t_sum_of_squares);
        if (result != 0) return ALIGNMENT_NUMERICAL_FAILURE;
    }

    double translation_error = 0.0;
    if (t_scale != 0.0) {
        translation_error = t_scale * sqrt(t_sum_of_squares);
        if (!isfinite(translation_error)) return ALIGNMENT_NUMERICAL_FAILURE;
    }

    // Relative rotation: R_delta = R_expected^T R_estimated
    mat3d expected_transpose;
    mat3d relative_rotation;

    mat3d_transpose(expected->rotation, expected_transpose);
    mat3d_mul(expected_transpose, estimated->rotation, relative_rotation);
    for (size_t i = 0; i < 9; ++i) {
        if (!isfinite(relative_rotation[i])) return ALIGNMENT_NUMERICAL_FAILURE;
    }

    const double trace =
        relative_rotation[0] + relative_rotation[4] + relative_rotation[8];
    if (!isfinite(trace)) return ALIGNMENT_NUMERICAL_FAILURE;

    const double skew_x = relative_rotation[5] - relative_rotation[7];
    const double skew_y = relative_rotation[6] - relative_rotation[2];
    const double skew_z = relative_rotation[1] - relative_rotation[3];
    const double sine = 0.5 * hypot(hypot(skew_x, skew_y), skew_z);
    double cosine = 0.5 * (trace - 1.0);
    if (!isfinite(sine) || !isfinite(cosine))
        return ALIGNMENT_NUMERICAL_FAILURE;

    // Correct floating-point rounding
    if (cosine < -1.0) cosine = -1.0;
    if (cosine > 1.0) cosine = 1.0;

    const double rotation_error = atan2(sine, cosine);
    if (!isfinite(rotation_error)) return ALIGNMENT_NUMERICAL_FAILURE;

    const AlignmentTransformError result = {
        .rotation_radians = rotation_error,
        .translation = translation_error,
    };

    *out = result;
    return ALIGNMENT_OK;
}

AlignmentNelderMeadOptions alignment_nelder_mead_options_default(void) {
    return (AlignmentNelderMeadOptions){
        .optimizer = nelder_mead_options_default(),
        .rotation_steps = {0.05, 0.05, 0.05},
        .translation_steps = {1.0, 1.0, 1.0},
    };
}

AlignmentStatus point_cloud_align_nelder_mead(
    const PointCloudd* source,
    const PointCloudd* target,
    const AlignmentTransform* initial_transform,
    AlignmentObjective objective,
    void* user_data,
    const AlignmentNelderMeadOptions* options,
    AlignmentTransform* out,
    NelderMeadResult* out_result
) {
    if (objective == NULL || out == NULL) {
        return ALIGNMENT_INVALID_ARGUMENT;
    }

    AlignmentStatus status = validate_correspondences(source, target, 3);
    if (status != ALIGNMENT_OK) {
        return status;
    }

    const AlignmentNelderMeadOptions opts =
        options != NULL ? *options : alignment_nelder_mead_options_default();

    AlignmentNelderMeadContext context = {
        .source = source,
        .target = target,
        .objective = objective,
        .user_data = user_data,
        .status = ALIGNMENT_OK,
    };
    status = alignment_prepare_initial_transform(
        initial_transform, &context.initial
    );
    if (status != ALIGNMENT_OK) {
        return status;
    }

    const double initial_params[6] = {0};
    const double initial_steps[6] = {
        opts.rotation_steps[0],
        opts.rotation_steps[1],
        opts.rotation_steps[2],
        opts.translation_steps[0],
        opts.translation_steps[1],
        opts.translation_steps[2],
    };
    double best_params[6];
    NelderMeadResult result;

    const OptimizationStatus optimization_status = nelder_mead_minimize(
        alignment_nelder_mead_evaluate,
        &context,
        6,
        initial_params,
        initial_steps,
        &opts.optimizer,
        best_params,
        &result
    );
    if (context.status != ALIGNMENT_OK) {
        return context.status;
    }
    status = alignment_status_from_optimization(optimization_status);
    if (status != ALIGNMENT_OK && status != ALIGNMENT_ITERATION_LIMIT &&
        status != ALIGNMENT_EVALUATION_LIMIT) {
        return status;
    }

    AlignmentTransform transform;
    const AlignmentStatus conversion_status = alignment_transform_from_params(
        best_params, &context.initial, &transform
    );
    if (conversion_status != ALIGNMENT_OK) {
        return conversion_status;
    }

    *out = transform;
    if (out_result != NULL) {
        *out_result = result;
    }

    return status;
}
