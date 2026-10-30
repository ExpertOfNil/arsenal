#include "alignment.h"
#include "test_check.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>

typedef struct WeightedObjectiveContext {
    const double* weights;
    size_t count;
    size_t calls;
} WeightedObjectiveContext;

static OptimizationEval weighted_squared_error(
    const PointCloudd* source,
    const PointCloudd* target,
    const AlignmentTransform* transform,
    void* user_data,
    double* out_value
) {
    WeightedObjectiveContext* context = user_data;

    TEST_CHECK(source->count == context->count);
    TEST_CHECK(target->count == context->count);
    ++context->calls;

    double value = 0.0;

    for (size_t i = 0; i < source->count; ++i) {
        vec3d position;
        mat3d_mul_vec3d(
            transform->rotation,
            source->points[i].position,
            position
        );

        for (size_t axis = 0; axis < 3; ++axis) {
            const double residual =
                position[axis] +
                transform->translation[axis] -
                target->points[i].position[axis];

            value += context->weights[i] * residual * residual;
        }
    }

    *out_value = value;
    return OPTIMIZATION_EVAL_OK;
}

static void test_known_transform_in_place(void) {
    Point3d source_points[] = {
        {.id = 10, .position = {0.0, 0.0, 0.0}},
        {.id = 20, .position = {1.0, 0.0, 0.0}},
        {.id = 30, .position = {0.0, 2.0, 0.0}},
        {.id = 40, .position = {0.0, 0.0, 3.0}},
        {.id = 50, .position = {-1.0, 0.5, 2.0}},
        {.id = 60, .position = {2.0, -1.0, 0.5}},
    };
    Point3d target_points[6] = {0};
    const double weights[6] = {1.0, 2.0, 0.5, 3.0, 1.5, 0.75};

    PointCloudd source = {
        .points = source_points,
        .count = 6,
        .capacity = 6,
    };
    PointCloudd target = {
        .points = target_points,
        .count = 6,
        .capacity = 6,
    };

    /*
     * Construct the expected Z rotation directly so generating
     * the target does not depend on rotation-vector conversion.
     */
    const double cosine = cos(0.35);
    const double sine = sin(0.35);

    const AlignmentTransform expected = {
        .rotation = {
            cosine, sine, 0.0,
            -sine, cosine, 0.0,
            0.0, 0.0, 1.0,
        },
        .translation = {0.4, -0.7, 1.2},
    };

    for (size_t i = 0; i < source.count; ++i) {
        target_points[i].id = source_points[i].id;

        mat3d_mul_vec3d(
            expected.rotation,
            source_points[i].position,
            target_points[i].position
        );

        for (size_t axis = 0; axis < 3; ++axis) {
            target_points[i].position[axis] +=
                expected.translation[axis];
        }
    }

    /* Start with a small X rotation and a different translation. */
    const double initial_cosine = cos(0.12);
    const double initial_sine = sin(0.12);

    AlignmentTransform estimated = {
        .rotation = {
            1.0, 0.0, 0.0,
            0.0, initial_cosine, initial_sine,
            0.0, -initial_sine, initial_cosine,
        },
        .translation = {0.1, -0.2, 0.3},
    };

    WeightedObjectiveContext context = {
        .weights = weights,
        .count = 6,
    };

    AlignmentNelderMeadOptions options =
        alignment_nelder_mead_options_default();

    options.optimizer.max_iterations = 10000;
    options.optimizer.max_evaluations = 50000;
    options.optimizer.param_tolerance = 1e-7;
    options.optimizer.objective_tolerance_absolute = 1e-14;

    for (size_t axis = 0; axis < 3; ++axis) {
        options.rotation_steps[axis] = 0.1;
        options.translation_steps[axis] = 0.25;
    }

    NelderMeadResult result;

    const AlignmentStatus status = point_cloud_align_nelder_mead(
        &source,
        &target,
        &estimated,
        weighted_squared_error,
        &context,
        &options,
        &estimated,
        &result
    );

    TEST_CHECK(status == ALIGNMENT_OK);
    TEST_CHECK(result.evaluations == context.calls);
    TEST_CHECK(result.iterations > 0);
    TEST_CHECK(result.objective_value < 1e-10);

    AlignmentTransformError error;
    TEST_CHECK(
        alignment_compare_transforms(
            &estimated, &expected, &error
        ) == ALIGNMENT_OK
    );
    TEST_CHECK(error.rotation_radians < 1e-5);
    TEST_CHECK(error.translation < 1e-5);

    AlignmentFitMetrics fit;
    TEST_CHECK(
        point_cloud_assess_alignment(
            &source, &target, &estimated, &fit
        ) == ALIGNMENT_OK
    );
    TEST_CHECK(fit.rmse < 1e-5);
    TEST_CHECK(fit.orthogonality_error < 1e-10);
    TEST_CHECK(fabs(fit.determinant - 1.0) < 1e-10);
}

int main(void) {
    test_known_transform_in_place();

    puts("alignment tests passed");
    return 0;
}
