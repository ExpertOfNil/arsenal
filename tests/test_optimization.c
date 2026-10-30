#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "optimization.h"
#include "test_check.h"

typedef struct ObjectiveContext {
    double target[2];
    size_t calls;

    /* Inject an exceptional result on the second invocation. */
    OptimizationEval second_result;
    int second_nonfinite;
} ObjectiveContext;

static OptimizationEval quadratic(
    const double* params, size_t param_count, void* user_data, double* out_value
) {
    ObjectiveContext* context = user_data;
    TEST_CHECK(param_count > 0 && param_count <= 2);

    ++context->calls;

    if (context->calls == 2) {
        if (context->second_result != OPTIMIZATION_EVAL_OK) {
            return context->second_result;
        }

        if (context->second_nonfinite) {
            *out_value = NAN;
            return OPTIMIZATION_EVAL_OK;
        }
    }

    double value = 0.0;

    for (size_t i = 0; i < param_count; ++i) {
        const double residual = params[i] - context->target[i];
        value += residual * residual;
    }

    *out_value = value;
    return OPTIMIZATION_EVAL_OK;
}

static void test_quadratic_in_place(void) {
    ObjectiveContext context = {
        .target = {3.0, -2.0},
    };
    double params[2] = {0.0, 0.0};
    const double steps[2] = {1.0, 1.0};
    NelderMeadResult result = {0};

    const OptimizationStatus status = nelder_mead_minimize(
        quadratic, &context, 2, params, steps, NULL, params, &result
    );

    TEST_CHECK(status == OPTIMIZATION_OK);
    TEST_CHECK(fabs(params[0] - 3.0) < 1e-5);
    TEST_CHECK(fabs(params[1] + 2.0) < 1e-5);
    TEST_CHECK(result.objective_value < 1e-10);
    TEST_CHECK(result.evaluations == context.calls);
    TEST_CHECK(result.iterations > 0);
}

static void test_budget_during_expansion(void) {
    ObjectiveContext context = {
        .target = {3.0, 0.0},
    };
    const double initial[1] = {0.0};
    const double steps[1] = {1.0};
    double output[1] = {-99.0};
    NelderMeadResult result = {0};

    NelderMeadOptions options = nelder_mead_options_default();
    options.max_evaluations = 3;

    const OptimizationStatus status = nelder_mead_minimize(
        quadratic, &context, 1, initial, steps, &options, output, &result
    );

    TEST_CHECK(status == OPTIMIZATION_EVALUATION_LIMIT);
    TEST_CHECK(context.calls == 3);
    TEST_CHECK(result.evaluations == 3);
    TEST_CHECK(result.iterations == 0);
    TEST_CHECK(output[0] == 2.0);
    TEST_CHECK(result.objective_value == 1.0);
}

static void test_iteration_limit(void) {
    ObjectiveContext context = {
        .target = {3.0, -2.0},
    };
    const double initial[2] = {0.0, 0.0};
    const double steps[2] = {1.0, 1.0};
    double output[2] = {-99.0, -99.0};
    NelderMeadResult result = {0};

    NelderMeadOptions options = nelder_mead_options_default();
    options.max_iterations = 1;

    const OptimizationStatus status = nelder_mead_minimize(
        quadratic, &context, 2, initial, steps, &options, output, &result
    );

    TEST_CHECK(status == OPTIMIZATION_ITERATION_LIMIT);
    TEST_CHECK(result.iterations == 1);
    TEST_CHECK(result.evaluations == context.calls);
    TEST_CHECK(isfinite(result.objective_value));
    TEST_CHECK(result.objective_value <= 8.0);
}

static void test_objective_failures(void) {
    for (int mode = 0; mode < 3; ++mode) {
        ObjectiveContext context = {
            .target = {3.0, -2.0},
            .second_result = mode == 0   ? OPTIMIZATION_EVAL_FAILED
                             : mode == 1 ? OPTIMIZATION_EVAL_REJECTED
                                         : OPTIMIZATION_EVAL_OK,
            .second_nonfinite = mode == 2,
        };

        const double initial[2] = {0.0, 0.0};
        const double steps[2] = {1.0, 1.0};
        double output[2] = {-99.0, -98.0};
        NelderMeadResult result = {
            .iterations = 123,
            .evaluations = 456,
            .objective_value = -789.0,
        };

        const OptimizationStatus status = nelder_mead_minimize(
            quadratic, &context, 2, initial, steps, NULL, output, &result
        );

        TEST_CHECK(
            status == (mode == 2 ? OPTIMIZATION_NUMERICAL_FAILURE
                                 : OPTIMIZATION_OBJECTIVE_FAILURE)
        );

        TEST_CHECK(context.calls == 2);
        TEST_CHECK(output[0] == -99.0);
        TEST_CHECK(output[1] == -98.0);
        TEST_CHECK(result.iterations == 123);
        TEST_CHECK(result.evaluations == 456);
        TEST_CHECK(result.objective_value == -789.0);
    }
}

int main(void) {
    test_quadratic_in_place();
    test_budget_during_expansion();
    test_iteration_limit();
    test_objective_failures();

    puts("optimization tests passed");
    return 0;
}
