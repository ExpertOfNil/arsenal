#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <stddef.h>

typedef enum OptimizationEval {
    OPTIMIZATION_EVAL_OK = 0,
    OPTIMIZATION_EVAL_REJECTED,
    OPTIMIZATION_EVAL_FAILED,
} OptimizationEval;

typedef OptimizationEval (*OptimizationObjective)(
    const double* params, size_t param_count, void* user_data, double* out_value
);

typedef enum OptimizationStatus {
    OPTIMIZATION_OK = 0,
    OPTIMIZATION_INVALID_ARGUMENT,
    OPTIMIZATION_ALLOCATION_FAILURE,
    OPTIMIZATION_OBJECTIVE_FAILURE,
    OPTIMIZATION_NUMERICAL_FAILURE,
    OPTIMIZATION_ITERATION_LIMIT,
    OPTIMIZATION_EVALUATION_LIMIT,
} OptimizationStatus;

typedef struct NelderMeadOptions {
    size_t max_iterations;
    size_t max_evaluations;

    double param_tolerance;
    double objective_tolerance_absolute;
    double objective_tolerance_relative;
} NelderMeadOptions;

typedef struct NelderMeadResult {
    size_t iterations;
    size_t evaluations;
    double objective_value;
} NelderMeadResult;

NelderMeadOptions nelder_mead_options_default(void);

OptimizationStatus nelder_mead_minimize(
    OptimizationObjective objective,
    void* user_data,
    size_t param_count,
    const double* initial_params,
    const double* initial_steps,
    const NelderMeadOptions* options,
    double* out_params,
    NelderMeadResult* out_result
);

#endif /* OPTIMIZATION_H */
