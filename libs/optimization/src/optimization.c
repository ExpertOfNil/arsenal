#include "optimization.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct NelderMeadStore {
    double* storage;
    double* vertices;
    double* values;
    double* centroid;
    double* reflected;
    double* trial;
    double* steps;
    double* best;

    size_t param_count;
    size_t vertex_count;
} NelderMeadStore;

typedef struct NelderMeadRun {
    NelderMeadStore* store;
    OptimizationObjective objective;
    void* user_data;
    size_t max_evaluations;
    size_t evaluations;

    double best_value;
} NelderMeadRun;

typedef struct NelderMeadRanking {
    size_t best;
    size_t worst;
    size_t second_worst;
} NelderMeadRanking;

/* Private Functions */

static OptimizationStatus nelder_mead_store_init(
    size_t param_count, NelderMeadStore* out
) {
    if (out == NULL || param_count == 0) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    // Bound all counts by max doubles in size_t-sized allocation
    const size_t max_elements = SIZE_MAX / sizeof(double);
    if (param_count >= max_elements) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    const size_t vertex_count = param_count + 1;
    if (param_count > max_elements / vertex_count) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    const size_t vertex_coords_count = param_count * vertex_count;
    size_t total_element_count = vertex_coords_count;
    if (vertex_count > max_elements - total_element_count) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }
    total_element_count += vertex_count;

    if (param_count > (max_elements - total_element_count) / 5) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }
    total_element_count += 5 * param_count;

    double* storage = malloc(total_element_count * sizeof(double));
    if (storage == NULL) {
        return OPTIMIZATION_ALLOCATION_FAILURE;
    }

    NelderMeadStore store = {0};

    store.storage = storage;
    store.vertices = storage;
    store.values = store.vertices + vertex_coords_count;
    store.centroid = store.values + vertex_count;
    store.reflected = store.centroid + param_count;
    store.trial = store.reflected + param_count;
    store.steps = store.trial + param_count;
    store.best = store.steps + param_count;

    store.param_count = param_count;
    store.vertex_count = vertex_count;

    *out = store;
    return OPTIMIZATION_OK;
}

static OptimizationStatus nelder_mead_evaluate(
    NelderMeadRun* run, const double* params, double* out_value
) {
    if (run->evaluations >= run->max_evaluations) {
        return OPTIMIZATION_EVALUATION_LIMIT;
    }

    const size_t param_count = run->store->param_count;
    for (size_t i = 0; i < param_count; ++i) {
        if (!isfinite(params[i])) {
            return OPTIMIZATION_NUMERICAL_FAILURE;
        }
    }

    double value = NAN;

    ++run->evaluations;
    const OptimizationEval evaluation =
        run->objective(params, param_count, run->user_data, &value);

    switch (evaluation) {
        case OPTIMIZATION_EVAL_OK:
            if (!isfinite(value)) {
                return OPTIMIZATION_NUMERICAL_FAILURE;
            }

            if (value < run->best_value) {
                memcpy(run->store->best, params, param_count * sizeof(double));
                run->best_value = value;
            }

            *out_value = value;
            return OPTIMIZATION_OK;
        case OPTIMIZATION_EVAL_REJECTED:
            *out_value = INFINITY;
            return OPTIMIZATION_OK;
        case OPTIMIZATION_EVAL_FAILED:
        default:
            return OPTIMIZATION_OBJECTIVE_FAILURE;
    }
}

static OptimizationStatus nelder_mead_initialize_simplex(
    NelderMeadRun* run,
    const double* initial_params,
    const double* initial_steps
) {
    NelderMeadStore* store = run->store;
    const size_t param_count = store->param_count;
    const size_t vector_bytes = param_count * sizeof(double);

    for (size_t i = 0; i < param_count; ++i) {
        if (!isfinite(initial_params[i]) || !isfinite(initial_steps[i]) ||
            initial_steps[i] <= 0.0) {
            return OPTIMIZATION_INVALID_ARGUMENT;
        }

        const double displaced = initial_params[i] + initial_steps[i];
        if (!isfinite(displaced) || displaced == initial_params[i]) {
            return OPTIMIZATION_INVALID_ARGUMENT;
        }
    }

    memcpy(store->steps, initial_steps, vector_bytes);
    // Vector 0 is the initial param vector
    memcpy(store->vertices, initial_params, vector_bytes);

    // Each remaining vertex displaces 1 coordinate
    for (size_t i = 0; i < param_count; ++i) {
        double* vertex = store->vertices + (i + 1) * param_count;
        memcpy(vertex, initial_params, vector_bytes);
        vertex[i] += store->steps[i];
    }

    for (size_t i = 0; i < store->vertex_count; ++i) {
        const double* vertex = store->vertices + i * param_count;

        OptimizationStatus status =
            nelder_mead_evaluate(run, vertex, &store->values[i]);
        if (status != OPTIMIZATION_OK) {
            return status;
        }

        /*
         * The initial simplex must be entirely inside the objective's domain.
         * Rejected trial points will be allowed once the search begins
         */
        if (!isfinite(store->values[i])) {
            return OPTIMIZATION_OBJECTIVE_FAILURE;
        }
    }
    return OPTIMIZATION_OK;
}

static OptimizationStatus nelder_mead_validate_args(
    OptimizationObjective objective,
    size_t param_count,
    const double* initial_params,
    const double* initial_steps,
    const NelderMeadOptions* opts,
    double* out_params
) {
    if (objective == NULL || param_count == 0 || initial_params == NULL ||
        initial_steps == NULL || opts == NULL || out_params == NULL) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    if (opts->max_iterations == 0 || opts->max_evaluations == 0) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    if (!isfinite(opts->param_tolerance) || opts->param_tolerance <= 0.0) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    if (!isfinite(opts->objective_tolerance_absolute) ||
        opts->objective_tolerance_absolute < 0.0) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    if (!isfinite(opts->objective_tolerance_relative) ||
        opts->objective_tolerance_relative < 0.0) {
        return OPTIMIZATION_INVALID_ARGUMENT;
    }

    return OPTIMIZATION_OK;
}

static NelderMeadRanking nelder_mead_rank(const NelderMeadStore* store) {
    NelderMeadRanking ranking = {
        .best = 0,
        .worst = 0,
        .second_worst = 0,
    };

    for (size_t i = 1; i < store->vertex_count; ++i) {
        if (store->values[i] < store->values[ranking.best]) {
            ranking.best = i;
        }

        if (store->values[i] > store->values[ranking.worst]) {
            ranking.worst = i;
        }
    }

    ranking.second_worst = ranking.worst == 0 ? 1 : 0;

    for (size_t i = 0; i < store->vertex_count; ++i) {
        if (i == ranking.worst) {
            continue;
        }

        if (store->values[i] > store->values[ranking.second_worst]) {
            ranking.second_worst = i;
        }
    }

    return ranking;
}

static int nelder_mead_converged(
    const NelderMeadStore* store,
    const NelderMeadOptions* opts,
    size_t best_index
) {
    const size_t param_count = store->param_count;
    const double* best_vertex = store->vertices + best_index * param_count;
    const double best_value = store->values[best_index];
    if (!isfinite(best_value)) return 0;

    const double objective_limit =
        opts->objective_tolerance_absolute +
        opts->objective_tolerance_relative * fabs(best_value);

    for (size_t i = 0; i < store->vertex_count; ++i) {
        if (!isfinite(store->values[i])) {
            return 0;
        }

        const double objective_delta = fabs(store->values[i] - best_value);
        if (!isfinite(objective_delta) || objective_delta > objective_limit) {
            return 0;
        }

        const double* vertex = store->vertices + i * param_count;
        for (size_t j = 0; j < param_count; ++j) {
            const double param_delta = fabs(vertex[j] - best_vertex[j]);
            const double delta_norm = param_delta / store->steps[j];
            if (!isfinite(delta_norm) || delta_norm > opts->param_tolerance) {
                return 0;
            }
        }
    }
    return 1;
}

static OptimizationStatus nelder_mead_update(
    NelderMeadRun* run, NelderMeadRanking ranking
) {
    NelderMeadStore* store = run->store;
    const size_t param_count = store->param_count;
    const size_t vector_bytes = param_count * sizeof(double);
    const double inverse_count = 1.0 / (double)param_count;

    double* worst = store->vertices + ranking.worst * param_count;

    const double best_value = store->values[ranking.best];
    const double worst_value = store->values[ranking.worst];
    const double second_worst_value = store->values[ranking.second_worst];

    // Centroid of every vertex except the worst
    for (size_t i = 0; i < param_count; ++i) {
        double coord = 0.0;

        for (size_t j = 0; j < store->vertex_count; ++j) {
            if (j == ranking.worst) {
                continue;
            }

            coord += store->vertices[j * param_count + i] * inverse_count;
        }

        if (!isfinite(coord)) {
            return OPTIMIZATION_NUMERICAL_FAILURE;
        }

        store->centroid[i] = coord;
    }

    // Reflection: c + (c - worst)
    for (size_t i = 0; i < param_count; ++i) {
        store->reflected[i] =
            store->centroid[i] + (store->centroid[i] - worst[i]);
    }

    double reflected_value;
    OptimizationStatus status =
        nelder_mead_evaluate(run, store->reflected, &reflected_value);
    if (status != OPTIMIZATION_OK) {
        return status;
    }

    const double* replacement = NULL;
    double replacement_value = INFINITY;
    if (reflected_value < best_value) {
        // Expansion: c + 2 * (reflected - c)
        for (size_t i = 0; i < param_count; ++i) {
            store->trial[i] = store->centroid[i] +
                              2.0 * (store->reflected[i] - store->centroid[i]);
        }

        double expanded_value;
        status = nelder_mead_evaluate(run, store->trial, &expanded_value);
        if (status != OPTIMIZATION_OK) {
            return status;
        }

        if (expanded_value < reflected_value) {
            replacement = store->trial;
            replacement_value = expanded_value;
        } else {
            replacement = store->reflected;
            replacement_value = reflected_value;
        }
    } else if (reflected_value < second_worst_value) {
        replacement = store->reflected;
        replacement_value = reflected_value;
    } else {
        const int outside = reflected_value < worst_value;

        /*
         * Outside contraction: midpoint of c and reflected
         * Inside contraction: midpoint of c and worst
         */
        const double* endpoint = outside ? store->reflected : worst;

        for (size_t i = 0; i < param_count; ++i) {
            store->trial[i] = 0.5 * store->centroid[i] + 0.5 * endpoint[i];
        }

        double contracted_value;
        status = nelder_mead_evaluate(run, store->trial, &contracted_value);
        if (status != OPTIMIZATION_OK) {
            return status;
        }

        const int accept_contraction =
            isfinite(contracted_value) &&
            (outside ? contracted_value <= reflected_value
                     : contracted_value < worst_value);

        if (accept_contraction) {
            replacement = store->trial;
            replacement_value = contracted_value;
        }
    }

    if (replacement != NULL) {
        memcpy(worst, replacement, vector_bytes);
        store->values[ranking.worst] = replacement_value;
        return OPTIMIZATION_OK;
    }

    // Failed contraction: shrink every non-best vertex
    const double* best = store->vertices + ranking.best * param_count;

    for (size_t i = 0; i < store->vertex_count; ++i) {
        if (i == ranking.best) {
            continue;
        }

        double* vertex = store->vertices + i * param_count;
        for (size_t j = 0; j < param_count; ++j) {
            store->trial[j] = 0.5 * best[j] + 0.5 * vertex[j];
        }

        double value;
        status = nelder_mead_evaluate(run, store->trial, &value);
        if (status != OPTIMIZATION_OK) {
            return status;
        }

        memcpy(vertex, store->trial, vector_bytes);
        store->values[i] = value;
    }

    return OPTIMIZATION_OK;
}

static void nelder_mead_store_destroy(NelderMeadStore* store) {
    if (store == NULL) return;
    free(store->storage);
    *store = (NelderMeadStore){0};
}

/* Public Functions */

NelderMeadOptions nelder_mead_options_default(void) {
    return (NelderMeadOptions){
        .max_iterations = 2000,
        .max_evaluations = 10000,
        .param_tolerance = 1e-6,
        .objective_tolerance_absolute = 1e-12,
        .objective_tolerance_relative = 1e-10,
    };
}

OptimizationStatus nelder_mead_minimize(
    OptimizationObjective objective,
    void* user_data,
    size_t param_count,
    const double* initial_params,
    const double* initial_steps,
    const NelderMeadOptions* options,
    double* out_params,
    NelderMeadResult* out_result
) {
    const NelderMeadOptions opts =
        options != NULL ? *options : nelder_mead_options_default();

    OptimizationStatus status = nelder_mead_validate_args(
        objective, param_count, initial_params, initial_steps, &opts, out_params
    );
    if (status != OPTIMIZATION_OK) {
        return status;
    }

    NelderMeadStore store = {0};
    status = nelder_mead_store_init(param_count, &store);
    if (status != OPTIMIZATION_OK) {
        return status;
    }

    NelderMeadRun run = {
        .store = &store,
        .objective = objective,
        .user_data = user_data,
        .max_evaluations = opts.max_evaluations,
        .evaluations = 0,
        .best_value = INFINITY,
    };
    size_t iterations = 0;
    status =
        nelder_mead_initialize_simplex(&run, initial_params, initial_steps);

    while (status == OPTIMIZATION_OK) {
        const NelderMeadRanking ranking = nelder_mead_rank(&store);
        if (nelder_mead_converged(&store, &opts, ranking.best)) {
            break;
        }

        if (iterations >= opts.max_iterations) {
            status = OPTIMIZATION_ITERATION_LIMIT;
            break;
        }

        if (run.evaluations >= opts.max_evaluations) {
            status = OPTIMIZATION_EVALUATION_LIMIT;
            break;
        }

        status = nelder_mead_update(&run, ranking);
        if (status == OPTIMIZATION_OK) {
            ++iterations;
        }
    }

    if (status == OPTIMIZATION_OK || status == OPTIMIZATION_ITERATION_LIMIT ||
        status == OPTIMIZATION_EVALUATION_LIMIT) {
        memcpy(out_params, store.best, param_count * sizeof(double));
        if (out_result != NULL) {
            *out_result = (NelderMeadResult){
                .iterations = iterations,
                .evaluations = run.evaluations,
                .objective_value = run.best_value,
            };
        }
    }

    nelder_mead_store_destroy(&store);
    return status;
}
