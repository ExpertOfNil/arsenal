#include "glmath.h"
#include "test_check.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>

typedef struct RotationVectorCase {
    const char* name;
    vec3d omega;
    mat3d expected;
} RotationVectorCase;

static void test_rotation_vectors(void) {
    const double half_pi = 0.5 * acos(-1.0);

    const RotationVectorCase cases[] = {
        {
            .name = "zero",
            .omega = {0.0, 0.0, 0.0},
            .expected = {
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            },
        },
        {
            .name = "positive X",
            .omega = {half_pi, 0.0, 0.0},
            .expected = {
                1.0, 0.0, 0.0,
                0.0, 0.0, 1.0,
                0.0, -1.0, 0.0,
            },
        },
        {
            .name = "negative X",
            .omega = {-half_pi, 0.0, 0.0},
            .expected = {
                1.0, 0.0, 0.0,
                0.0, 0.0, -1.0,
                0.0, 1.0, 0.0,
            },
        },
        {
            .name = "positive Y",
            .omega = {0.0, half_pi, 0.0},
            .expected = {
                0.0, 0.0, -1.0,
                0.0, 1.0, 0.0,
                1.0, 0.0, 0.0,
            },
        },
        {
            .name = "negative Y",
            .omega = {0.0, -half_pi, 0.0},
            .expected = {
                0.0, 0.0, 1.0,
                0.0, 1.0, 0.0,
                -1.0, 0.0, 0.0,
            },
        },
        {
            .name = "positive Z",
            .omega = {0.0, 0.0, half_pi},
            .expected = {
                0.0, 1.0, 0.0,
                -1.0, 0.0, 0.0,
                0.0, 0.0, 1.0,
            },
        },
        {
            .name = "negative Z",
            .omega = {0.0, 0.0, -half_pi},
            .expected = {
                0.0, -1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, 1.0,
            },
        },
        {
            /*
             * A positive 120-degree rotation about (1,1,1)
             * cycles X -> Y, Y -> Z, Z -> X.
             */
            .name = "oblique axis",
            .omega = {
                2.0 * acos(-1.0) / (3.0 * sqrt(3.0)),
                2.0 * acos(-1.0) / (3.0 * sqrt(3.0)),
                2.0 * acos(-1.0) / (3.0 * sqrt(3.0)),
            },
            .expected = {
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
                1.0, 0.0, 0.0,
            },
        },
    };

    const size_t case_count = sizeof(cases) / sizeof(cases[0]);

    for (size_t i = 0; i < case_count; ++i) {
        const RotationVectorCase* test = &cases[i];

        mat3d rotation;
        TEST_CHECK(
            mat3d_from_rotation_vector(test->omega, rotation) == 0
        );

        for (size_t j = 0; j < 9; ++j) {
            const double error = fabs(
                rotation[j] - test->expected[j]
            );

            if (!(error < 1e-14)) {
                fprintf(
                    stderr,
                    "%s: matrix[%zu]: got %.17g, expected %.17g\n",
                    test->name,
                    j,
                    rotation[j],
                    test->expected[j]
                );
            }

            TEST_CHECK(error < 1e-14);
        }
    }
}

int main(void) {
    test_rotation_vectors();

    puts("glmath tests passed");
    return 0;
}
