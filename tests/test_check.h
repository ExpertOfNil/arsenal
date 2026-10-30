#ifndef ARSENAL_TEST_CHECK_H
#define ARSENAL_TEST_CHECK_H

#include <stdio.h>
#include <stdlib.h>

static inline void test_check_failed(
    const char* expression, const char* file, int line
) {
    fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
    exit(EXIT_FAILURE);
}

#define TEST_CHECK(expression)                                  \
    do {                                                        \
        if (!(expression)) {                                    \
            test_check_failed(#expression, __FILE__, __LINE__); \
        }                                                       \
    } while (0)

#endif /* ARSENAL_TEST_CHECK_H */
