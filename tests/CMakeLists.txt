add_executable(arsenal_test_optimization
    test_optimization.c
)
target_link_libraries(arsenal_test_optimization
    PRIVATE
        arsenal::optimization
)
arsenal_enable_warnings(arsenal_test_optimization)
add_test(
    NAME optimization
    COMMAND arsenal_test_optimization
)

add_executable(arsenal_test_glmath
    test_glmath.c
)
target_link_libraries(arsenal_test_glmath
    PRIVATE
        arsenal::glmath
)
arsenal_enable_warnings(arsenal_test_glmath)
add_test(
    NAME glmath
    COMMAND arsenal_test_glmath
)

if (ARSENAL_BUILD_ALIGNMENT)
    add_executable(arsenal_test_alignment_nelder_mead
        test_alignment_nelder_mead.c
    )
    target_link_libraries(arsenal_test_alignment_nelder_mead
        PRIVATE
            arsenal::alignment
    )
    arsenal_enable_warnings(arsenal_test_alignment_nelder_mead)
    add_test(
        NAME alignment_nelder_mead
        COMMAND arsenal_test_alignment_nelder_mead
    )
endif()
