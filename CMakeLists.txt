cmake_minimum_required(VERSION 3.20)
project(arsenal VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ARSENAL_ENABLE_LAPACKE
    "Enable LAPACKE-backed glmath operations" ON)
option(ARSENAL_BUILD_ALIGNMENT
    "Build alignment (requires LAPACKE)" ${ARSENAL_ENABLE_LAPACKE})
option(ARSENAL_BUILD_WS_CLIENT
    "Build the libwebsockets client" ON)
option(ARSENAL_WARNINGS_AS_ERRORS
    "Treat compiler warnings as errors" OFF)

if(ARSENAL_BUILD_ALIGNMENT AND NOT ARSENAL_ENABLE_LAPACKE)
    message(FATAL_ERROR
        "Alignment currently requires LAPACKE. Enable "
        "ARSENAL_ENABLE_LAPACKE or disable ARSENAL_BUILD_ALIGNMENT."
    )
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(PkgConfig QUIET)

# External math requirements.
add_library(arsenal_math INTERFACE)

if(UNIX)
    target_link_libraries(arsenal_math INTERFACE m)
endif()

# LAPACKE discovery. Fortran LAPACK alone does not provide lapacke.h.
if(ARSENAL_ENABLE_LAPACKE)
    add_library(arsenal_lapacke INTERFACE)

    if(PkgConfig_FOUND)
        pkg_check_modules(LAPACKE QUIET IMPORTED_TARGET lapacke)
    endif()

    if(TARGET PkgConfig::LAPACKE)
        target_link_libraries(arsenal_lapacke INTERFACE
            PkgConfig::LAPACKE
        )
    else()
        find_path(ARSENAL_LAPACKE_INCLUDE_DIR
            NAMES lapacke.h
            REQUIRED
        )
        find_library(ARSENAL_LAPACKE_LIBRARY
            NAMES lapacke
            REQUIRED
        )
        find_package(LAPACK REQUIRED)

        target_include_directories(arsenal_lapacke INTERFACE
            "${ARSENAL_LAPACKE_INCLUDE_DIR}"
        )
        target_link_libraries(arsenal_lapacke INTERFACE
            "${ARSENAL_LAPACKE_LIBRARY}"
            LAPACK::LAPACK
        )
    endif()

    # Existing code uses #ifdef: do not define this as 0 when disabled.
    target_compile_definitions(arsenal_lapacke INTERFACE
        GLMATH_ENABLE_LAPACK=1
    )
endif()

if(ARSENAL_BUILD_WS_CLIENT)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBWEBSOCKETS REQUIRED IMPORTED_TARGET libwebsockets)
endif()

function(arsenal_enable_warnings target)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
        )

        if(ARSENAL_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

# Register components in dependency order.
#
# DEPS contains internal component names.
# LIBRARIES contains external targets/libraries.
#
# Each component produces:
#   arsenal_<name>_usage   : headers and transitive usage requirements
#   arsenal_<name>_objects : compiled implementation
#   arsenal_<name>         : archive containing its internal dependency closure
function(arsenal_add_component name)
    cmake_parse_arguments(COMPONENT
        ""
        "DIRECTORY;SOURCE"
        "DEPS;LIBRARIES"
        ${ARGN}
    )

    set(include_dir
        "${PROJECT_SOURCE_DIR}/libs/${COMPONENT_DIRECTORY}/include"
    )
    set(source_file
        "${PROJECT_SOURCE_DIR}/libs/${COMPONENT_DIRECTORY}/src/${COMPONENT_SOURCE}"
    )

    add_library(arsenal_${name}_usage INTERFACE)
    target_include_directories(arsenal_${name}_usage INTERFACE
        "${include_dir}"
    )
    target_compile_features(arsenal_${name}_usage INTERFACE c_std_11)

    if(COMPONENT_LIBRARIES)
        target_link_libraries(arsenal_${name}_usage INTERFACE
            ${COMPONENT_LIBRARIES}
        )
    endif()

    set(closure "${name}")

    foreach(dependency IN LISTS COMPONENT_DEPS)
        if(NOT TARGET arsenal_${dependency}_usage)
            message(FATAL_ERROR
                "${name}: register dependency '${dependency}' first."
            )
        endif()

        target_link_libraries(arsenal_${name}_usage INTERFACE
            arsenal_${dependency}_usage
        )

        get_target_property(dependency_closure
            arsenal_${dependency}_usage
            ARSENAL_COMPONENT_CLOSURE
        )
        list(APPEND closure ${dependency_closure})
    endforeach()

    list(REMOVE_DUPLICATES closure)

    set_property(TARGET arsenal_${name}_usage PROPERTY
        ARSENAL_COMPONENT_CLOSURE "${closure}"
    )

    add_library(arsenal_${name}_objects OBJECT "${source_file}")
    target_link_libraries(arsenal_${name}_objects PRIVATE
        arsenal_${name}_usage
    )
    arsenal_enable_warnings(arsenal_${name}_objects)

    set(archive_objects)
    foreach(component IN LISTS closure)
        list(APPEND archive_objects
            "$<TARGET_OBJECTS:arsenal_${component}_objects>"
        )
    endforeach()

    add_library(arsenal_${name} STATIC ${archive_objects})
    add_library(arsenal::${name} ALIAS arsenal_${name})

    target_link_libraries(arsenal_${name} PUBLIC
        arsenal_${name}_usage
    )

    set_property(GLOBAL APPEND PROPERTY
        ARSENAL_COMPONENT_NAMES "${name}"
    )
endfunction()

# Header-only allocator.
add_library(arsenal_alloc INTERFACE)
add_library(arsenal::alloc ALIAS arsenal_alloc)

target_include_directories(arsenal_alloc INTERFACE
    "${PROJECT_SOURCE_DIR}/libs/alloc/include"
)
target_compile_features(arsenal_alloc INTERFACE c_std_11)

# Compiled components.
set(glmath_libraries arsenal_math)
if(ARSENAL_ENABLE_LAPACKE)
    list(APPEND glmath_libraries arsenal_lapacke)
endif()

arsenal_add_component(glmath
    DIRECTORY glmath
    SOURCE glmath.c
    LIBRARIES ${glmath_libraries}
)

arsenal_add_component(point_cloud
    DIRECTORY point_cloud
    SOURCE point_cloud.c
    DEPS glmath
)

arsenal_add_component(logging
    DIRECTORY logging
    SOURCE logging.c
    LIBRARIES Threads::Threads
)

arsenal_add_component(natnet_client
    DIRECTORY natnet_client
    SOURCE natnet_client.c
    LIBRARIES Threads::Threads
)

if(ARSENAL_BUILD_ALIGNMENT)
    arsenal_add_component(alignment
        DIRECTORY alignment
        SOURCE alignment.c
        DEPS point_cloud glmath
    )
endif()

if(ARSENAL_BUILD_WS_CLIENT)
    # The repository directory is currently spelled "netowrk".
    arsenal_add_component(ws_client
        DIRECTORY netowrk
        SOURCE ws_client.c
        DEPS logging
        LIBRARIES PkgConfig::LIBWEBSOCKETS Threads::Threads
    )
endif()

# Cohesive archive: include each enabled component's objects exactly once.
get_property(components GLOBAL PROPERTY ARSENAL_COMPONENT_NAMES)

set(all_objects)
set(all_usage_targets)

foreach(component IN LISTS components)
    list(APPEND all_objects
        "$<TARGET_OBJECTS:arsenal_${component}_objects>"
    )
    list(APPEND all_usage_targets
        arsenal_${component}_usage
    )
endforeach()

add_library(arsenal STATIC ${all_objects})
add_library(arsenal::arsenal ALIAS arsenal)

target_link_libraries(arsenal PUBLIC
    ${all_usage_targets}
    arsenal_alloc
)

message(STATUS "Arsenal components: ${components}")
message(STATUS "Arsenal LAPACKE: ${ARSENAL_ENABLE_LAPACKE}")

# Collect public headers from every library, including header-only libraries.
file(GLOB arsenal_public_headers
    CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/libs/*/include/*.h"
)

set(arsenal_header_output_dir "${PROJECT_SOURCE_DIR}/include")

add_custom_target(arsenal_headers ALL
    COMMAND "${CMAKE_COMMAND}" -E make_directory
        "${arsenal_header_output_dir}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        ${arsenal_public_headers}
        "${arsenal_header_output_dir}"
    COMMENT "Gathering Arsenal public headers into ${arsenal_header_output_dir}"
    VERBATIM
)
