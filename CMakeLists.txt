cmake_minimum_required(VERSION 3.21)
project(scope LANGUAGES CXX)

include(FetchContent)

# With no build type CMake passes neither -O nor -g, so default to Debug.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(DISABLE_LOGGING "Disable logging" OFF)
set(LOGGING_LEVEL INFO CACHE STRING "Logging level")
option(SANITIZE "Build scope-test with AddressSanitizer and UndefinedBehaviorSanitizer" ON)
option(COVERAGE "Build scope-test with gcov instrumentation, for gcovr" OFF)

########## Dependencies ##########

# FOUND brings stb_image and Eigen with it. Pinned to a commit so a change on
# FOUND's main cannot alter a build. To move to a newer FOUND, change this hash.
FetchContent_Declare(
    found
    GIT_REPOSITORY https://github.com/UWCubeSat/found.git
    GIT_TAG 747d29b71aa0276080398c9ef912cd562b260bcc
    EXCLUDE_FROM_ALL
)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(found googletest)

########## SCOPE ##########

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

file(GLOB_RECURSE SCOPE_SOURCES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/src/scope/*.cpp)
list(REMOVE_ITEM SCOPE_SOURCES
    ${PROJECT_SOURCE_DIR}/src/scope/main.cpp
    ${PROJECT_SOURCE_DIR}/src/scope/synth-main.cpp)
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/test/*.cpp)

# Warnings and definitions for SCOPE's own targets, not the fetched libraries.
add_library(scope_options INTERFACE)
target_compile_options(scope_options INTERFACE
    -Wall -Wextra -Wno-missing-field-initializers -pedantic -Wdouble-promotion -Werror)
if(NOT DISABLE_LOGGING)
    target_compile_definitions(scope_options INTERFACE ENABLE_LOGGING LOGGING_LEVEL=${LOGGING_LEVEL})
endif()

add_library(scope_lib STATIC ${SCOPE_SOURCES})
target_include_directories(scope_lib PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(scope_lib PUBLIC found::found_lib stb_image PRIVATE scope_options)

add_executable(scope ${PROJECT_SOURCE_DIR}/src/scope/main.cpp)
target_link_libraries(scope PRIVATE scope_lib scope_options)

# The synthetic data generator, for testing scope end to end.
add_executable(scope-synth ${PROJECT_SOURCE_DIR}/src/scope/synth-main.cpp)
target_link_libraries(scope-synth PRIVATE scope_lib scope_options)

########## Tests ##########

# The tests compile SCOPE's sources again instead of linking scope_lib, so the
# sanitizers and coverage instrument them without slowing down scope itself.
add_executable(scope-test ${TEST_SOURCES} ${SCOPE_SOURCES})
target_include_directories(scope-test PRIVATE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(scope-test PRIVATE found::found_lib stb_image gmock scope_options)
# An Eigen matrix read before it is written holds NaN, which the tests notice.
target_compile_definitions(scope-test PRIVATE EIGEN_INITIALIZE_MATRICES_BY_NAN)

if(SANITIZE)
    set(SANITIZER_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer)
    target_compile_options(scope-test PRIVATE ${SANITIZER_FLAGS})
    target_link_options(scope-test PRIVATE ${SANITIZER_FLAGS})
endif()

if(COVERAGE)
    target_compile_options(scope-test PRIVATE --coverage)
    target_link_options(scope-test PRIVATE --coverage)
endif()
