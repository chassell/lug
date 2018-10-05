cmake_minimum_required(VERSION 2.6)
project(lug)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION_PATCH 0)

# to configure the options specify them in in the command line or change them in the cmake UI.
# Don't edit the makefile!
option(LUG_BUILD_STATIC "Build static library" OFF)
option(LUG_BUILD_SHARED "Build shared library" ON)
option(LUG_BUILD_TESTS "Build tests" ON)
# option(LUG_BUILD_SAMPLES "Build samples" ON)

## targets
if (LUG_BUILD_STATIC OR LUG_BUILD_SHARED)
    add_subdirectory("lug" ${LUG_BINARY_DIR})
    set(default-target "lug-static")
endif ()

if (LUG_BUILD_SHARED)
    set(default-target "lug-shared")
endif()

if (LUG_BUILD_TESTS)
    add_subdirectory("samples")
    add_subdirectory("tests")
endif ()
