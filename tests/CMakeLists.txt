cmake_minimum_required(VERSION 2.6)

set(TEST_SOURCES 
    leftrecursion.cpp
    nonterminals.cpp
    parser.cpp
    predicates.cpp
    terminals.cpp
    )

include_directories("${lug_SOURCE_DIR}")

add_compile_options(-std=c++17)

if (LUG_BUILD_TESTS)
    foreach (TSRC ${TEST_SOURCES})
        STRING(REGEX REPLACE ".cpp$" "" TBIN ${TSRC})
        get_filename_component(TBIN ${TBIN} NAME)
        add_executable(${TBIN} ${TSRC})
        target_link_libraries(${TBIN} ${default-target})
    endforeach ()
endif ()

# source_group("Source/Engine" FILES ${SOURCES_ENGINE})
# source_group("Include/Common" FILES ${HEADERS_COMMON})
