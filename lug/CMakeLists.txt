cmake_minimum_required(VERSION 2.6)

## sources
set(SOURCES_LIB
    lug.cpp
    unicode.cpp
    utf8.cpp
    )

set(HEADERS_LIB
    detail.hpp
    error.hpp
    lug.hpp
    unicode.hpp
    utf8.hpp
    )

include_directories("${lug_SOURCE_DIR}")

## properties
# version info
set_property(GLOBAL PROPERTY VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

add_compile_options(-std=c++17)

## targets
if (LUG_BUILD_STATIC)
    add_library(lug-static STATIC ${SOURCES_LIB} ${HEADERS_LIB})
    set_property(TARGET lug-static PROPERTY OUTPUT_NAME lug)
    set(default-target lug-static)
endif ()

if (LUG_BUILD_SHARED)
    add_library(lug-shared SHARED ${SOURCES_LIB} ${HEADERS_LIB})
    set_property(TARGET lug-shared PROPERTY OUTPUT_NAME lug)

    set_target_properties(lug-shared PROPERTIES
        VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
        SOVERSION ${VERSION_MAJOR})

    set(default-target lug-shared)
endif ()

## installation
install(FILES ${HEADERS_LIB} DESTINATION include/lug)
# configure_file(lug.pc.in lug.pc @ONLY)

if (LUG_BUILD_STATIC)
    install(TARGETS lug-static
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
endif ()

if (LUG_BUILD_SHARED)
    install(TARGETS lug-shared
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
endif ()
