cmake_minimum_required(VERSION 2.6)

include_directories("${SRC_DIR}")
include_directories("${INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

PROJECT(geom)


if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    main.cpp
)

add_executable(geomtest ${SOURCES})

#~ TARGET_LINK_LIBRARIES(geomtest geom)
#~ TARGET_LINK_LIBRARIES(polytopetest polytope ${SRC_DIR}/../external/cddlib-094b/lib-src/libcdd.a)
