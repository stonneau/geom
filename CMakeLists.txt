cmake_minimum_required(VERSION 2.6)

if ( UNIX )  
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif ()

SET(PROJECT_NAME geom)
SET(PROJECT_DESCRIPTION
   "a few 2d algorithms for convex hulls"
 )
 SET(PROJECT_URL "")


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

set(EXTERNAL_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/external/lib")
set(EXTERNAL_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/external/lib")

if ( MSVC )  
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS")
endif()

#~ SETUP_PROJECT()

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
    include/geom/config.h
    include/geom/algorithms2D.h
  )

find_package(Eigen3 REQUIRED)

add_subdirectory (test)

#~ SETUP_PROJECT_FINALIZE()
#~ SETUP_PROJECT_CPACK()
