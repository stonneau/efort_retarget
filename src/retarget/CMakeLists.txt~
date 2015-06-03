cmake_minimum_required(VERSION 2.6)

set(SRC_TOOLS_DIR "${PROJECT_SOURCE_DIR}/src/tools")

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${SRC_DIR})
include_directories("${SRC_TOOLS_DIR})
include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("${PQP_INCLUDE_DIR}")

PROJECT(retarget)


if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    Motion.h Motion.cpp
    MotionInternal.h MotionInternal.cpp
    ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
    ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
)


LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

add_library(retarget ${SOURCES})

TARGET_LINK_LIBRARIES(retarget prmcollision)
TARGET_LINK_LIBRARIES(retarget prm)
TARGET_LINK_LIBRARIES(retarget prmpath)
TARGET_LINK_LIBRARIES(retarget PQP)

SET_TARGET_PROPERTIES(retarget PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(retarget PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
