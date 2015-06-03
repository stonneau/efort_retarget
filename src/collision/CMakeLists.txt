cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${SRC_DIR})
include_directories("${SRC_TOOLS_DIR})
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

message("${PQP_INCLUDE_DIR}")
PROJECT(prmcollision)


if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    ParserObj.h
    ParserObj.cpp
    ObjectDictionary.h
    ObjectDictionary.cpp
    Object.h
    Object.cpp
    Sphere.h
    Sphere.cpp
    Collider.h
    Collider.cpp
        ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
        ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
)


LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

add_library(prmcollision ${SOURCES})

TARGET_LINK_LIBRARIES(prmcollision PQP)

SET_TARGET_PROPERTIES(prmcollision PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(prmcollision PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
