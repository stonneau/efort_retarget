cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${SRC_DIR})
include_directories("${SRC_TOOLS_DIR})
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

PROJECT(prm)

LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    Model.h
    SimplePRM.h
    SimplePRM.cpp
    LocalPlanner.h
    LocalPlanner.cpp
    Generator.h
    Generator.cpp
    Scenario.h
    Scenario.cpp
        ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
        ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
		${SRC_TOOLS_DIR}/Timer.h ${SRC_TOOLS_DIR}/Timer.cpp
)

add_library(prm ${SOURCES})
TARGET_LINK_LIBRARIES(prm prmcollision)
TARGET_LINK_LIBRARIES(prm PQP)

SET_TARGET_PROPERTIES(prm PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(prm PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
