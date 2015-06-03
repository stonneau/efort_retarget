cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/src/prmpath")
include_directories("${SRC_DIR})
include_directories("${SRC_TOOLS_DIR})
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

PROJECT(prmpath)

LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    CompleteScenario.h
    CompleteScenario.cpp
    PostureSelection.h
    PostureSelection.cpp
    Robot.h
    Robot.cpp
    JointConstraint.h
    JointConstraint.cpp
    Jacobian.h
    Jacobian.cpp
    ROM.h
    ROM.cpp
    EffectorROM.cpp
    sampling/Sample.h
    sampling/Sample.cpp
    animation/StateInterpolation.h animation/StateInterpolation.cpp
    Export/ITOMPExporter.h Export/ITOMPExporter.cpp
    Export/Exporter.h Export/Exporter.cpp
    Export/BVHExporter.h Export/BVHExporter.cpp
    Export/FileHandler.h Export/FileHandler.cpp
    ik/IKSolver.h ik/IKSolver.cpp
    smoothing/smooth.h smoothing/smooth.cpp
    ik/PartialDerivativeConstraint.h ik/PartialDerivativeConstraint.cpp
    ik/MatchTargetConstraint.h ik/MatchTargetConstraint.cpp
    ik/VectorAlignmentConstraint.h ik/VectorAlignmentConstraint.cpp
    ik/ForceManipulabilityConstraint.h ik/ForceManipulabilityConstraint.cpp
    ik/ObstacleAvoidanceConstraint.h ik/ObstacleAvoidanceConstraint.cpp
        ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
        ${SRC_TOOLS_DIR}/ExpMap.h ${SRC_TOOLS_DIR}/ExpMap.cpp
        ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
)

add_library(prmpath ${SOURCES})
TARGET_LINK_LIBRARIES(prmpath prmcollision)
TARGET_LINK_LIBRARIES(prmpath prm)
TARGET_LINK_LIBRARIES(prmpath PQP)

SET_TARGET_PROPERTIES(prmpath PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(prmpath PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
