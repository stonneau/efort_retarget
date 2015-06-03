cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${DRAWSTUFF_INCLUDE_DIR}")
include_directories("${SRC_DIR}")
include_directories("${SRC_TOOLS_DIR}")
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

LINK_DIRECTORIES(${DRAWSTUFF_LIBRARY_DIR})
LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

add_executable(
        effectorROM
        main.cpp
)

TARGET_LINK_LIBRARIES(effectorROM prmcollision)
TARGET_LINK_LIBRARIES(effectorROM prm)
TARGET_LINK_LIBRARIES(effectorROM prmpath)
