cmake_minimum_required(VERSION 2.6)


find_package(DrawStuff REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${DRAWSTUFF_INCLUDE_DIR}")
include_directories("${SRC_DIR}")
include_directories("${SRC_TOOLS_DIR}")
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

LINK_DIRECTORIES(${DRAWSTUFF_LIBRARY_DIR})
LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

add_executable(
        drawapp
        Main.cpp
		${SRC_TOOLS_DIR}/Timer.h ${SRC_TOOLS_DIR}/Timer.cpp
)

TARGET_LINK_LIBRARIES(drawapp drawstuff)
TARGET_LINK_LIBRARIES(drawapp prmcollision)
TARGET_LINK_LIBRARIES(drawapp prm)
TARGET_LINK_LIBRARIES(drawapp prmpath)
TARGET_LINK_LIBRARIES(drawapp PQP)
TARGET_LINK_LIBRARIES(drawapp retarget)
TARGET_LINK_LIBRARIES(drawapp ${OPENGL_LIBRARIES})
