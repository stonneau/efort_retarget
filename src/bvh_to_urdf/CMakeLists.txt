cmake_minimum_required(VERSION 2.6)

include_directories("${SRC_DIR}")
include_directories("${SRC_TOOLS_DIR}")
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

find_package(Eigen3 REQUIRED)
add_executable(
        bvh_to_urdf
        main.cpp
        create_obj_box.h
        create_obj_box.cpp
        generate_contacts.h
        generate_contacts.cpp
        ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
        ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
)

TARGET_LINK_LIBRARIES(bvh_to_urdf prmcollision)
TARGET_LINK_LIBRARIES(bvh_to_urdf prm)
TARGET_LINK_LIBRARIES(bvh_to_urdf prmpath)
TARGET_LINK_LIBRARIES(bvh_to_urdf PQP)
