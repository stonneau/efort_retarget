cmake_minimum_required(VERSION 2.6)
project(astar)


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set(SRC_TOOLS_DIR "${PROJECT_SOURCE_DIR}/src/tools")

set(EXTERNAL_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/external/lib")
set(EXTERNAL_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/external/lib")

if ( UNIX )  
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif ()

if ( MSVC )  
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS")
endif ()

find_package(OpenGL REQUIRED)
find_package(PQP REQUIRED)
find_package(Eigen3 REQUIRED)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#add_subdirectory (src/bvh_to_urdf)
add_subdirectory (src/prm)
add_subdirectory (src/prmpath)
add_subdirectory (src/retarget)
add_subdirectory (src/collision)
#add_subdirectory (src/drawapp)

