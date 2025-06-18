cmake_minimum_required(VERSION 3.22)

project(occ3d VERSION 1.0)

set(CMAKE_CXX_VERSION 11)
set(CMAKE_CXX_VERSION_REQUIRED True)
set(CMAKE_BUILD_TYPE RELEASE)

# occ3d library
set(OCC3D_PATH_EIGEN3 ext/eigen3)
set(OCC3D_PATH_OPEN3D ext/Open3D)
add_subdirectory(${OCC3D_PATH_EIGEN3})
add_subdirectory(${OCC3D_PATH_OPEN3D})
set(OCC3D_LIB_PATH lib)
set(OCC3D_LIB_PATH_INCLUDE ${OCC3D_LIB_PATH}/include)
file(GLOB_RECURSE OCC3D_LIB_SOURCES lib/src/*.cpp)
add_library(occ3d STATIC ${OCC3D_LIB_SOURCES})
target_include_directories(occ3d PRIVATE SYSTEM ${OCC3D_PATH_EIGEN3} SYSTEM ${OCC3D_PATH_OPEN3D}/cpp)
target_link_libraries(occ3d PRIVATE Eigen3::Eigen Open3D::Open3D)

# voxelizer binary
file(GLOB_RECURSE VOXELIZER_SOURCES src/*.cpp)

add_executable(voxelizer ${VOXELIZER_SOURCES})
target_include_directories(voxelizer PRIVATE ${OCC3D_LIB_PATH_INCLUDE})
target_link_libraries(voxelizer PRIVATE occ3d)