cmake_minimum_required(VERSION 3.22)
project(occ3d VERSION 1.0)

set(CMAKE_CXX_VERSION 17)
set(CMAKE_CXX_VERSION_REQUIRED True)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# occ3d library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Open3D REQUIRED)
set(OCC3D_LIB_PATH lib)
set(OCC3D_LIB_PATH_INCLUDE ${OCC3D_LIB_PATH}/include)
file(GLOB_RECURSE OCC3D_LIB_SOURCES lib/src/*.cpp)
add_library(occ3d STATIC ${OCC3D_LIB_SOURCES})
target_include_directories(occ3d PUBLIC ${OCC3D_LIB_PATH_INCLUDE})
target_link_libraries(occ3d PUBLIC Eigen3::Eigen Open3D::Open3D)

# voxelizer binary
file(GLOB_RECURSE VOXELIZER_SOURCES src/*.cpp)
add_executable(voxelizer ${VOXELIZER_SOURCES})
target_include_directories(voxelizer PUBLIC include ${OCC3D_LIB_PATH_INCLUDE})
target_link_libraries(voxelizer PUBLIC occ3d)