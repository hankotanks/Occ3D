cmake_minimum_required(VERSION 3.22)
project(occ3d VERSION 1.0)

# basic options
set(CMAKE_CXX_VERSION 17)
set(CMAKE_CXX_VERSION_REQUIRED True)
set(CMAKE_BUILD_TYPE RELEASE)
# required for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# build the occ3d library
set(OCC3D_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(${OCC3D_LIB})

# voxelizer
file(GLOB_RECURSE VOXELIZER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(voxelizer ${VOXELIZER_SRC})
target_include_directories(voxelizer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(voxelizer PRIVATE occ3d)