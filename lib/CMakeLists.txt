set(OCC3D_EXT ${CMAKE_CURRENT_SOURCE_DIR}/ext)

# build eigen3
add_subdirectory(${OCC3D_EXT}/eigen3)

# build Open3D
# reduce size of Open3D by disabling examples/tests
set(GLIBCXX_USE_CXX11_ABI ON CACHE BOOL "Fix for Open3D build process.")
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable Open3D examples.")
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "Don't build Open3D python module.")
# set(BUILD_FILAMENT_FROM_SOURCE ON CACHE BOOL "Build Filament from source for Open3D.")
set(DEVELOPER_BUILD OFF CACHE BOOL "Build Open3D release.")
add_subdirectory(${OCC3D_EXT}/Open3D)

# create library
file(GLOB_RECURSE OCC3D_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(occ3d STATIC ${OCC3D_SRC})

# since eigen3 is header-only
# we need to specify the include folder
get_target_property(OCC3D_INC_EIGEN Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
set(OCC3D_INC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(occ3d PUBLIC ${OCC3D_INC} ${OCC3D_INC_EIGEN})

# link with eigen3 and Open3D
target_link_libraries(occ3d PRIVATE Eigen3::Eigen Open3D::Open3D)
