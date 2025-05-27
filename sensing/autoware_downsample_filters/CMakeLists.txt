cmake_minimum_required(VERSION 3.14)
project(autoware_downsample_filters)

find_package(autoware_cmake REQUIRED)

autoware_package()

# Check for ARM64 architecture (QCS6490)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Building for ARM64 architecture (QCS6490)")
  
  # ARM64-specific optimization flags
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+crypto+crc -mtune=cortex-a76")
  
  # Look for OpenCL for Adreno GPU acceleration
  find_package(OpenCL QUIET)
  if(OpenCL_FOUND)
    message(STATUS "OpenCL found, enabling Adreno GPU acceleration for voxel grid filter")
    add_definitions(-DUSE_ADRENO_GPU)
    set(GPU_ACCEL_SOURCES
      src/voxel_grid_downsample_filter/gpu/adreno_voxel_grid.cpp
    )
    set(GPU_ACCEL_LIBS ${OpenCL_LIBRARIES})
    include_directories(${OpenCL_INCLUDE_DIRS})
  endif()
endif()

set(RANDOM random_downsample_filter)
set(VOXEL_GRID voxel_grid_downsample_filter)


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/random_downsample_filter/random_downsample_filter_node.cpp
  src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.cpp
  src/voxel_grid_downsample_filter/faster_voxel_grid_downsample_filter.cpp
  src/voxel_grid_downsample_filter/memory.cpp
  ${GPU_ACCEL_SOURCES}
)

# Link GPU acceleration libraries if available
if(DEFINED GPU_ACCEL_LIBS)
  target_link_libraries(${PROJECT_NAME} ${GPU_ACCEL_LIBS})
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::downsample_filters::RandomDownsampleFilter"
  EXECUTABLE ${RANDOM}_node)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::downsample_filters::VoxelGridDownsampleFilter"
  EXECUTABLE ${VOXEL_GRID}_node)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
