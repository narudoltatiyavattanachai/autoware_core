// Copyright 2023 Qualcomm Innovation Center, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VOXEL_GRID_DOWNSAMPLE_FILTER__GPU__ADRENO_VOXEL_GRID_HPP_
#define VOXEL_GRID_DOWNSAMPLE_FILTER__GPU__ADRENO_VOXEL_GRID_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::downsample_filters
{

#ifdef USE_ADRENO_GPU
// GPU-accelerated voxel grid filtering for QCS6490
bool gpuAcceleratedVoxelGridFilter(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output,
  float leaf_size_x, float leaf_size_y, float leaf_size_z);
#endif

}  // namespace autoware::downsample_filters

#endif  // VOXEL_GRID_DOWNSAMPLE_FILTER__GPU__ADRENO_VOXEL_GRID_HPP_