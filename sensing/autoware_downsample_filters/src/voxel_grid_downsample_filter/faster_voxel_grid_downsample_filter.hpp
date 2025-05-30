// Copyright 2023 TIER IV, Inc.
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

#ifndef VOXEL_GRID_DOWNSAMPLE_FILTER__FASTER_VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_
#define VOXEL_GRID_DOWNSAMPLE_FILTER__FASTER_VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_

#include "transform_info.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <unordered_map>
#include <vector>

namespace autoware::downsample_filters
{

class FasterVoxelGridDownsampleFilter
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

public:
  FasterVoxelGridDownsampleFilter();
  void set_voxel_size(float voxel_size_x, float voxel_size_y, float voxel_size_z);
  void set_field_offsets(const PointCloud2ConstPtr & input, const rclcpp::Logger & logger);
  void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info,
    const rclcpp::Logger & logger);

private:
  struct Centroid
  {
    float x;
    float y;
    float z;
    float intensity;
    uint32_t point_count_;

    Centroid() : x(0), y(0), z(0), intensity(0), point_count_(1) {}
    Centroid(float _x, float _y, float _z, float _intensity)
    : x(_x), y(_y), z(_z), intensity(_intensity)
    {
      this->point_count_ = 1;
    }

    void add_point(float _x, float _y, float _z, float _intensity)
    {
      this->x += _x;
      this->y += _y;
      this->z += _z;
      this->intensity += _intensity;
      this->point_count_++;
    }

    Eigen::Vector4f calc_centroid() const
    {
      Eigen::Vector4f centroid(
        (this->x / this->point_count_), (this->y / this->point_count_),
        (this->z / this->point_count_), (this->intensity / this->point_count_));
      return centroid;
    }
  };

  Eigen::Vector3f inverse_voxel_size_;
  int x_offset_;
  int y_offset_;
  int z_offset_;
  int intensity_index_;
  int intensity_offset_;
  bool offset_initialized_;

  Eigen::Vector4f get_point_from_global_offset(
    const PointCloud2ConstPtr & input, size_t global_offset) const;

  bool get_min_max_voxel(
    const PointCloud2ConstPtr & input, Eigen::Vector3i & min_voxel, Eigen::Vector3i & max_voxel);

  std::unordered_map<uint32_t, Centroid> calc_centroids_each_voxel(
    const PointCloud2ConstPtr & input, const Eigen::Vector3i & max_voxel,
    const Eigen::Vector3i & min_voxel);

  void copy_centroids_to_output(
    const std::unordered_map<uint32_t, Centroid> & voxel_centroid_map, PointCloud2 & output,
    const TransformInfo & transform_info) const;
};

}  // namespace autoware::downsample_filters
#endif  // VOXEL_GRID_DOWNSAMPLE_FILTER__FASTER_VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_  // NOLINT
