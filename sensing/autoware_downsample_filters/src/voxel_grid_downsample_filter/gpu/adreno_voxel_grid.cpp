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

#ifdef USE_ADRENO_GPU

#include <CL/cl.h>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include <iostream>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "../faster_voxel_grid_downsample_filter.hpp"

namespace autoware::downsample_filters
{

// OpenCL kernel for voxel grid downsampling
const char * voxel_grid_kernel_source = R"(
__kernel void voxel_grid_kernel(
    __global float4 * points,
    __global float * intensities,
    __global int * voxel_indices,
    __global float4 * centroids,
    __global float * centroid_intensities,
    __global int * point_count,
    const int num_points,
    const float inverse_voxel_size_x,
    const float inverse_voxel_size_y,
    const float inverse_voxel_size_z)
{
    int idx = get_global_id(0);
    if (idx >= num_points) return;
    
    float4 point = points[idx];
    float intensity = intensities[idx];
    
    // Calculate voxel index
    int voxel_x = (int)(point.x * inverse_voxel_size_x);
    int voxel_y = (int)(point.y * inverse_voxel_size_y);
    int voxel_z = (int)(point.z * inverse_voxel_size_z);
    
    // Create a unique voxel index
    int voxel_idx = ((voxel_x * 73856093) ^ (voxel_y * 19349663) ^ (voxel_z * 83492791)) % 1000000;
    voxel_indices[idx] = voxel_idx;
    
    // Atomic operations to update centroids and counts
    atomic_add(&centroids[voxel_idx].x, point.x);
    atomic_add(&centroids[voxel_idx].y, point.y);
    atomic_add(&centroids[voxel_idx].z, point.z);
    atomic_add(&centroid_intensities[voxel_idx], intensity);
    atomic_add(&point_count[voxel_idx], 1);
}
)";

// Helper class for Adreno GPU acceleration
class AdrenoGpuAccelerator
{
public:
  AdrenoGpuAccelerator()
  : initialized_(false)
  {
    initializeOpenCL();
  }

  ~AdrenoGpuAccelerator()
  {
    if (initialized_) {
      clReleaseKernel(kernel_);
      clReleaseProgram(program_);
      clReleaseCommandQueue(command_queue_);
      clReleaseContext(context_);
    }
  }

  bool isInitialized() const { return initialized_; }

  // Process pointcloud using Adreno GPU
  bool processPointCloud(const sensor_msgs::msg::PointCloud2 & input,
                         sensor_msgs::msg::PointCloud2 & output,
                         float leaf_size_x, float leaf_size_y, float leaf_size_z)
  {
    if (!initialized_) {
      return false;
    }

    try {
      // Extract points from PointCloud2
      size_t point_count = input.width * input.height;
      std::vector<float> points_data(point_count * 4);  // x,y,z,padding
      std::vector<float> intensities(point_count);

      // Extract data from PointCloud2
      extractPointsAndIntensities(input, points_data, intensities);

      // Create OpenCL buffers
      cl_int err;
      cl_mem points_buffer = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
        sizeof(float) * points_data.size(), points_data.data(), &err);
      cl_mem intensities_buffer = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
        sizeof(float) * intensities.size(), intensities.data(), &err);
      
      // Allocate buffers for results
      std::vector<int> voxel_indices(point_count, 0);
      cl_mem voxel_indices_buffer = clCreateBuffer(context_, CL_MEM_WRITE_ONLY,
        sizeof(int) * voxel_indices.size(), nullptr, &err);
      
      // We don't know how many unique voxels in advance, allocate a large buffer
      size_t max_voxels = point_count / 10 + 1;  // Estimate
      std::vector<float> centroids_data(max_voxels * 4, 0.0f);
      std::vector<float> centroid_intensities(max_voxels, 0.0f);
      std::vector<int> point_count_per_voxel(max_voxels, 0);
      
      cl_mem centroids_buffer = clCreateBuffer(context_, CL_MEM_READ_WRITE,
        sizeof(float) * centroids_data.size(), nullptr, &err);
      cl_mem centroid_intensities_buffer = clCreateBuffer(context_, CL_MEM_READ_WRITE,
        sizeof(float) * centroid_intensities.size(), nullptr, &err);
      cl_mem point_count_buffer = clCreateBuffer(context_, CL_MEM_READ_WRITE,
        sizeof(int) * point_count_per_voxel.size(), nullptr, &err);
      
      // Set kernel arguments
      float inv_leaf_x = 1.0f / leaf_size_x;
      float inv_leaf_y = 1.0f / leaf_size_y;
      float inv_leaf_z = 1.0f / leaf_size_z;
      
      clSetKernelArg(kernel_, 0, sizeof(cl_mem), &points_buffer);
      clSetKernelArg(kernel_, 1, sizeof(cl_mem), &intensities_buffer);
      clSetKernelArg(kernel_, 2, sizeof(cl_mem), &voxel_indices_buffer);
      clSetKernelArg(kernel_, 3, sizeof(cl_mem), &centroids_buffer);
      clSetKernelArg(kernel_, 4, sizeof(cl_mem), &centroid_intensities_buffer);
      clSetKernelArg(kernel_, 5, sizeof(cl_mem), &point_count_buffer);
      clSetKernelArg(kernel_, 6, sizeof(int), &point_count);
      clSetKernelArg(kernel_, 7, sizeof(float), &inv_leaf_x);
      clSetKernelArg(kernel_, 8, sizeof(float), &inv_leaf_y);
      clSetKernelArg(kernel_, 9, sizeof(float), &inv_leaf_z);
      
      // Execute kernel
      size_t global_work_size = point_count;
      err = clEnqueueNDRangeKernel(command_queue_, kernel_, 1, nullptr,
        &global_work_size, nullptr, 0, nullptr, nullptr);
      
      if (err != CL_SUCCESS) {
        std::cerr << "Failed to execute kernel: " << err << std::endl;
        return false;
      }
      
      // Read results back
      clEnqueueReadBuffer(command_queue_, voxel_indices_buffer, CL_TRUE, 0,
        sizeof(int) * voxel_indices.size(), voxel_indices.data(), 0, nullptr, nullptr);
      clEnqueueReadBuffer(command_queue_, centroids_buffer, CL_TRUE, 0,
        sizeof(float) * centroids_data.size(), centroids_data.data(), 0, nullptr, nullptr);
      clEnqueueReadBuffer(command_queue_, centroid_intensities_buffer, CL_TRUE, 0,
        sizeof(float) * centroid_intensities.size(), centroid_intensities.data(), 0, nullptr, nullptr);
      clEnqueueReadBuffer(command_queue_, point_count_buffer, CL_TRUE, 0,
        sizeof(int) * point_count_per_voxel.size(), point_count_per_voxel.data(), 0, nullptr, nullptr);
      
      // Process results and generate output cloud
      generateOutputCloud(point_count_per_voxel, centroids_data, centroid_intensities, output, input);
      
      // Clean up
      clReleaseMemObject(points_buffer);
      clReleaseMemObject(intensities_buffer);
      clReleaseMemObject(voxel_indices_buffer);
      clReleaseMemObject(centroids_buffer);
      clReleaseMemObject(centroid_intensities_buffer);
      clReleaseMemObject(point_count_buffer);
      
      return true;
    } catch (const std::exception & e) {
      std::cerr << "Error in GPU processing: " << e.what() << std::endl;
      return false;
    }
  }

private:
  cl_context context_;
  cl_command_queue command_queue_;
  cl_program program_;
  cl_kernel kernel_;
  bool initialized_;

  void initializeOpenCL()
  {
    cl_int err;
    
    // Get platform
    cl_platform_id platform;
    cl_uint num_platforms;
    err = clGetPlatformIDs(1, &platform, &num_platforms);
    if (err != CL_SUCCESS || num_platforms == 0) {
      std::cerr << "Failed to find OpenCL platforms." << std::endl;
      return;
    }
    
    // Get device (prefer GPU)
    cl_device_id device;
    cl_uint num_devices;
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, &num_devices);
    if (err != CL_SUCCESS || num_devices == 0) {
      std::cerr << "Failed to find OpenCL GPU device." << std::endl;
      return;
    }
    
    // Check if it's an Adreno GPU
    char vendor[256];
    err = clGetDeviceInfo(device, CL_DEVICE_VENDOR, sizeof(vendor), vendor, nullptr);
    if (err != CL_SUCCESS || strstr(vendor, "Qualcomm") == nullptr) {
      std::cerr << "Device is not a Qualcomm Adreno GPU." << std::endl;
      // Continue anyway, might still work
    }
    
    // Create context
    context_ = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &err);
    if (err != CL_SUCCESS) {
      std::cerr << "Failed to create OpenCL context." << std::endl;
      return;
    }
    
    // Create command queue
    command_queue_ = clCreateCommandQueue(context_, device, 0, &err);
    if (err != CL_SUCCESS) {
      std::cerr << "Failed to create command queue." << std::endl;
      clReleaseContext(context_);
      return;
    }
    
    // Create program
    const char * source = voxel_grid_kernel_source;
    size_t source_size = strlen(source);
    program_ = clCreateProgramWithSource(context_, 1, &source, &source_size, &err);
    if (err != CL_SUCCESS) {
      std::cerr << "Failed to create program." << std::endl;
      clReleaseCommandQueue(command_queue_);
      clReleaseContext(context_);
      return;
    }
    
    // Build program
    err = clBuildProgram(program_, 1, &device, nullptr, nullptr, nullptr);
    if (err != CL_SUCCESS) {
      char build_log[16384];
      clGetProgramBuildInfo(program_, device, CL_PROGRAM_BUILD_LOG, sizeof(build_log), build_log, nullptr);
      std::cerr << "Error building program: " << build_log << std::endl;
      clReleaseProgram(program_);
      clReleaseCommandQueue(command_queue_);
      clReleaseContext(context_);
      return;
    }
    
    // Create kernel
    kernel_ = clCreateKernel(program_, "voxel_grid_kernel", &err);
    if (err != CL_SUCCESS) {
      std::cerr << "Failed to create kernel." << std::endl;
      clReleaseProgram(program_);
      clReleaseCommandQueue(command_queue_);
      clReleaseContext(context_);
      return;
    }
    
    initialized_ = true;
  }

  void extractPointsAndIntensities(
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::vector<float> & points,
    std::vector<float> & intensities)
  {
    // Get field offsets
    int x_idx = -1, y_idx = -1, z_idx = -1, intensity_idx = -1;
    for (size_t d = 0; d < cloud.fields.size(); ++d) {
      if (cloud.fields[d].name == "x") x_idx = static_cast<int>(d);
      if (cloud.fields[d].name == "y") y_idx = static_cast<int>(d);
      if (cloud.fields[d].name == "z") z_idx = static_cast<int>(d);
      if (cloud.fields[d].name == "intensity") intensity_idx = static_cast<int>(d);
    }
    
    if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
      throw std::runtime_error("Point cloud missing x, y, or z fields");
    }
    
    // Extract point data
    const uint8_t * cloud_data = cloud.data.data();
    size_t point_count = cloud.width * cloud.height;
    
    for (size_t i = 0; i < point_count; ++i) {
      size_t point_offset = i * cloud.point_step;
      
      // Extract x, y, z
      float x = *reinterpret_cast<const float *>(&cloud_data[point_offset + cloud.fields[x_idx].offset]);
      float y = *reinterpret_cast<const float *>(&cloud_data[point_offset + cloud.fields[y_idx].offset]);
      float z = *reinterpret_cast<const float *>(&cloud_data[point_offset + cloud.fields[z_idx].offset]);
      
      // Store in points vector (x,y,z,padding)
      points[i * 4] = x;
      points[i * 4 + 1] = y;
      points[i * 4 + 2] = z;
      points[i * 4 + 3] = 0.0f;  // padding for alignment
      
      // Extract intensity if available
      if (intensity_idx != -1) {
        intensities[i] = *reinterpret_cast<const float *>(
          &cloud_data[point_offset + cloud.fields[intensity_idx].offset]);
      } else {
        intensities[i] = 0.0f;
      }
    }
  }

  void generateOutputCloud(
    const std::vector<int> & point_counts,
    const std::vector<float> & centroids,
    const std::vector<float> & intensities,
    sensor_msgs::msg::PointCloud2 & output,
    const sensor_msgs::msg::PointCloud2 & input_for_metadata)
  {
    // Count non-zero entries (valid voxels)
    size_t valid_voxels = 0;
    for (size_t i = 0; i < point_counts.size(); ++i) {
      if (point_counts[i] > 0) {
        valid_voxels++;
      }
    }
    
    // Set up output cloud header
    output.header = input_for_metadata.header;
    output.height = 1;
    output.width = static_cast<uint32_t>(valid_voxels);
    output.is_dense = true;
    output.is_bigendian = input_for_metadata.is_bigendian;
    output.fields = input_for_metadata.fields;
    output.point_step = input_for_metadata.point_step;
    output.row_step = output.point_step * output.width;
    output.data.resize(output.row_step);
    
    // Copy centroid data to output
    size_t output_idx = 0;
    for (size_t i = 0; i < point_counts.size(); ++i) {
      if (point_counts[i] > 0) {
        // Calculate centroid by dividing by point count
        float centroid_x = centroids[i * 4] / point_counts[i];
        float centroid_y = centroids[i * 4 + 1] / point_counts[i];
        float centroid_z = centroids[i * 4 + 2] / point_counts[i];
        float centroid_intensity = intensities[i] / point_counts[i];
        
        // Find field offsets in output
        int x_idx = -1, y_idx = -1, z_idx = -1, intensity_idx = -1;
        for (size_t d = 0; d < output.fields.size(); ++d) {
          if (output.fields[d].name == "x") x_idx = static_cast<int>(d);
          if (output.fields[d].name == "y") y_idx = static_cast<int>(d);
          if (output.fields[d].name == "z") z_idx = static_cast<int>(d);
          if (output.fields[d].name == "intensity") intensity_idx = static_cast<int>(d);
        }
        
        // Get the beginning of this point in the data array
        uint8_t * output_data = output.data.data() + output_idx * output.point_step;
        
        // Copy centroid data
        if (x_idx >= 0) {
          *reinterpret_cast<float *>(&output_data[output.fields[x_idx].offset]) = centroid_x;
        }
        if (y_idx >= 0) {
          *reinterpret_cast<float *>(&output_data[output.fields[y_idx].offset]) = centroid_y;
        }
        if (z_idx >= 0) {
          *reinterpret_cast<float *>(&output_data[output.fields[z_idx].offset]) = centroid_z;
        }
        if (intensity_idx >= 0) {
          *reinterpret_cast<float *>(&output_data[output.fields[intensity_idx].offset]) = centroid_intensity;
        }
        
        output_idx++;
      }
    }
  }
};

// GPU-accelerated voxel grid filtering for QCS6490
bool gpuAcceleratedVoxelGridFilter(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output,
  float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
  static std::unique_ptr<AdrenoGpuAccelerator> accelerator = std::make_unique<AdrenoGpuAccelerator>();
  
  if (accelerator->isInitialized()) {
    return accelerator->processPointCloud(input, output, leaf_size_x, leaf_size_y, leaf_size_z);
  } else {
    // Fall back to CPU implementation if GPU acceleration failed to initialize
    return false;
  }
}

}  // namespace autoware::downsample_filters

#endif  // USE_ADRENO_GPU