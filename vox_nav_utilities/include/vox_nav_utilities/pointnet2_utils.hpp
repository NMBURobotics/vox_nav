// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#include <vector>
#include <torch/torch.h>
#include <torch/cuda.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

namespace pointnet2_utils
{

  at::Tensor square_distance(at::Tensor & source_tensor, at::Tensor & target_tensor);

  at::Tensor index_points(at::Tensor & points, at::Tensor & idx);

  at::Tensor farthest_point_sample(at::Tensor input_tensor, int num_samples);

  at::Tensor query_ball_point(double radius, int nsample, at::Tensor & xyz, at::Tensor & new_xyz);

  at::Tensor load_pcl_as_torch_tensor(
    const std::string cloud_filename, int N, torch::Device device);

  at::Tensor load_pcl_as_torch_tensor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int N, torch::Device device);

  void check_avail_device();

  std::pair<at::Tensor, at::Tensor> sample_and_group(
    int npoint, double radius, int nsample,
    at::Tensor xyz, at::Tensor points);

  void torch_tensor_to_pcl_cloud(
    const at::Tensor * input_tensor,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> point_color);


}  // namespace pointnet2_utils
