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

  at::Tensor square_distance(at::Tensor source_tensor, at::Tensor target_tensor)
  {
    c10::IntArrayRef source_tensor_shape = source_tensor.sizes();
    c10::IntArrayRef target_tensor_shape = target_tensor.sizes();

    int B = source_tensor_shape[0];
    int N = source_tensor_shape[1];
    int M = target_tensor_shape[1];

    auto dist = -2 * torch::matmul(source_tensor, target_tensor.permute({0, 2, 1}));

    dist += torch::sum(source_tensor.pow(2), -1).view(
      {B, N, 1});

    dist += torch::sum(target_tensor.pow(2), -1).view(
      {B, 1, M});

    return dist;
  }

  at::Tensor index_points(at::Tensor points, at::Tensor idx)
  {
    c10::IntArrayRef points_shape = points.sizes();
    c10::IntArrayRef idx_shape = idx.sizes();
    at::Tensor extracted_tensor;

    if (idx.sizes().size() == 2) {
      extracted_tensor = at::zeros(
        {points_shape[0],
          idx_shape[1],
          points_shape[2]
        },
        idx.device());
      for (int i = 0; i < idx_shape[0]; i++) {
        for (int j = 0; j < idx_shape[1]; j++) {
          pcl::PointXYZRGB crr_point;
          int index = idx.index({i, j}).item<int>();
          at::Tensor sampled_point = points.index({i, index});
          extracted_tensor.index_put_({i, j}, sampled_point);
        }
      }
    } else {
      extracted_tensor = at::zeros(
        {points_shape[0],
          idx_shape[1],
          idx_shape[2],
          points_shape[2]
        },
        idx.device());

      for (int i = 0; i < idx_shape[0]; i++) {
        for (int j = 0; j < idx_shape[1]; j++) {
          for (int k = 0; k < idx_shape[2]; k++) {
            pcl::PointXYZRGB crr_point;
            int index = idx.index({i, j, k}).item<int>();
            at::Tensor sampled_point = points.index({i, index});
            extracted_tensor.index_put_({i, j, k}, sampled_point);
          }
        }
      }
    }
    return extracted_tensor;
  }

  at::Tensor farthest_point_sample(at::Tensor input_tensor, int num_samples)
  {
    torch::Device device = input_tensor.device();
    c10::IntArrayRef input_shape = input_tensor.sizes();
    c10::IntArrayRef output_shape = {input_shape.front(), num_samples};

    //  options for indice tensors
    auto options =
      torch::TensorOptions()
      .dtype(torch::kLong)
      .device(device);

    at::Tensor centroids = at::zeros(output_shape, options);
    at::Tensor distance =
      at::ones(
      {input_shape[0], input_shape[1]},
      torch::dtype(torch::kFloat32).device(device)).multiply(1e10);
    at::Tensor farthest =
      at::randint(0, input_shape[1], {input_shape.front(), }, options);
    at::Tensor batch_indices = at::arange(0, input_shape.front(), options);

    for (int i = 0; i < num_samples; i++) {
      centroids.index_put_({torch::indexing::Slice(), i}, farthest);
      at::Tensor centroid =
        input_tensor.index(
        {batch_indices, farthest, torch::indexing::Slice()})
        .view({input_shape.front(), 1, 3});

      at::Tensor dist = torch::sum(
        (input_tensor.subtract(centroid))
        .pow(2), -1);
      at::Tensor mask = dist < distance;
      distance.index_put_({mask}, dist.index({mask}));
      farthest = std::get<1>(torch::max(distance, -1));
    }
    return centroids;
  }

  void check_avail_device()
  {
    torch::Device device = torch::kCPU;
    if (torch::cuda::is_available()) {
      std::cout << "CUDA is available! Training on GPU possible." << std::endl;
      device = torch::kCUDA;
    } else {
      std::cout << "CUDA is NOT available! Training on CPU possible." << std::endl;
    }
  }

  at::Tensor query_ball_point(double radius, int nsample, at::Tensor xyz, at::Tensor new_xyz)
  {
    c10::IntArrayRef xyz_shape = xyz.sizes();
    c10::IntArrayRef new_xyz_shape = new_xyz.sizes();

    int B, N, C, S;
    B = xyz_shape[0];
    N = xyz_shape[1];
    C = xyz_shape[2];
    S = new_xyz_shape[1];

    auto group_idx =
      torch::arange(
      {N},
      torch::dtype(torch::kLong).device(xyz.device())).view({1, 1, N}).repeat({B, S, 1});

    auto sqrdists = square_distance(new_xyz, xyz);

    group_idx.index_put_({sqrdists > std::pow(radius, 2)}, N);

    group_idx = std::get<0>(group_idx.sort(-1)).index(
      {torch::indexing::Slice(), torch::indexing::Slice(),
        torch::indexing::Slice(torch::indexing::None, nsample)});

    auto group_first = group_idx
      .index({torch::indexing::Slice(), torch::indexing::Slice(), 0})
      .view({B, S, 1})
      .repeat({1, 1, nsample});

    auto mask = group_idx == N;
    group_idx.index_put_({mask}, group_first.index({mask}) );

    return group_idx;
  }

  std::pair<at::Tensor, at::Tensor> sample_and_group(
    int npoint, double radius, int nsample,
    at::Tensor xyz, at::Tensor points)
  {

    c10::IntArrayRef xyz_shape = xyz.sizes();
    int D = 0;

    if (points.sizes().size()) {
      c10::IntArrayRef points_shape = points.sizes();
      D = points_shape[2];
    }

    int B, N, C, S;
    B = xyz_shape[0];
    N = xyz_shape[1];
    C = xyz_shape[2];
    S = npoint;

    auto fps_sampled_indices = farthest_point_sample(xyz, npoint);
    auto new_xyz = index_points(xyz, fps_sampled_indices);
    auto idx = query_ball_point(radius, nsample, xyz, new_xyz);
    auto grouped_xyz = index_points(xyz, idx);
    auto grouped_xyz_norm = grouped_xyz - new_xyz.view({B, S, 1, C});

    at::Tensor new_points = at::zeros(
      {B, npoint, nsample, C + D},
      xyz.device());

    if (points.sizes().size()) {
      auto grouped_points = index_points(points, idx);
      grouped_points = grouped_points.view({B, npoint, nsample, D});
      new_points = torch::cat({grouped_xyz, grouped_points}, -1);
    } else {
      new_points = grouped_xyz;
    }

    return std::make_pair(new_xyz, new_points);
  }

  void torch_tensor_to_pcl_cloud(
    const at::Tensor * input_tensor,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> point_color)
  {
    c10::IntArrayRef input_shape = input_tensor->sizes();
    if (input_shape.size() == 3) {
      for (int i = 0; i < input_shape[0]; i++) {
        for (int j = 0; j < input_shape[1]; j++) {
          pcl::PointXYZRGB crr_point;
          crr_point.x = input_tensor->index({i, j, 0}).item<float>();
          crr_point.y = input_tensor->index({i, j, 1}).item<float>();
          crr_point.z = input_tensor->index({i, j, 2}).item<float>();
          crr_point.r = point_color.at(0);
          crr_point.g = point_color.at(1);
          crr_point.b = point_color.at(2);
          cloud->points.push_back(crr_point);
        }
      }
    } else {
      for (int i = 0; i < input_shape[0]; i++) {
        for (int j = 0; j < input_shape[1]; j++) {
          for (int k = 0; k < input_shape[2]; k++) {
            pcl::PointXYZRGB crr_point;
            crr_point.x = input_tensor->index({i, j, k, 0}).item<float>();
            crr_point.y = input_tensor->index({i, j, k, 1}).item<float>();
            crr_point.z = input_tensor->index({i, j, k, 2}).item<float>();
            crr_point.r = point_color.at(0);
            crr_point.g = point_color.at(1);
            crr_point.b = point_color.at(2);
            cloud->points.push_back(crr_point);
          }
        }
      }
    }
    cloud->width = 1;
    cloud->height = cloud->points.size();
  }

  at::Tensor load_pcl_as_torch_tensor(
    const std::string cloud_filename, int N, torch::Device device)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    if (!pcl::io::loadPCDFile(cloud_filename, cloud)) {
      std::cout << "Gonna load a cloud with " << cloud.points.size() << " points" << std::endl;
    } else {
      std::cerr << "Could not read PCD file: " << cloud_filename << std::endl;
    }
    // Convert cloud to a tensor with shape of [B,N,C]
    // Determine batches
    int B = std::floor(cloud.points.size() / N);
    at::Tensor cloud_as_tensor = torch::zeros({B, N, 3}, device);
    for (int i = 0; i < B; i++) {
      for (int j = 0; j < N; j++) {
        if (i * N + j < cloud.points.size()) {
          auto crr_point = cloud.points[i * N + j];
          auto point_tensor = at::zeros({1, 3}, device);
          point_tensor.index_put_({0, 0}, crr_point.x);
          point_tensor.index_put_({0, 1}, crr_point.y);
          point_tensor.index_put_({0, 2}, crr_point.z);
          cloud_as_tensor.index_put_(
            {i, j}, point_tensor);
        }
      }
    }
    return cloud_as_tensor;
  }

  at::Tensor load_pcl_as_torch_tensor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int N, torch::Device device)
  {
    // Convert cloud to a tensor with shape of [B,N,C]
    // Determine batches
    int B = std::floor(cloud->points.size() / N);
    at::Tensor cloud_as_tensor = torch::zeros({B, N, 3}, device);
    for (int i = 0; i < B; i++) {
      for (int j = 0; j < N; j++) {
        if (i * N + j < cloud->points.size()) {
          auto crr_point = cloud->points[i * N + j];
          auto point_tensor = at::zeros({1, 3}, device);
          point_tensor.index_put_({0, 0}, crr_point.x);
          point_tensor.index_put_({0, 1}, crr_point.y);
          point_tensor.index_put_({0, 2}, crr_point.z);
          cloud_as_tensor.index_put_(
            {i, j}, point_tensor);
        }
      }
    }
    return cloud_as_tensor;
  }
}  // namespace pointnet2_utils
