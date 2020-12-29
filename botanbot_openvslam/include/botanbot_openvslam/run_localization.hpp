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

/*
 * Parts of code has been taken from
 *      https://github.com/xdspacelab/openvslam/tree/develop/ros
 *      Institute: AIST in Japan, XDSPACE
 */

#ifndef BOTANBOT_OPENVSLAM__RUN_LOCALIZATION_HPP_
#define BOTANBOT_OPENVSLAM__RUN_LOCALIZATION_HPP_

#include <pangolin_viewer/viewer.h>

#include <openvslam/system.h>
#include <openvslam/config.h>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <chrono>
#include <numeric>
#include <memory>
#include <vector>
#include <string>

namespace botanbot_openvslam
{

class RunLocalization : public rclcpp::Node
{
public:
  RunLocalization();
  ~RunLocalization();

  void rgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth);
  void monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void executeViewerPangolinThread();

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>
    RGBDApprxTimeSyncPolicy;
  typedef message_filters::Synchronizer<RGBDApprxTimeSyncPolicy> RGBDApprxTimeSyncer;

private:
  std::shared_ptr<openvslam::system> SLAM_;
  std::shared_ptr<openvslam::config> cfg_;
  std::shared_ptr<RGBDApprxTimeSyncer> rgbd_approx_time_syncher_;
  std::shared_ptr<cv::Mat> mask_;
  std::shared_ptr<pangolin_viewer::viewer> pangolin_viewer_;
  std::shared_ptr<std::thread> pangolin_viewer_thread_;

  std::chrono::steady_clock::time_point initial_time_stamp_;
  std::vector<double> track_times_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgbd_color_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgbd_depth_image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_image_subscriber_;

  std::string vocab_file_path_;
  std::string setting_file_path_;
  std::string mask_img_path_;
  std::string prebuilt_map_path_;
  bool debug_mode_;
  bool eval_log_;
  bool enable_mapping_module_;
};

}  // namespace botanbot_openvslam
#endif  // BOTANBOT_OPENVSLAM__RUN_LOCALIZATION_HPP_
