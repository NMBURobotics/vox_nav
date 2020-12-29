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

/**
 * @brief namespace for openvslam clients.
 *
 */
namespace botanbot_openvslam
{

/**
 * @brief A RCLCPP node for performing localization on a prebuilt map with openvslam
 *
 */
class RunLocalization : public rclcpp::Node
{
public:
/**
 * @brief Construct a new Run Localization object
 *
 */
  RunLocalization();

  /**
   * @brief Destroy the Run Localization object
   *
   */
  ~RunLocalization();

  /**
   * @brief RGBD callback, subscries to both color and depth image with approx. time syncer, and runs openvslam localization.
   *
   * @param color
   * @param depth
   */
  void rgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth);

  /**
   * @brief mono callback, subscries to an mono image and runs openvslam localization on a prebuild map.
   *
   * @param msg
   */
  void monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  /**
   * @brief A dedicated thread to run pangolin viewer.
   *
   */
  void executeViewerPangolinThread();

  /**
   * @brief Typedefs for shortnening Approx time Syncer initialization.
   *
   */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>
    RGBDApprxTimeSyncPolicy;
  typedef message_filters::Synchronizer<RGBDApprxTimeSyncPolicy> RGBDApprxTimeSyncer;

private:
  // shared ptr to object to perform actual localization
  std::shared_ptr<openvslam::system> SLAM_;
  // shared ptr to configuration that was fromed ith files under config directory of this package
  std::shared_ptr<openvslam::config> cfg_;
  // shared ptr to Approx time syncer , message filter type, in order to register the rgbdCallback
  std::shared_ptr<RGBDApprxTimeSyncer> rgbd_approx_time_syncher_;
  // shared ptr to a mask image , void if there was no mask image provided in config file
  std::shared_ptr<cv::Mat> mask_;
  // shared ptr to pnagolin viewer object
  std::shared_ptr<pangolin_viewer::viewer> pangolin_viewer_;
  // shared ptr to dedicated thread for pagolin viewer
  std::shared_ptr<std::thread> pangolin_viewer_thread_;
  // keep a copy initial time stamp
  std::chrono::steady_clock::time_point initial_time_stamp_;
  //
  std::vector<double> track_times_;
  // subscriber for color cam in case rgbd camera model localization
  message_filters::Subscriber<sensor_msgs::msg::Image> rgbd_color_image_subscriber_;
  // subscriber for depth cam in case rgbd camera model localization
  message_filters::Subscriber<sensor_msgs::msg::Image> rgbd_depth_image_subscriber_;
  // subscriber for mono cam in case monocular camera model localization
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_image_subscriber_;
  // parameter to hold full path to vocab.dbow2 file
  std::string vocab_file_path_;
  // parameter to hold full path to slam_config.yaml file
  std::string setting_file_path_;
  // parameter to hold  full path to mask image
  std::string mask_img_path_;
  // parameter to hold  full path to a prebuilt map
  std::string prebuilt_map_path_;
  // enable/disable debug messages from openvslam itself
  bool debug_mode_;
  // if true evalution file will be dumped
  bool eval_log_;
  // enable/disable mappin while localzating
  bool enable_mapping_module_;
};

}  // namespace botanbot_openvslam
#endif  // BOTANBOT_OPENVSLAM__RUN_LOCALIZATION_HPP_
