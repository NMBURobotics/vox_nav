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

#include "vox_nav_openvslam/run_slam.hpp"

#include <memory>
#include <chrono>
#include <string>

namespace vox_nav_openvslam
{
  RunSlam::RunSlam()
  :  Node("run_slam_rclcpp_node")
  {
    this->declare_parameter("vocab_file_path", "none");
    this->declare_parameter("setting_file_path", "none");
    this->declare_parameter("mask_img_path", "");
    this->declare_parameter("map_db_path", "");
    this->declare_parameter("debug_mode", true);
    this->declare_parameter("eval_log", true);
    this->declare_parameter("write_map_info", true);
    this->declare_parameter("map_info_path", "");

    vocab_file_path_ = this->get_parameter("vocab_file_path").as_string();
    setting_file_path_ = this->get_parameter("setting_file_path").as_string();
    mask_img_path_ = this->get_parameter("mask_img_path").as_string();
    map_db_path_ = this->get_parameter("map_db_path").as_string();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    eval_log_ = this->get_parameter("eval_log").as_bool();
    write_map_info_ = this->get_parameter("write_map_info").as_bool();
    map_info_path_ = this->get_parameter("map_info_path").as_string();

    mask_ = std::make_shared<cv::Mat>();
    if (!mask_img_path_.empty()) {
      try {
        mask_ = std::make_shared<cv::Mat>(cv::imread(mask_img_path_, cv::IMREAD_GRAYSCALE));
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to recieve mask image, make sure you haven given "
          "correct full path to mask image. Exception: %s",
          ex.what());
      }
    }

    try {
      cfg_ = std::make_shared<openvslam::config>(setting_file_path_);
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to initialize openvslam::config, this is a crucial failure, "
        "cannot proceed anymore. Exception: %s",
        ex.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Config OK... ");

    gps_waypoint_collector_node_ = std::make_shared<vox_nav_utilities::GPSWaypointCollector>();

    initial_time_stamp_ = std::chrono::steady_clock::now();
    SLAM_ = std::make_shared<openvslam::system>(cfg_, vocab_file_path_.c_str());
    SLAM_->startup();

    RCLCPP_INFO(get_logger(), "Setted up SLAM... ");


    // register correct callback according to seleted camera model type
    if (cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
      mono_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/color/image_raw", rclcpp::SystemDefaultsQoS(),
        std::bind(&RunSlam::monoCallback, this, std::placeholders::_1));
    } else if ((cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD)) {
      rgbd_color_image_subscriber_.subscribe(
        this, "camera/color/image_raw",
        rmw_qos_profile_sensor_data);
      rgbd_depth_image_subscriber_.subscribe(
        this, "camera/depth/image_raw",
        rmw_qos_profile_sensor_data);
      rgbd_approx_time_syncher_.reset(
        new RGBDApprxTimeSyncer(
          RGBDApprxTimeSyncPolicy(10),
          rgbd_color_image_subscriber_,
          rgbd_depth_image_subscriber_));
      rgbd_approx_time_syncher_->registerCallback(
        std::bind(
          &RunSlam::rgbdCallback, this, std::placeholders::_1,
          std::placeholders::_2));
    } else if ((cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo)) {
      rgbd_color_image_subscriber_.subscribe(
        this, "camera/left/image_raw",
        rmw_qos_profile_sensor_data);
      rgbd_depth_image_subscriber_.subscribe(
        this, "camera/right/image_raw",
        rmw_qos_profile_sensor_data);
      rgbd_approx_time_syncher_.reset(
        new RGBDApprxTimeSyncer(
          RGBDApprxTimeSyncPolicy(10),
          rgbd_color_image_subscriber_,
          rgbd_depth_image_subscriber_));
      rgbd_approx_time_syncher_->registerCallback(
        std::bind(
          &RunSlam::stereoCallback, this, std::placeholders::_1,
          std::placeholders::_2));
    } else {
      throw std::runtime_error("Invalid setup type: " + cfg_->camera_->get_setup_type_string());
    }

    rclcpp::sleep_for(std::chrono::seconds(2));

    pangolin_viewer_ = std::make_shared<pangolin_viewer::viewer>(
      openvslam::util::yaml_optional_ref(
        cfg_->yaml_node_, "PangolinViewer"), SLAM_.get(),
      SLAM_->get_frame_publisher(), SLAM_->get_map_publisher());

    pangolin_viewer_thread_ =
      std::make_shared<std::thread>(std::thread(&RunSlam::executeViewerPangolinThread, this));
    RCLCPP_INFO(get_logger(), "Constructed an instance of RunSlam node ... ");
  }

  RunSlam::~RunSlam()
  {
    if (eval_log_) {
      // output the trajectories for evaluation
      SLAM_->save_frame_trajectory("frame_trajectory.txt", "TUM");
      SLAM_->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
      // output the tracking times for evaluation
      std::ofstream ofs("track_times.txt", std::ios::out);
      if (ofs.is_open()) {
        for (const auto track_time : track_times_) {
          ofs << track_time << std::endl;
        }
        ofs.close();
      }
    }
    if (!map_db_path_.empty()) {
      // output the map database
      SLAM_->save_map_database(map_db_path_);
    }
    SLAM_->shutdown();
    pangolin_viewer_->request_terminate();
    pangolin_viewer_thread_->join();
    RCLCPP_INFO(get_logger(), "Deconstructed an instance of RunSlam node , bye... ");
  }

  void RunSlam::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left,
    const sensor_msgs::msg::Image::ConstSharedPtr & right)
  {
    if (!gps_waypoint_collector_node_->isOrientedGPSDataReady()) {
      RCLCPP_WARN(
        get_logger(),
        "Oriented GPS coordinates are not recieved yet, the initial pose of map is unknown!, "
        "spinning gps waypoint collector node and trying again ...");
      rclcpp::spin_some(gps_waypoint_collector_node_);
      rclcpp::sleep_for(std::chrono::seconds(1));
      return;
    } else {
      std::call_once(
        gps_data_recieved_flag_, [this]() {
          if (write_map_info_) {
            RCLCPP_WARN(
              get_logger(),
              "Recieved a pair of gps and quaternion data, data will be recieved and dumped "
              "to yaml file once additional map related information will be dumped yaml file: %s ",
              map_info_path_.c_str()
            );
            vox_nav_openvslam::writeMapInfotoYAML(
              map_info_path_,
              map_db_path_,
              std::string("stereo"),
              gps_waypoint_collector_node_);
          }
        });
    }
    // At this point we do have initial pose of map to be created
    auto leftcv = cv_bridge::toCvShare(left)->image;
    auto rightcv = cv_bridge::toCvShare(right)->image;
    if (leftcv.empty() || rightcv.empty()) {
      return;
    }
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp =
      std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - initial_time_stamp_).count();
    // input the current frame and estimate the camera pose
    SLAM_->feed_stereo_frame(leftcv, rightcv, timestamp, *mask_);
    const auto tp_2 = std::chrono::steady_clock::now();
    const auto track_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
  }

  void RunSlam::rgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth)
  {
    if (!gps_waypoint_collector_node_->isOrientedGPSDataReady()) {
      RCLCPP_WARN(
        get_logger(),
        "Oriented GPS coordinates are not recieved yet, the initial pose of map is unknown!, "
        "spinning gps waypoint collector node and trying again ...");
      rclcpp::spin_some(gps_waypoint_collector_node_);
      rclcpp::sleep_for(std::chrono::seconds(1));
      return;
    } else {
      std::call_once(
        gps_data_recieved_flag_, [this]() {
          if (write_map_info_) {
            RCLCPP_WARN(
              get_logger(),
              "Recieved a pair of gps and imu data, data will be recieved and dumped "
              "to yaml file once additional map related information will be dumped yaml file: %s ",
              map_info_path_.c_str()
            );
            vox_nav_openvslam::writeMapInfotoYAML(
              map_info_path_,
              map_db_path_,
              std::string("rgbd"),
              gps_waypoint_collector_node_);
          }
        });
    }
    // At this point we do have initial pose of map to be created
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
      return;
    }
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp =
      std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - initial_time_stamp_).count();
    // input the current frame and estimate the camera pose
    SLAM_->feed_RGBD_frame(colorcv, depthcv, timestamp, *mask_);
    const auto tp_2 = std::chrono::steady_clock::now();
    const auto track_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
  }

  void RunSlam::monoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp =
      std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - initial_time_stamp_).count();
    // input the current frame and estimate the camera pose
    SLAM_->feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, *mask_);
    const auto tp_2 = std::chrono::steady_clock::now();
    const auto track_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
  }

  void RunSlam::executeViewerPangolinThread()
  {
    pangolin_viewer_->run();
    if (SLAM_->terminate_is_requested()) {
      RCLCPP_INFO(get_logger(), "Shut down request receieved");
      // wait until the loop BA is finished
      while (SLAM_->loop_BA_is_running()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
        RCLCPP_INFO(get_logger(), "Spinning until BA loop is closed ...");
      }
      rclcpp::shutdown();
    }
  }
}  // namespace vox_nav_openvslam

/**
 * @brief Entry point to slam node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_openvslam::RunSlam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
