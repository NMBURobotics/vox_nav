#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>
#include "botanbot_openvslam/popl.hpp"

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

class RunSlam : public rclcpp::Node
{
private:
  openvslam::system SLAM_;
  std::shared_ptr<openvslam::config> cfg_;

  rmw_qos_profile_t custom_qos_;
  std::chrono::steady_clock::time_point tp_0_;
  cv::Mat mask_;
  std::vector<double> track_times_;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sf_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sf_;

  // ros susbcriber to get camera image
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_image_subscriber_;

  std::shared_ptr<pangolin_viewer::viewer> viewer_;

  std::string vocab_file_path_;
  std::string setting_file_path_;
  std::string mask_img_path_;
  std::string map_db_path_;
  bool debug_mode_;
  bool eval_log_;

public:
  RunSlam(
    const std::shared_ptr<openvslam::config> & cfg, const std::string & vocab_file_path,
    const std::string & mask_img_path);
  ~RunSlam();

  void rgbd_callback(
    const sensor_msgs::msg::Image::SharedPtr & color,
    const sensor_msgs::msg::Image::SharedPtr & depth);

  void mono_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

};

RunSlam::RunSlam(
  const std::shared_ptr<openvslam::config> & cfg, const std::string & vocab_file_path,
  const std::string & mask_img_path)
:  Node("run_slam_rclcpp_node"), SLAM_(cfg, vocab_file_path), cfg_(cfg),
  custom_qos_(rmw_qos_profile_default),
  tp_0_(std::chrono::steady_clock::now()),
  mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE))
{

  this->declare_parameter("vocab_file_path", "none");
  this->declare_parameter("setting_file_path", "none");
  this->declare_parameter("mask_img_path", "none");
  this->declare_parameter("map_db_path", "");
  this->declare_parameter("debug_mode", true);
  this->declare_parameter("eval_log", true);

  vocab_file_path_ = this->get_parameter("vocab_file_path").as_string();
  setting_file_path_ = this->get_parameter("setting_file_path").as_string();
  mask_img_path_ = this->get_parameter("mask_img_path").as_string();
  map_db_path_ = this->get_parameter("map_db_path").as_string();
  debug_mode_ = this->get_parameter("debug_mode").as_bool();
  eval_log_ = this->get_parameter("eval_log").as_bool();

  custom_qos_.depth = 1;
  SLAM_.startup();

  try {
    cfg_ = std::make_shared<openvslam::config>(setting_file_path->value());
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  viewer_ = std::make_shared<pangolin_viewer::viewer>(
    cfg_, &SLAM_,
    SLAM_.get_frame_publisher(), SLAM_.get_map_publisher());

  // run tracking
  if (cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
    camera_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/color/image_raw", rclcpp::SystemDefaultsQoS(),
      std::bind(&RunSlam::mono_callback, this, std::placeholders::_1));
  } else if ((cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD)) {

    color_sf_.subscribe(this, "camera/color/image_raw");
    depth_sf_.subscribe(this, "camera/depth/image_raw");

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
        sensor_msgs::msg::Image> approximate_policy;
    message_filters::Synchronizer<approximate_policy> syncApproximate(approximate_policy(
        10), color_sf_, depth_sf_);
    syncApproximate.registerCallback(&RunSlam::rgbd_callback, this);

  } else {
    throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
  }
}

RunSlam::~RunSlam()
{
  if (eval_log_) {
    // output the trajectories for evaluation
    SLAM_.save_frame_trajectory("frame_trajectory.txt", "TUM");
    SLAM_.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
    // output the tracking times for evaluation
    std::ofstream ofs("track_times.txt", std::ios::out);
    if (ofs.is_open()) {
      for (const auto track_time : track_times) {
        ofs << track_time << std::endl;
      }
      ofs.close();
    }
  }

  if (!map_db_path.empty()) {
    // output the map database
    SLAM.save_map_database(map_db_path);
  }

  SLAM_.shutdown();
}

void RunSlam::rgbd_callback(
  const sensor_msgs::msg::Image::SharedPtr & color,
  const sensor_msgs::msg::Image::SharedPtr & depth)
{
  auto colorcv = cv_bridge::toCvShare(color)->image;
  auto depthcv = cv_bridge::toCvShare(depth)->image;
  if (colorcv.empty() || depthcv.empty()) {
    return;
  }

  const auto tp_1 = std::chrono::steady_clock::now();
  const auto timestamp =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

  // input the current frame and estimate the camera pose
  SLAM_.feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

  const auto tp_2 = std::chrono::steady_clock::now();

  const auto track_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
  track_times_.push_back(track_time);
}


void RunSlam::mono_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::cout << "Entering to  mono::callback .. " << std::endl;

  const auto tp_1 = std::chrono::steady_clock::now();
  const auto timestamp =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

  // input the current frame and estimate the camera pose
  SLAM_.feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

  const auto tp_2 = std::chrono::steady_clock::now();

  const auto track_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
  track_times_.push_back(track_time);

  viewer.run();
  if (SLAM.terminate_is_requested()) {
    // wait until the loop BA is finished
    while (SLAM.loop_BA_is_running()) {
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
  }

  viewer.request_terminate();
  thread.join();
}

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);


  return EXIT_SUCCESS;
}
