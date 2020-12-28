#include <botanbot_openvslam/openvslam_ros.hpp>

#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace openvslam_ros
{
system::system(
  const std::shared_ptr<openvslam::config> & cfg, const std::string & vocab_file_path,
  const std::string & mask_img_path)
: SLAM_(cfg, vocab_file_path), cfg_(cfg), node_(std::make_shared<rclcpp::Node>("run_slam")),
  custom_qos_(rmw_qos_profile_default),
  tp_0_(std::chrono::steady_clock::now()),
  mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE))
{
  custom_qos_.depth = 1;
  exec_.add_node(node_);
  std::cout << "Ended system constructor .. " << std::endl;

}

mono::mono(
  const std::shared_ptr<openvslam::config> & cfg, const std::string & vocab_file_path,
  const std::string & mask_img_path)
: system(cfg, vocab_file_path, mask_img_path)
{
  sub_ = image_transport::create_subscription(
    node_.get(), "/camera/color/image_raw",
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      callback(msg);
    }, "raw", custom_qos_);
  std::cout << "Ended mono constructor .. " << std::endl;
}
void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
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
}

rgbd::rgbd(
  const std::shared_ptr<openvslam::config> & cfg, const std::string & vocab_file_path,
  const std::string & mask_img_path)
: system(cfg, vocab_file_path, mask_img_path)
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image> approximate_policy;

  message_filters::Synchronizer<approximate_policy> syncApproximate(approximate_policy(
      10), color_sf_, depth_sf_);

  color_sf_.subscribe(node_.get(), "/camera/color/image_raw", rmw_qos_profile_default);
  depth_sf_.subscribe(node_.get(), "/camera/depth/image_raw", rmw_qos_profile_default);
  syncApproximate.registerCallback(&rgbd::callback, this);
}

void rgbd::callback(
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
} // namespace openvslam_ros
