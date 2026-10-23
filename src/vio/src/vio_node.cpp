// Copyright (C) 2019, Aerial Robotics Group, HKUST.
// Derived from VINS; licensed under GNU GPL v3.0 (see LICENSE).
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <vins/msg/local_sensor_external_trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <deque>
#include <cmath>
#include <memory>
#include <csignal>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void OnSignal(int) { stop_requested = 1; }
}
#include "estimator/estimator.h"
#include "utility/visualization.h"
#include "utility/gnss_time_sync.hpp"

// ROS owns all transport queues. The estimator owns its optimization worker.
// A single-threaded executor serializes callbacks, so no transport buffer is shared across threads.
class VioTransport {
 public:
  explicit VioTransport(const rclcpp::Node::SharedPtr& node) : node_(node), estimator_(node) {
    tolerance_ = node_->declare_parameter<double>("image_sync_tolerance", 0.003);
    detector_timeout_ = node_->declare_parameter<double>("detector_wait_timeout", 0.5);
    if (tolerance_ < 0 || detector_timeout_ <= 0)
      throw std::runtime_error("image_sync_tolerance must be nonnegative and detector_wait_timeout positive");
    estimator_.setParameter();
    registerPub(node_);
    subscribers_.push_back(node_->create_subscription<std_msgs::msg::Bool>(
        "~/restart", rclcpp::QoS(1).reliable(), std::bind(&VioTransport::OnRestart, this, std::placeholders::_1)));
    const auto sensor_qos = rclcpp::SensorDataQoS();
    if (USE_IMU || (USE_WHEEL && w_replace))
      subscribers_.push_back(node_->create_subscription<sensor_msgs::msg::Imu>(
          IMU_TOPIC, sensor_qos, std::bind(&VioTransport::OnImu, this, std::placeholders::_1)));
    if (USE_WHEEL)
      subscribers_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
          WHEEL_TOPIC, sensor_qos, std::bind(&VioTransport::OnWheel, this, std::placeholders::_1)));
    subscribers_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
        IMAGE0_TOPIC, sensor_qos, std::bind(&VioTransport::OnImage0, this, std::placeholders::_1)));
    if (STEREO || DEPTH)
      subscribers_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
          IMAGE1_TOPIC, sensor_qos, std::bind(&VioTransport::OnImage1, this, std::placeholders::_1)));
    if (USE_YOLO)
      subscribers_.push_back(node_->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
          "/darknet_ros/bounding_boxes", rclcpp::QoS(100).reliable(),
          std::bind(&VioTransport::OnBoxes, this, std::placeholders::_1)));
    if (GNSS_ENABLE) {
      subscribers_.push_back(node_->create_subscription<gnss_comm::msg::GnssEphemMsg>(
          GNSS_EPHEM_TOPIC, sensor_qos, std::bind(&VioTransport::OnEphemeris, this, std::placeholders::_1)));
      subscribers_.push_back(node_->create_subscription<gnss_comm::msg::GnssGloEphemMsg>(
          GNSS_GLO_EPHEM_TOPIC, sensor_qos, std::bind(&VioTransport::OnGlonassEphemeris, this, std::placeholders::_1)));
      subscribers_.push_back(node_->create_subscription<gnss_comm::msg::StampedFloat64Array>(
          GNSS_IONO_PARAMS_TOPIC, sensor_qos, std::bind(&VioTransport::OnIonosphere, this, std::placeholders::_1)));
      subscribers_.push_back(node_->create_subscription<gnss_comm::msg::GnssMeasMsg>(
          GNSS_MEAS_TOPIC, sensor_qos, std::bind(&VioTransport::OnGnss, this, std::placeholders::_1)));
      if (GNSS_LOCAL_ONLINE_SYNC) {
        subscribers_.push_back(node_->create_subscription<gnss_comm::msg::GnssTimePulseInfoMsg>(
            GNSS_TP_INFO_TOPIC, sensor_qos, std::bind(&VioTransport::OnTimePulse, this, std::placeholders::_1)));
        subscribers_.push_back(node_->create_subscription<vins::msg::LocalSensorExternalTrigger>(
            LOCAL_TRIGGER_INFO_TOPIC, sensor_qos, std::bind(&VioTransport::OnTrigger, this, std::placeholders::_1)));
      } else {
        estimator_.inputGNSSTimeDiff(GNSS_LOCAL_TIME_DIFF);
        time_valid_ = true;
      }
    }
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), std::bind(&VioTransport::OnTimer, this));
  }
  ~VioTransport() {
    estimator_.stop();
    releasePub();
  }

 private:
  using Image = sensor_msgs::msg::Image;
  using ImagePtr = Image::ConstSharedPtr;
  using Boxes = darknet_ros_msgs::msg::BoundingBoxes;
  using BoxesPtr = Boxes::ConstSharedPtr;
  void OnRestart(const std_msgs::msg::Bool::ConstSharedPtr message) {
    if (!message->data) return;
    estimator_.stop();
    images0_.clear(); images1_.clear(); boxes_.clear(); imus_.clear(); wheels_.clear();
    last_image_ = last_imu_ = last_wheel_ = -1;
    estimator_.clearState();
    time_valid_ = GNSS_ENABLE && !GNSS_LOCAL_ONLINE_SYNC;
    pulse_valid_ = false;
    if (time_valid_) estimator_.inputGNSSTimeDiff(GNSS_LOCAL_TIME_DIFF);
    estimator_.setParameter();
  }
  struct PendingImage { ImagePtr message; std::chrono::steady_clock::time_point received; };
  void OnImage0(const ImagePtr image) {
    if (!images0_.empty() && rclcpp::Time(image->header.stamp) <= rclcpp::Time(images0_.back().message->header.stamp)) {
      RCLCPP_WARN(node_->get_logger(), "Dropping non-increasing primary image timestamp"); return;
    }
    images0_.push_back({image, std::chrono::steady_clock::now()});
    DrainImages();
  }
  void OnImage1(const ImagePtr image) {
    if (!images1_.empty() && rclcpp::Time(image->header.stamp) <= rclcpp::Time(images1_.back()->header.stamp)) {
      RCLCPP_WARN(node_->get_logger(), "Dropping non-increasing secondary image timestamp"); return;
    }
    images1_.push_back(image);
    while (!images1_.empty() && rclcpp::Time(images1_.front()->header.stamp).seconds() < last_image_ - tolerance_)
      images1_.pop_front();
    DrainImages();
  }
  void OnBoxes(const BoxesPtr message) {
    if (!boxes_.empty() && rclcpp::Time(message->image_header.stamp) <= rclcpp::Time(boxes_.back()->image_header.stamp)) return;
    boxes_.push_back(message);
    while (!boxes_.empty() && rclcpp::Time(boxes_.front()->image_header.stamp).seconds() < last_image_ - tolerance_)
      boxes_.pop_front();
    DrainImages();
  }
  void OnTimer() { DrainImages(); }
  static cv::Mat ToGray(const ImagePtr& image) {
    if (image->encoding == "8UC1") {
      auto copy = std::make_shared<Image>(*image);
      copy->encoding = sensor_msgs::image_encodings::MONO8;
      return cv_bridge::toCvCopy(copy, sensor_msgs::image_encodings::MONO8)->image;
    }
    return cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8)->image;
  }
  static cv::Mat ToDepth(const ImagePtr& image) {
    auto converted = cv_bridge::toCvCopy(image);
    if (converted->image.type() == CV_16UC1) return converted->image;
    if (converted->image.type() == CV_32FC1) {
      cv::Mat millimeters;
      converted->image.convertTo(millimeters, CV_16UC1, 1000.0);
      return millimeters;
    }
    throw std::runtime_error("Depth image must be 16UC1 millimeters or 32FC1 meters");
  }
  void DrainImages() {
    while (rclcpp::ok() && !images0_.empty()) {
      const auto& primary = images0_.front();
      const double stamp = rclcpp::Time(primary.message->header.stamp).seconds();
      if (stamp <= last_image_) { images0_.pop_front(); continue; }
      const bool timed_out = std::chrono::duration<double>(std::chrono::steady_clock::now() - primary.received).count() > detector_timeout_;
      ImagePtr second;
      if (STEREO || DEPTH) {
        while (!images1_.empty() && rclcpp::Time(images1_.front()->header.stamp).seconds() < stamp - tolerance_) images1_.pop_front();
        if (images1_.empty()) {
          if (!timed_out) return;
          RCLCPP_WARN(node_->get_logger(), "Dropping image without synchronized secondary image");
          images0_.pop_front(); continue;
        }
        if (rclcpp::Time(images1_.front()->header.stamp).seconds() > stamp + tolerance_) { images0_.pop_front(); continue; }
        second = images1_.front();
      }
      BoxesPtr detection;
      if (USE_YOLO) {
        while (!boxes_.empty() && rclcpp::Time(boxes_.front()->image_header.stamp).seconds() < stamp - tolerance_) boxes_.pop_front();
        if (boxes_.empty()) {
          if (!timed_out) return;
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
              "Dropping image: no synchronized YOLO result; start darknet_ros and check its image topic");
          images0_.pop_front(); if (second) images1_.pop_front(); continue;
        }
        if (rclcpp::Time(boxes_.front()->image_header.stamp).seconds() > stamp + tolerance_) {
          images0_.pop_front(); if (second) images1_.pop_front(); continue;
        }
        detection = boxes_.front();
      }
      try {
        const cv::Mat left = ToGray(primary.message);
        const cv::Mat right = second ? (DEPTH ? ToDepth(second) : ToGray(second)) : cv::Mat();
        if (USE_LINE) estimator_.inputImagewithline(stamp, left, right);
        else if (USE_YOLO) estimator_.inputImagebox(stamp, detection, left, right);
        else estimator_.inputImage(stamp, left, right);
      } catch (const std::exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "Rejected image at %.9f: %s", stamp, error.what());
      }
      last_image_ = stamp; images0_.pop_front();
      if (second) images1_.pop_front(); if (detection) boxes_.pop_front();
    }
  }
  void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr message) {
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    if (stamp <= last_imu_) { RCLCPP_WARN(node_->get_logger(), "Dropping non-increasing IMU timestamp"); return; }
    last_imu_ = stamp;
    if (USE_WHEEL && w_replace) {
      imus_.push_back(message);
      while (imus_.size() > 2 && stamp - rclcpp::Time(imus_[1]->header.stamp).seconds() > 2.0) imus_.pop_front();
    }
    if (USE_IMU) estimator_.inputIMU(stamp,
      Vector3d(message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z),
      Vector3d(message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z));
    DrainWheels();
  }
  void OnWheel(const nav_msgs::msg::Odometry::ConstSharedPtr message) {
    if (rclcpp::Time(message->header.stamp).seconds() <= last_wheel_) { RCLCPP_WARN(node_->get_logger(), "Dropping non-increasing wheel timestamp"); return; }
    last_wheel_ = rclcpp::Time(message->header.stamp).seconds(); wheels_.push_back(message); DrainWheels();
  }
  void DrainWheels() {
    while (!wheels_.empty()) {
      const auto message = wheels_.front();
      const double stamp = rclcpp::Time(message->header.stamp).seconds();
      double yaw_rate = message->twist.twist.angular.z;
      if (w_replace) {
        if (imus_.empty() || rclcpp::Time(imus_.back()->header.stamp).seconds() < stamp) return;
        while (imus_.size() > 1 && rclcpp::Time(imus_[1]->header.stamp).seconds() <= stamp) imus_.pop_front();
        if (rclcpp::Time(imus_.front()->header.stamp).seconds() > stamp) {
          RCLCPP_WARN(node_->get_logger(), "Dropping wheel sample without preceding IMU for yaw interpolation");
          wheels_.pop_front(); continue;
        }
        const auto& before = imus_.front(); yaw_rate = -before->angular_velocity.y;
        if (imus_.size() > 1) {
          const auto& after = imus_[1];
          const double before_stamp = rclcpp::Time(before->header.stamp).seconds();
          const double fraction = (stamp - before_stamp) / (rclcpp::Time(after->header.stamp).seconds() - before_stamp);
          yaw_rate += fraction * (-after->angular_velocity.y - yaw_rate);
        }
      }
      estimator_.inputWheel(stamp,
        Vector3d(message->twist.twist.linear.x, message->twist.twist.linear.y, message->twist.twist.linear.z),
        Vector3d(message->twist.twist.angular.x, message->twist.twist.angular.y, yaw_rate));
      const auto& p = message->pose.pose.position; const auto& q = message->pose.pose.orientation;
      auto header = message->header; header.frame_id = "world";
      pubWheelRawodom(Vector3d(p.x, p.y, p.z), Eigen::Quaterniond(q.w, q.x, q.y, q.z), header);
      estimator_.inputrawodom(stamp, Vector3d(p.x, p.y, 0)); wheels_.pop_front();
    }
  }
  void OnEphemeris(const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr message) { estimator_.inputEphem(gnss_comm::msg2ephem(message)); }
  void OnGlonassEphemeris(const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr message) { estimator_.inputEphem(gnss_comm::msg2glo_ephem(message)); }
  void OnIonosphere(const gnss_comm::msg::StampedFloat64Array::ConstSharedPtr message) {
    if (message->data.size() != 8) { RCLCPP_ERROR(node_->get_logger(), "GNSS ionosphere message requires 8 coefficients"); return; }
    estimator_.inputIonoParams(rclcpp::Time(message->header.stamp).seconds(), message->data);
  }
  void OnGnss(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr message) {
    if (!time_valid_) { RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Waiting for GNSS/local PPS synchronization"); return; }
    auto observations = gnss_comm::msg2meas(message);
    if (observations.empty()) return;
    const double stamp = gnss_comm::time2sec(observations.front()->time);
    estimator_.inputGNSS(stamp, std::move(observations));
  }
  void OnTimePulse(const gnss_comm::msg::GnssTimePulseInfoMsg::ConstSharedPtr message) {
    using namespace gnss_comm;
    const auto time = vins_adapter::ToGpsPulseTime(gpst2time(message->time.week, message->time.tow), message->time_sys, message->utc_based);
    if (!time) { RCLCPP_ERROR(node_->get_logger(), "Unknown GNSS PPS clock scale (expected gnss_comm T_SYS_*)"); return; }
    next_pulse_ = time2sec(*time); pulse_received_ = std::chrono::steady_clock::now(); pulse_valid_ = true;
  }
  void OnTrigger(const vins::msg::LocalSensorExternalTrigger::ConstSharedPtr message) {
    if (!pulse_valid_ || std::chrono::duration<double>(std::chrono::steady_clock::now() - pulse_received_).count() > 1.0) {
      RCLCPP_WARN(node_->get_logger(), "Local trigger has no fresh GNSS time-pulse information"); return;
    }
    estimator_.inputGNSSTimeDiff(next_pulse_ - rclcpp::Time(message->header.stamp).seconds());
    pulse_valid_ = false; time_valid_ = true;
  }
  rclcpp::Node::SharedPtr node_;
  Estimator estimator_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<PendingImage> images0_;
  std::deque<ImagePtr> images1_;
  std::deque<BoxesPtr> boxes_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imus_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> wheels_;
  double last_image_ = -1, last_imu_ = -1, last_wheel_ = -1;
  double tolerance_, detector_timeout_, next_pulse_ = 0;
  bool time_valid_ = false, pulse_valid_ = false;
  std::chrono::steady_clock::time_point pulse_received_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  try {
    auto node = std::make_shared<rclcpp::Node>("vins_estimator");
    const auto config = node->declare_parameter<std::string>("config_file", "");
    if (config.empty()) throw std::runtime_error("Set the required config_file ROS parameter to a VINS OpenCV YAML configuration");
    readParameters(node, config);
    VioTransport transport(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) executor.spin_once(std::chrono::milliseconds(100));
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("vins_estimator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
