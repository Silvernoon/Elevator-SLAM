// Copyright (C) 2019, Aerial Robotics Group, HKUST.
// Derived from VINS; licensed under GNU GPL v3.0 (see LICENSE).
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vins/LocalSensorExternalTrigger.h>
#include <std_msgs/Bool.h>
#include <deque>
#include <cmath>
#include "estimator/estimator.h"
#include "utility/visualization.h"
#include "utility/gnss_time_sync.hpp"

// ROS owns all transport queues. The estimator owns its optimization worker.
// A single spinner serializes callbacks, so no transport buffer is shared across threads.
class VioTransport {
 public:
  explicit VioTransport(ros::NodeHandle node) : node_(std::move(node)) {
    node_.param("image_sync_tolerance", tolerance_, 0.003);
    node_.param("detector_wait_timeout", detector_timeout_, 0.5);
    if (tolerance_ < 0 || detector_timeout_ <= 0)
      throw std::runtime_error("image_sync_tolerance must be nonnegative and detector_wait_timeout positive");
    estimator_.setParameter();
    registerPub(node_);
    ros::TransportHints hints;
    subscribers_.push_back(node_.subscribe("restart", 1, &VioTransport::OnRestart, this));
    if (USE_TCP_NODELAY) hints.tcpNoDelay();
    if (USE_IMU || (USE_WHEEL && w_replace))
      subscribers_.push_back(node_.subscribe(IMU_TOPIC, 5000, &VioTransport::OnImu, this, hints));
    if (USE_WHEEL)
      subscribers_.push_back(node_.subscribe(WHEEL_TOPIC, 5000, &VioTransport::OnWheel, this, hints));
    subscribers_.push_back(node_.subscribe(IMAGE0_TOPIC, 100, &VioTransport::OnImage0, this));
    if (STEREO || DEPTH)
      subscribers_.push_back(node_.subscribe(IMAGE1_TOPIC, 100, &VioTransport::OnImage1, this));
    if (USE_YOLO)
      subscribers_.push_back(node_.subscribe("/darknet_ros/bounding_boxes", 100, &VioTransport::OnBoxes, this));
    if (GNSS_ENABLE) {
      subscribers_.push_back(node_.subscribe(GNSS_EPHEM_TOPIC, 100, &VioTransport::OnEphemeris, this));
      subscribers_.push_back(node_.subscribe(GNSS_GLO_EPHEM_TOPIC, 100, &VioTransport::OnGlonassEphemeris, this));
      subscribers_.push_back(node_.subscribe(GNSS_IONO_PARAMS_TOPIC, 100, &VioTransport::OnIonosphere, this));
      subscribers_.push_back(node_.subscribe(GNSS_MEAS_TOPIC, 100, &VioTransport::OnGnss, this));
      if (GNSS_LOCAL_ONLINE_SYNC) {
        subscribers_.push_back(node_.subscribe(GNSS_TP_INFO_TOPIC, 100, &VioTransport::OnTimePulse, this));
        subscribers_.push_back(node_.subscribe(LOCAL_TRIGGER_INFO_TOPIC, 100, &VioTransport::OnTrigger, this));
      } else {
        estimator_.inputGNSSTimeDiff(GNSS_LOCAL_TIME_DIFF);
        time_valid_ = true;
      }
    }
    timer_ = node_.createWallTimer(ros::WallDuration(0.01), &VioTransport::OnTimer, this);
  }

 private:
  void OnRestart(const std_msgs::BoolConstPtr& message) {
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
  struct PendingImage {
    sensor_msgs::ImageConstPtr message;
    ros::WallTime received;
  };
  void OnImage0(const sensor_msgs::ImageConstPtr& image) {
    if (!images0_.empty() && image->header.stamp <= images0_.back().message->header.stamp) {
      ROS_WARN("Dropping non-increasing primary image timestamp");
      return;
    }
    images0_.push_back({image, ros::WallTime::now()});
    DrainImages();
  }
  void OnImage1(const sensor_msgs::ImageConstPtr& image) {
    if (!images1_.empty() && image->header.stamp <= images1_.back()->header.stamp) {
      ROS_WARN("Dropping non-increasing secondary image timestamp");
      return;
    }
    images1_.push_back(image);
    // Secondary images older than the last consumed primary cannot be paired.
    while (!images1_.empty() && images1_.front()->header.stamp.toSec() < last_image_ - tolerance_)
      images1_.pop_front();
    DrainImages();
  }
  void OnBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& message) {
    if (!boxes_.empty() && message->image_header.stamp <= boxes_.back()->image_header.stamp)
      return;  // Detector can publish the same image more than once.
    boxes_.push_back(message);
    while (!boxes_.empty() && boxes_.front()->image_header.stamp.toSec() < last_image_ - tolerance_)
      boxes_.pop_front();
    DrainImages();
  }
  void OnTimer(const ros::WallTimerEvent&) { DrainImages(); }
  static cv::Mat ToGray(const sensor_msgs::ImageConstPtr& image) {
    if (image->encoding == "8UC1") {
      auto copy = boost::make_shared<sensor_msgs::Image>(*image);
      copy->encoding = sensor_msgs::image_encodings::MONO8;
      return cv_bridge::toCvCopy(copy, sensor_msgs::image_encodings::MONO8)->image;
    }
    return cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8)->image;
  }
  static cv::Mat ToDepth(const sensor_msgs::ImageConstPtr& image) {
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
    while (ros::ok() && !images0_.empty()) {
      const auto& primary = images0_.front();
      const double stamp = primary.message->header.stamp.toSec();
      if (stamp <= last_image_) { images0_.pop_front(); continue; }
      const bool timed_out = (ros::WallTime::now() - primary.received).toSec() > detector_timeout_;
      sensor_msgs::ImageConstPtr second;
      if (STEREO || DEPTH) {
        while (!images1_.empty() && images1_.front()->header.stamp.toSec() < stamp - tolerance_)
          images1_.pop_front();
        if (images1_.empty()) {
          if (!timed_out) return;
          ROS_WARN("Dropping image without synchronized secondary image");
          images0_.pop_front();
          continue;
        }
        if (images1_.front()->header.stamp.toSec() > stamp + tolerance_) {
          images0_.pop_front();
          continue;
        }
        second = images1_.front();
      }
      darknet_ros_msgs::BoundingBoxesConstPtr detection;
      if (USE_YOLO) {
        while (!boxes_.empty() && boxes_.front()->image_header.stamp.toSec() < stamp - tolerance_)
          boxes_.pop_front();
        if (boxes_.empty()) {
          if (!timed_out) return;
          // Never silently run unmasked when semantic filtering is requested.
          ROS_WARN_THROTTLE(2.0, "Dropping image: no synchronized YOLO result; start darknet_ros and check its image topic");
          images0_.pop_front();
          if (second) images1_.pop_front();
          continue;
        }
        if (boxes_.front()->image_header.stamp.toSec() > stamp + tolerance_) {
          images0_.pop_front();
          if (second) images1_.pop_front();
          continue;
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
        ROS_ERROR_STREAM("Rejected image at " << stamp << ": " << error.what());
      }
      last_image_ = stamp;
      images0_.pop_front();
      if (second) images1_.pop_front();
      if (detection) boxes_.pop_front();
    }
  }
  void OnImu(const sensor_msgs::ImuConstPtr& message) {
    const double stamp = message->header.stamp.toSec();
    if (stamp <= last_imu_) { ROS_WARN("Dropping non-increasing IMU timestamp"); return; }
    last_imu_ = stamp;
    if (USE_WHEEL && w_replace) {
      imus_.push_back(message);
      while (imus_.size() > 2 && (stamp - imus_[1]->header.stamp.toSec()) > 2.0) imus_.pop_front();
    }
    if (USE_IMU) estimator_.inputIMU(stamp,
      Vector3d(message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z),
      Vector3d(message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z));
    DrainWheels();
  }
  void OnWheel(const nav_msgs::OdometryConstPtr& message) {
    if (message->header.stamp.toSec() <= last_wheel_) { ROS_WARN("Dropping non-increasing wheel timestamp"); return; }
    last_wheel_ = message->header.stamp.toSec();
    wheels_.push_back(message);
    DrainWheels();
  }
  void DrainWheels() {
    while (!wheels_.empty()) {
      const auto message = wheels_.front();
      const double stamp = message->header.stamp.toSec();
      double yaw_rate = message->twist.twist.angular.z;
      if (w_replace) {
        if (imus_.empty() || imus_.back()->header.stamp.toSec() < stamp) return;
        while (imus_.size() > 1 && imus_[1]->header.stamp.toSec() <= stamp) imus_.pop_front();
        if (imus_.front()->header.stamp.toSec() > stamp) {
          ROS_WARN("Dropping wheel sample without preceding IMU for yaw interpolation");
          wheels_.pop_front();
          continue;
        }
        const auto& before = imus_.front();
        yaw_rate = -before->angular_velocity.y; // Upstream wheel/IMU axis convention.
        if (imus_.size() > 1) {
          const auto& after = imus_[1];
          const double fraction = (stamp - before->header.stamp.toSec()) /
            (after->header.stamp.toSec() - before->header.stamp.toSec());
          yaw_rate += fraction * (-after->angular_velocity.y - yaw_rate);
        }
      }
      estimator_.inputWheel(stamp,
        Vector3d(message->twist.twist.linear.x, message->twist.twist.linear.y, message->twist.twist.linear.z),
        Vector3d(message->twist.twist.angular.x, message->twist.twist.angular.y, yaw_rate));
      const auto& p = message->pose.pose.position;
      const auto& q = message->pose.pose.orientation;
      std_msgs::Header header = message->header;
      header.frame_id = "world";
      pubWheelRawodom(Vector3d(p.x, p.y, p.z), Eigen::Quaterniond(q.w, q.x, q.y, q.z), header);
      estimator_.inputrawodom(stamp, Vector3d(p.x, p.y, 0));
      wheels_.pop_front();
    }
  }
  void OnEphemeris(const gnss_comm::GnssEphemMsgConstPtr& message) { estimator_.inputEphem(gnss_comm::msg2ephem(message)); }
  void OnGlonassEphemeris(const gnss_comm::GnssGloEphemMsgConstPtr& message) { estimator_.inputEphem(gnss_comm::msg2glo_ephem(message)); }
  void OnIonosphere(const gnss_comm::StampedFloat64ArrayConstPtr& message) {
    if (message->data.size() != 8) { ROS_ERROR("GNSS ionosphere message requires 8 coefficients"); return; }
    estimator_.inputIonoParams(message->header.stamp.toSec(), message->data);
  }
  void OnGnss(const gnss_comm::GnssMeasMsgConstPtr& message) {
    if (!time_valid_) { ROS_WARN_THROTTLE(2.0, "Waiting for GNSS/local PPS synchronization"); return; }
    auto observations = gnss_comm::msg2meas(message);
    if (observations.empty()) return;
    const double stamp = gnss_comm::time2sec(observations.front()->time);
    estimator_.inputGNSS(stamp, std::move(observations));
  }
  void OnTimePulse(const gnss_comm::GnssTimePulseInfoMsgConstPtr& message) {
    using namespace gnss_comm;
    const auto time = vins_adapter::ToGpsPulseTime(
        gpst2time(message->time.week, message->time.tow), message->time_sys, message->utc_based);
    if (!time) {
      ROS_ERROR("Unknown GNSS PPS clock scale (expected gnss_comm T_SYS_*)");
      return;
    }
    next_pulse_ = time2sec(*time);
    pulse_received_ = ros::WallTime::now();
    pulse_valid_ = true;
  }
  void OnTrigger(const vins::LocalSensorExternalTriggerConstPtr& message) {
    // Receiver time-pulse info describes the next pulse. Consume it only once.
    if (!pulse_valid_ || (ros::WallTime::now() - pulse_received_).toSec() > 1.0) {
      ROS_WARN("Local trigger has no fresh GNSS time-pulse information");
      return;
    }
    estimator_.inputGNSSTimeDiff(next_pulse_ - message->header.stamp.toSec());
    pulse_valid_ = false;
    time_valid_ = true;
  }
  ros::NodeHandle node_;
  Estimator estimator_;
  std::vector<ros::Subscriber> subscribers_;
  ros::WallTimer timer_;
  std::deque<PendingImage> images0_;
  std::deque<sensor_msgs::ImageConstPtr> images1_;
  std::deque<darknet_ros_msgs::BoundingBoxesConstPtr> boxes_;
  std::deque<sensor_msgs::ImuConstPtr> imus_;
  std::deque<nav_msgs::OdometryConstPtr> wheels_;
  double last_image_ = -1, last_imu_ = -1, last_wheel_ = -1;
  double tolerance_, detector_timeout_, next_pulse_ = 0;
  bool time_valid_ = false, pulse_valid_ = false;
  ros::WallTime pulse_received_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "vins_estimator");
  ros::NodeHandle node("~");
  try {
    std::string config;
    if (!node.getParam("config_file", config)) {
      const auto args = ros::removeROSArgs(argc, argv);
      if (args.size() != 2) throw std::runtime_error("Set ~config_file or pass one VINS YAML configuration path");
      config = args[1];
    }
    readParameters(config);
    VioTransport transport(node);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
