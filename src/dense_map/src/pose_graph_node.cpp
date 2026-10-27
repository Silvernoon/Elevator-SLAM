/*******************************************************
 * Native ROS 2 Foxy RGB-D dense mapping node.
 *******************************************************/
#include "pose_graph.h"
#include "parameters.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <map>
#include <memory>
#include <stdexcept>

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic = Eigen::Vector3d::Zero();
Eigen::Matrix3d qic = Eigen::Matrix3d::Identity();
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
int VISUALIZATION_SHIFT_X = 0, VISUALIZATION_SHIFT_Y = 0;
int ROW = 0, COL = 0, DEBUG_IMAGE = 0;
std::string BRIEF_PATTERN_FILE, POSE_GRAPH_SAVE_PATH, VINS_RESULT_PATH;
std::string MAP_FRAME = "fusion_world";

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void OnSignal(int) { stop_requested = 1; }
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

struct Pose {
  Vector3d p;
  Quaterniond q;
};

Pose DecodePose(const geometry_msgs::msg::Pose &p) {
  Pose out{Vector3d(p.position.x, p.position.y, p.position.z),
           Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)};
  if (!out.p.allFinite() || !out.q.coeffs().allFinite() || out.q.norm() < 1e-8)
    throw std::runtime_error("invalid pose");
  out.q.normalize();
  return out;
}

std::string ResolvePath(const std::string &base, const std::string &path) {
  const boost::filesystem::path file(path);
  return (file.is_absolute() ? file : boost::filesystem::path(base) / file).string();
}

Eigen::Matrix4d ReadRigidMatrix(const cv::Mat &value, const std::string &name) {
  if (value.rows != 4 || value.cols != 4)
    throw std::runtime_error(name + " must be 4x4");
  Eigen::Matrix4d T;
  cv::cv2eigen(value, T);
  if (!T.allFinite() || (T.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-6 ||
      (T.block<3, 3>(0, 0).transpose() * T.block<3, 3>(0, 0) - Matrix3d::Identity()).norm() > 1e-4 ||
      T.block<3, 3>(0, 0).determinant() < .99)
    throw std::runtime_error(name + " must be a rigid transform");
  return T;
}

class DenseMapping {
public:
  explicit DenseMapping(const rclcpp::Node::SharedPtr &node) : node_(node) {
    auto param = [&](const std::string &key, auto default_value) {
      using T = decltype(default_value);
      return node_->declare_parameter<T>(key, default_value);
    };
    const std::string config = param("config_file", std::string());
    if (config.empty()) throw std::runtime_error("config_file is required");
    cv::FileStorage fs(config, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("cannot open config_file: " + config);
    const std::string base = boost::filesystem::path(config).parent_path().string();
    std::string camera_file;
    fs["cam0_calib"] >> camera_file;
    camera_file = param("camera_file", camera_file);
    if (camera_file.empty()) throw std::runtime_error("cam0_calib or camera_file required");
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(ResolvePath(base, camera_file));
    if (!m_camera) throw std::runtime_error("cannot load camera calibration");
    ROW = m_camera->imageHeight();
    COL = m_camera->imageWidth();
    if (ROW <= 0 || COL <= 0) throw std::runtime_error("invalid calibration image dimensions");

    MAP_FRAME = param("frame_id", MAP_FRAME);
    fusion_child_ = param("fusion_child_frame", std::string("fusion_imu"));
    tolerance_ = param("sync_tolerance", .01);
    max_gap_ = param("max_pose_gap", .10);
    buffer_duration_ = param("buffer_duration", 5.);
    buffer_size_ = param("buffer_size", 300);
    depth_stride_ = param("depth_stride", 10);
    boundary_ = param("depth_boundary", 10);
    depth_scale_ = param("depth_scale", .001);
    min_depth_ = param("min_depth", .1);
    max_depth_ = param("max_depth", 7.);
    skip_distance_ = param("skip_distance", 0.);
    skip_frames_ = param("skip_frames", 0);
    DEBUG_IMAGE = param("save_image", 0);
    equalize_ = param("equalize", false);
    depth_enabled_ = param("depth_enabled", true);
    save_on_shutdown_ = param("save_on_shutdown", true);
    if (tolerance_ < 0 || max_gap_ <= 0 || buffer_duration_ <= 0 || buffer_size_ < 2 ||
        depth_stride_ <= 0 || boundary_ < 0 || depth_scale_ <= 0 || min_depth_ <= 0 || max_depth_ <= min_depth_)
      throw std::runtime_error("invalid synchronization or depth parameters");

    cv::Mat native;
    fs["body_T_cam0"] >> native;
    Eigen::Matrix4d Tv = Eigen::Matrix4d::Identity();
    if (!native.empty()) Tv = ReadRigidMatrix(native, "body_T_cam0");
    auto supplied = param("imu_t_camera", std::vector<double>());
    auto bridge = param("imu_lio_T_imu_vio", std::vector<double>());
    Eigen::Matrix4d Tl = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Tlv = Eigen::Matrix4d::Identity();
    if (!supplied.empty()) {
      if (supplied.size() != 16) throw std::runtime_error("imu_t_camera requires 16 values");
      Tl = ReadRigidMatrix(cv::Mat(4, 4, CV_64F, supplied.data()), "imu_t_camera");
    }
    if (!bridge.empty()) {
      if (bridge.size() != 7 || !std::all_of(bridge.begin(), bridge.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("imu_lio_T_imu_vio requires 7 finite values");
      Quaterniond q(bridge[6], bridge[3], bridge[4], bridge[5]);
      if (q.norm() < 1e-8) throw std::runtime_error("invalid imu_lio_T_imu_vio");
      Tlv.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
      Tlv.block<3, 1>(0, 3) = Vector3d(bridge[0], bridge[1], bridge[2]);
    } else {
      if (supplied.empty() || native.empty())
        throw std::runtime_error("imu_lio_T_imu_vio is required unless imu_t_camera and body_T_cam0 are both supplied");
      Tlv = Tl * Tv.inverse();
    }
    bridge_r_ = Tlv.block<3, 3>(0, 0);
    bridge_t_ = Tlv.block<3, 1>(0, 3);
    if (supplied.empty() && !native.empty()) Tl = Tlv * Tv;
    if (!supplied.empty() || !native.empty()) {
      qic = Tl.block<3, 3>(0, 0);
      tic = Tl.block<3, 1>(0, 3);
      static_t_ = tic;
      static_r_ = qic;
      static_extrinsic_ = true;
    }

    std::string image, depth;
    fs["image0_topic"] >> image;
    fs["image1_topic"] >> depth;
    const auto pose_topic = param("pose_topic", std::string("/fusion/odometry"));
    image = param("image_topic", image);
    depth = param("depth_topic", depth);
    const auto keyframe_topic = param("keyframe_pose_topic", std::string("/vins_estimator/keyframe_pose"));
    const auto point_topic = param("keyframe_point_topic", std::string("/vins_estimator/keyframe_point"));
    const auto extrinsic_topic = param("extrinsic_topic", std::string("/vins_estimator/extrinsic"));
    if (image.empty() || (depth_enabled_ && depth.empty())) throw std::runtime_error("image/depth topic required");
    if (!static_extrinsic_ && extrinsic_topic.empty()) throw std::runtime_error("camera extrinsic required");

    const std::string pkg = ament_index_cpp::get_package_share_directory("dense_map");
    const auto vocabulary = param("vocabulary_file", pkg + "/resources/brief_k10L6.bin");
    BRIEF_PATTERN_FILE = param("brief_pattern_file", pkg + "/resources/brief_pattern.yml");
    if (!boost::filesystem::is_regular_file(vocabulary) || !boost::filesystem::is_regular_file(BRIEF_PATTERN_FILE))
      throw std::runtime_error("BRIEF vocabulary/pattern not found");
    const auto output = param("output_path", std::string("dense_map_output"));
    boost::filesystem::create_directories(output);
    POSE_GRAPH_SAVE_PATH = param("pose_graph_save_path", output + "/pose_graph");
    boost::filesystem::create_directories(POSE_GRAPH_SAVE_PATH);
    POSE_GRAPH_SAVE_PATH += "/";
    VINS_RESULT_PATH = output + "/fusion_loop.txt";
    cloud_path_ = param("cloud_save_path", output + "/map.pcd");

    graph_.registerPub(node_);
    pub_match_img = node_->create_publisher<sensor_msgs::msg::Image>("~/match_image", rclcpp::QoS(2).reliable());
    pub_points_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("~/point_cloud_loop_rect", rclcpp::QoS(2).reliable());
    graph_.loadVocabulary(vocabulary);
    graph_.setIMUFlag(param("use_imu", true));
    if (param("load_previous_pose_graph", false)) graph_.loadPoseGraph();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(buffer_size_));
    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(buffer_size_))).reliable();
    fusion_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(pose_topic, odom_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { OnFusion(m); });
    pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(keyframe_topic, odom_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { Insert(poses_, m); Process(); });
    point_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud>(point_topic, sensor_qos,
        [this](sensor_msgs::msg::PointCloud::ConstSharedPtr m) { Insert(points_, m); Process(); });
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(image, sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { Insert(images_, m); Process(); });
    if (depth_enabled_)
      depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(depth, sensor_qos,
          [this](sensor_msgs::msg::Image::ConstSharedPtr m) { Insert(depths_, m); Process(); });
    if (!extrinsic_topic.empty())
      extrinsic_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(extrinsic_topic, odom_qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { OnExtrinsic(m); });
    save_service_ = node_->create_service<std_srvs::srv::Trigger>("~/save_map",
        [this](std_srvs::srv::Trigger::Request::SharedPtr req,
               std_srvs::srv::Trigger::Response::SharedPtr res) { Save(req, res); });
    sequence_service_ = node_->create_service<std_srvs::srv::Trigger>("~/new_sequence",
        [this](std_srvs::srv::Trigger::Request::SharedPtr req,
               std_srvs::srv::Trigger::Response::SharedPtr res) { NewSequence(req, res); });
  }

  ~DenseMapping() {
    graph_.stop();
    if (save_on_shutdown_) {
      try { Persist(); }
      catch (const std::exception &e) { RCLCPP_ERROR(node_->get_logger(), "dense_map save: %s", e.what()); }
    }
  }

private:
  template<class M> using Buffer = std::map<rclcpp::Time, std::shared_ptr<const M>>;

  template<class M>
  void Insert(Buffer<M> &buffer, const std::shared_ptr<const M> &message) {
    buffer[rclcpp::Time(message->header.stamp)] = message;
    const auto latest = buffer.rbegin()->first;
    while (!buffer.empty() && (buffer.size() > static_cast<size_t>(buffer_size_) ||
           (latest - buffer.begin()->first).seconds() > buffer_duration_))
      buffer.erase(buffer.begin());
  }

  template<class M>
  typename Buffer<M>::mapped_type Nearest(const Buffer<M> &buffer, const rclcpp::Time &stamp) const {
    if (buffer.empty()) return {};
    auto hi = buffer.lower_bound(stamp);
    auto best = hi;
    if (hi != buffer.begin()) {
      auto lo = std::prev(hi);
      if (hi == buffer.end() || (stamp - lo->first) < (hi->first - stamp)) best = lo;
    }
    if (best == buffer.end() || std::abs((best->first - stamp).seconds()) > tolerance_) return {};
    return best->second;
  }

  bool FusionAt(const rclcpp::Time &stamp, Pose &out) const {
    auto hi = fusion_.lower_bound(stamp);
    if (hi != fusion_.end() && hi->first == stamp) {
      out = DecodePose(hi->second->pose.pose);
      return true;
    }
    if (hi == fusion_.end() || hi == fusion_.begin()) return false;
    auto lo = std::prev(hi);
    const double gap = (hi->first - lo->first).seconds();
    if (gap > max_gap_) return false;
    const double alpha = (stamp - lo->first).seconds() / gap;
    const auto x = DecodePose(lo->second->pose.pose);
    const auto y = DecodePose(hi->second->pose.pose);
    out.p = (1 - alpha) * x.p + alpha * y.p;
    out.q = x.q.slerp(alpha, y.q).normalized();
    return true;
  }

  bool ExtrinsicAt(const rclcpp::Time &stamp) {
    auto hi = extrinsics_.upper_bound(stamp);
    if (hi != extrinsics_.begin()) {
      auto lo = std::prev(hi);
      if ((stamp - lo->first).seconds() <= buffer_duration_) {
        const auto pose = DecodePose(lo->second->pose.pose);
        tic = bridge_r_ * pose.p + bridge_t_;
        qic = bridge_r_ * pose.q.toRotationMatrix();
        return true;
      }
    }
    if (!static_extrinsic_) return false;
    tic = static_t_;
    qic = static_r_;
    return true;
  }

  void OnFusion(const nav_msgs::msg::Odometry::ConstSharedPtr &message) {
    if (message->header.frame_id != MAP_FRAME || message->child_frame_id != fusion_child_) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "fusion pose frame mismatch");
      return;
    }
    try {
      DecodePose(message->pose.pose);
      Insert(fusion_, message);
      Process();
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "%s", e.what());
    }
  }

  void OnExtrinsic(const nav_msgs::msg::Odometry::ConstSharedPtr &message) {
    try {
      DecodePose(message->pose.pose);
      Insert(extrinsics_, message);
      Process();
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "%s", e.what());
    }
  }

  void Process() {
    for (auto it = poses_.begin(); it != poses_.end();) {
      const auto stamp = it->first;
      if (last_stamp_.nanoseconds() != 0 && stamp <= last_stamp_) {
        it = poses_.erase(it);
        continue;
      }
      const auto image = Nearest(images_, stamp);
      const auto points = Nearest(points_, stamp);
      const auto depth = depth_enabled_ ? Nearest(depths_, stamp) : nullptr;
      Pose fused;
      if (!image || !points || (depth_enabled_ && !depth) || !FusionAt(stamp, fused) || !ExtrinsicAt(stamp)) {
        ++it;
        continue;
      }
      try { AddKeyframe(it->second, image, depth, points, fused); }
      catch (const std::exception &e) { RCLCPP_WARN(node_->get_logger(), "Dropping dense keyframe: %s", e.what()); }
      last_stamp_ = stamp;
      it = poses_.erase(it);
    }
  }

  void AddKeyframe(const nav_msgs::msg::Odometry::ConstSharedPtr &vio,
                   const sensor_msgs::msg::Image::ConstSharedPtr &image,
                   const sensor_msgs::msg::Image::ConstSharedPtr &depth,
                   const sensor_msgs::msg::PointCloud::ConstSharedPtr &points, const Pose &fused) {
    if (skip_count_++ < skip_frames_) return;
    skip_count_ = 0;
    if (have_last_ && (fused.p - last_position_).norm() < skip_distance_) return;
    const Pose original = DecodePose(vio->pose.pose);
    const Matrix3d fr = fused.q.toRotationMatrix();
    const Matrix3d cr = fr * bridge_r_ * original.q.toRotationMatrix().transpose();
    const Vector3d ct = fused.p + fr * bridge_t_ - cr * original.p;
    std::vector<cv::Point3f> landmarks;
    std::vector<cv::Point2f> pixels, normalized;
    std::vector<double> ids;
    if (points->channels.size() != points->points.size())
      throw std::runtime_error("VIO point/channel count mismatch");
    for (size_t i = 0; i < points->points.size(); ++i) {
      const auto &p = points->points[i];
      const auto &c = points->channels[i].values;
      if (c.size() < 5) throw std::runtime_error("VIO feature channel requires nx,ny,u,v,id");
      const Vector3d transformed = cr * Vector3d(p.x, p.y, p.z) + ct;
      if (!transformed.allFinite()) continue;
      landmarks.emplace_back(transformed.x(), transformed.y(), transformed.z());
      normalized.emplace_back(c[0], c[1]);
      pixels.emplace_back(c[2], c[3]);
      ids.push_back(c[4]);
    }

    auto source = image;
    if (image->encoding == "8UC1") {
      auto relabeled = std::make_shared<sensor_msgs::msg::Image>(*image);
      relabeled->encoding = "mono8";
      source = relabeled;
    }
    cv::Mat gray = cv_bridge::toCvCopy(source, "mono8")->image;
    if (gray.rows != ROW || gray.cols != COL)
      throw std::runtime_error("image dimensions differ from calibration");
    if (equalize_) cv::createCLAHE()->apply(gray, gray);
    std::vector<Eigen::Matrix<float, 6, 1>> rgbd;
    if (depth_enabled_) {
      const auto color = cv_bridge::toCvShare(source, "rgb8");
      const auto d = cv_bridge::toCvShare(depth);
      if (d->image.size() != gray.size() || (d->image.type() != CV_16UC1 && d->image.type() != CV_32FC1))
        throw std::runtime_error("aligned depth dimensions/type mismatch");
      for (int v = boundary_; v < ROW - boundary_; v += depth_stride_) {
        for (int u = boundary_; u < COL - boundary_; u += depth_stride_) {
          const float z = d->image.type() == CV_16UC1 ? d->image.at<uint16_t>(v, u) * depth_scale_ : d->image.at<float>(v, u);
          if (!std::isfinite(z) || z < min_depth_ || z > max_depth_) continue;
          Vector3d ray;
          m_camera->liftProjective(Eigen::Vector2d(u, v), ray);
          if (std::abs(ray.z()) < 1e-8) continue;
          const auto rgb = color->image.at<cv::Vec3b>(v, u);
          Eigen::Matrix<float, 6, 1> p;
          p << ray.x() / ray.z() * z, ray.y() / ray.z() * z, z, rgb[0], rgb[1], rgb[2];
          rgbd.push_back(p);
        }
      }
    }
    Vector3d T = fused.p;
    Matrix3d R = fused.q.toRotationMatrix();
    auto *keyframe = new KeyFrame(rclcpp::Time(vio->header.stamp).seconds(), frame_index_, T, R,
                                  gray, rgbd, landmarks, pixels, normalized, ids, sequence_);
    graph_.addKeyFrame(keyframe, true);
    sensor_msgs::msg::PointCloud corrected;
    corrected.header = vio->header;
    corrected.header.frame_id = MAP_FRAME;
    for (const auto &p : landmarks) {
      geometry_msgs::msg::Point32 q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      corrected.points.push_back(q);
    }
    graph_.rectifyCloud(corrected);
    pub_points_->publish(corrected);
    ++frame_index_;
    have_last_ = true;
    last_position_ = T;
  }

  void Persist() {
    graph_.savePoseGraph();
    graph_.saveCloud(cloud_path_);
  }

  void Save(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    try {
      Persist();
      response->success = true;
      response->message = POSE_GRAPH_SAVE_PATH;
    } catch (const std::exception &e) {
      response->success = false;
      response->message = e.what();
    }
  }

  void NewSequence(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!have_last_) {
      response->message = "active sequence has no keyframes";
      return;
    }
    if (sequence_ >= 9) {
      response->message = "maximum 9 sequences";
      return;
    }
    ++sequence_;
    have_last_ = false;
    last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    poses_.clear();
    images_.clear();
    depths_.clear();
    points_.clear();
    fusion_.clear();
    extrinsics_.clear();
    response->success = true;
    response->message = "new sequence";
  }

  rclcpp::Node::SharedPtr node_;
  PoseGraph graph_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fusion_sub_, pose_sub_, extrinsic_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_, depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_points_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_, sequence_service_;
  Buffer<nav_msgs::msg::Odometry> fusion_, poses_, extrinsics_;
  Buffer<sensor_msgs::msg::Image> images_, depths_;
  Buffer<sensor_msgs::msg::PointCloud> points_;
  double tolerance_, max_gap_, buffer_duration_, depth_scale_, min_depth_, max_depth_, skip_distance_;
  int buffer_size_, depth_stride_, boundary_, skip_frames_;
  int skip_count_ = 0, frame_index_ = 0, sequence_ = 1;
  bool equalize_, depth_enabled_, save_on_shutdown_;
  bool static_extrinsic_ = false, have_last_ = false;
  Vector3d static_t_ = tic, last_position_ = Vector3d::Zero(), bridge_t_ = Vector3d::Zero();
  Matrix3d static_r_ = qic, bridge_r_ = Matrix3d::Identity();
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::string cloud_path_, fusion_child_;
};
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  try {
    auto node = std::make_shared<rclcpp::Node>("dense_map");
    DenseMapping mapping(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) executor.spin_once(std::chrono::milliseconds(100));
  } catch (const std::exception &e) {
    fprintf(stderr, "dense_map: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  pub_match_img.reset();
  rclcpp::shutdown();
  return 0;
}
