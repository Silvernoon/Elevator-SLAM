/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * This file is part of VINS, licensed under the GNU General Public License v3.0.
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "pose_graph.h"
#include "parameters.h"
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic = Eigen::Vector3d::Zero();
Eigen::Matrix3d qic = Eigen::Matrix3d::Identity();
ros::Publisher pub_match_img;
int VISUALIZATION_SHIFT_X = 0, VISUALIZATION_SHIFT_Y = 0;
int ROW = 0, COL = 0, DEBUG_IMAGE = 0;
std::string BRIEF_PATTERN_FILE, POSE_GRAPH_SAVE_PATH, VINS_RESULT_PATH;
std::string MAP_FRAME = "fusion_world";

namespace {
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
struct Pose {
    Vector3d p;
    Quaterniond q;
};
Pose DecodePose(const geometry_msgs::Pose &p) {
    Pose out{Vector3d(p.position.x, p.position.y, p.position.z),
             Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)};
    if (!out.p.allFinite() || !out.q.coeffs().allFinite() || out.q.norm() < 1e-8)
        throw std::runtime_error("invalid pose");
    out.q.normalize();
    return out;
}
std::string ResolvePath(const std::string &base, const std::string &path) {
    return (boost::filesystem::path(path).is_absolute() ? boost::filesystem::path(path)
            : boost::filesystem::path(base) / path).string();
}
Eigen::Matrix4d ReadRigidMatrix(const cv::Mat &value, const std::string &name) {
    if (value.rows != 4 || value.cols != 4) throw std::runtime_error(name + " must be 4x4");
    Eigen::Matrix4d T;
    cv::cv2eigen(value, T);
    if (!T.allFinite() || (T.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-6 ||
        (T.block<3,3>(0,0).transpose() * T.block<3,3>(0,0) - Matrix3d::Identity()).norm() > 1e-4 ||
        T.block<3,3>(0,0).determinant() < 0.99)
        throw std::runtime_error(name + " must be a rigid transform");
    return T;
}

class DenseMapping {
public:
    explicit DenseMapping(ros::NodeHandle n) : n_(n) {
        std::string config;
        if (!n_.getParam("config_file", config) || config.empty())
            throw std::runtime_error("~config_file must name the VIO camera/configuration YAML");
        cv::FileStorage fs(config, cv::FileStorage::READ);
        if (!fs.isOpened()) throw std::runtime_error("cannot open config_file: " + config);
        const std::string base = boost::filesystem::path(config).parent_path().string();
        std::string camera_file;
        fs["cam0_calib"] >> camera_file;
        n_.param("camera_file", camera_file, camera_file);
        if (camera_file.empty()) throw std::runtime_error("cam0_calib or ~camera_file required");
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(ResolvePath(base, camera_file));
        if (!m_camera) throw std::runtime_error("cannot load camera calibration");
        ROW = m_camera->imageHeight();
        COL = m_camera->imageWidth();
        if (ROW <= 0 || COL <= 0) throw std::runtime_error("invalid calibration image dimensions");
        n_.param("frame_id", MAP_FRAME, MAP_FRAME);
        n_.param("fusion_child_frame", fusion_child_, std::string("fusion_imu"));
        n_.param("sync_tolerance", tolerance_, 0.01);
        n_.param("max_pose_gap", max_gap_, 0.10);
        n_.param("buffer_duration", buffer_duration_, 5.0);
        n_.param("buffer_size", buffer_size_, 300);
        n_.param("depth_stride", depth_stride_, 10);
        n_.param("depth_boundary", boundary_, 10);
        n_.param("depth_scale", depth_scale_, 0.001);
        n_.param("min_depth", min_depth_, 0.1);
        n_.param("max_depth", max_depth_, 7.0);
        n_.param("skip_distance", skip_distance_, 0.0);
        n_.param("skip_frames", skip_frames_, 0);
        n_.param("save_image", DEBUG_IMAGE, 0);
        n_.param("equalize", equalize_, false);
        n_.param("depth_enabled", depth_enabled_, true);
        n_.param("save_on_shutdown", save_on_shutdown_, true);
        if (tolerance_ < 0 || max_gap_ <= 0 || buffer_duration_ <= 0 || buffer_size_ < 2 ||
            depth_stride_ <= 0 || boundary_ < 0 || depth_scale_ <= 0 || min_depth_ <= 0 || max_depth_ <= min_depth_)
            throw std::runtime_error("invalid synchronization or depth parameters");

        cv::Mat native_camera;
        fs["body_T_cam0"] >> native_camera;
        Eigen::Matrix4d T_vio_camera = Eigen::Matrix4d::Identity();
        if (!native_camera.empty()) T_vio_camera = ReadRigidMatrix(native_camera, "body_T_cam0");
        std::vector<double> supplied, bridge;
        n_.getParam("imu_t_camera", supplied);
        n_.getParam("imu_lio_T_imu_vio", bridge);
        Eigen::Matrix4d T_lio_camera = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_lio_vio = Eigen::Matrix4d::Identity();
        if (!supplied.empty()) {
            if (supplied.size() != 16) throw std::runtime_error("imu_t_camera requires 16 row-major LIO_IMU_T_camera values");
            T_lio_camera = ReadRigidMatrix(cv::Mat(4, 4, CV_64F, supplied.data()), "imu_t_camera");
        }
        if (!bridge.empty()) {
            if (bridge.size() != 7) throw std::runtime_error("imu_lio_T_imu_vio requires [x,y,z,qx,qy,qz,qw]");
            Quaterniond rotation(bridge[6], bridge[3], bridge[4], bridge[5]);
            Vector3d translation(bridge[0], bridge[1], bridge[2]);
            if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-8 || !translation.allFinite())
                throw std::runtime_error("invalid imu_lio_T_imu_vio");
            T_lio_vio.block<3,3>(0,0) = rotation.normalized().toRotationMatrix();
            T_lio_vio.block<3,1>(0,3) = translation;
        } else if (!supplied.empty() && !native_camera.empty()) {
            T_lio_vio = T_lio_camera * T_vio_camera.inverse();
        } else {
            throw std::runtime_error("two IMUs require imu_lio_T_imu_vio, or imu_t_camera AND native body_T_cam0");
        }
        bridge_r_ = T_lio_vio.block<3,3>(0,0);
        bridge_t_ = T_lio_vio.block<3,1>(0,3);
        if (supplied.empty() && !native_camera.empty()) T_lio_camera = T_lio_vio * T_vio_camera;
        if (!supplied.empty() || !native_camera.empty()) {
            qic = T_lio_camera.block<3,3>(0,0);
            tic = T_lio_camera.block<3,1>(0,3);
            static_t_ = tic; static_r_ = qic;
            static_extrinsic_ = true;
        }
        std::string pose_topic, image_topic, depth_topic, keyframe_topic, point_topic, extrinsic_topic;
        fs["image0_topic"] >> image_topic;
        fs["image1_topic"] >> depth_topic;
        n_.param("pose_topic", pose_topic, std::string("/fusion/odometry"));
        n_.param("image_topic", image_topic, image_topic);
        n_.param("depth_topic", depth_topic, depth_topic);
        n_.param("keyframe_pose_topic", keyframe_topic, std::string("/vins_estimator/keyframe_pose"));
        n_.param("keyframe_point_topic", point_topic, std::string("/vins_estimator/keyframe_point"));
        n_.param("extrinsic_topic", extrinsic_topic, std::string("/vins_estimator/extrinsic"));
        if (image_topic.empty() || (depth_enabled_ && depth_topic.empty()))
            throw std::runtime_error("image_topic and (when enabled) depth_topic required");
        if (!static_extrinsic_ && extrinsic_topic.empty())
            throw std::runtime_error("camera extrinsic required from config, parameter or topic");

        const std::string pkg = ros::package::getPath("dense_map");
        std::string vocabulary;
        n_.param("vocabulary_file", vocabulary, pkg + "/resources/brief_k10L6.bin");
        n_.param("brief_pattern_file", BRIEF_PATTERN_FILE, pkg + "/resources/brief_pattern.yml");
        if (!boost::filesystem::is_regular_file(vocabulary) || !boost::filesystem::is_regular_file(BRIEF_PATTERN_FILE))
            throw std::runtime_error("BRIEF vocabulary/pattern not found");
        std::string output;
        n_.param("output_path", output, std::string("dense_map_output"));
        boost::filesystem::create_directories(output);
        n_.param("pose_graph_save_path", POSE_GRAPH_SAVE_PATH, output + "/pose_graph");
        boost::filesystem::create_directories(POSE_GRAPH_SAVE_PATH);
        POSE_GRAPH_SAVE_PATH += "/";
        VINS_RESULT_PATH = output + "/fusion_loop.txt";
        cloud_path_ = output + "/map.pcd";
        n_.param("cloud_save_path", cloud_path_, cloud_path_);
        graph_.registerPub(n_);
        pub_match_img = n_.advertise<sensor_msgs::Image>("match_image", 2);
        pub_points_ = n_.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 2);
        graph_.loadVocabulary(vocabulary);
        bool load = false, use_imu = true;
        n_.param("load_previous_pose_graph", load, false);
        n_.param("use_imu", use_imu, true);
        if (load) graph_.loadPoseGraph();
        graph_.setIMUFlag(use_imu);
        fusion_sub_ = n_.subscribe<nav_msgs::Odometry>(pose_topic, buffer_size_, &DenseMapping::OnFusion, this);
        pose_sub_ = n_.subscribe<nav_msgs::Odometry>(keyframe_topic, buffer_size_, &DenseMapping::OnKeyframe, this);
        point_sub_ = n_.subscribe<sensor_msgs::PointCloud>(point_topic, buffer_size_, &DenseMapping::OnPoints, this);
        image_sub_ = n_.subscribe<sensor_msgs::Image>(image_topic, buffer_size_, &DenseMapping::OnImage, this);
        if (depth_enabled_) depth_sub_ = n_.subscribe<sensor_msgs::Image>(depth_topic, buffer_size_, &DenseMapping::OnDepth, this);
        if (!extrinsic_topic.empty()) extrinsic_sub_ = n_.subscribe<nav_msgs::Odometry>(extrinsic_topic, buffer_size_, &DenseMapping::OnExtrinsic, this);
        save_service_ = n_.advertiseService("save_map", &DenseMapping::Save, this);
        sequence_service_ = n_.advertiseService("new_sequence", &DenseMapping::NewSequence, this);
    }
    ~DenseMapping() {
        graph_.stop();
        if (save_on_shutdown_) {
            try { Persist(); } catch (const std::exception &e) { ROS_ERROR_STREAM("dense_map save: " << e.what()); }
        }
    }
private:
    template<class M> using Buffer = std::map<ros::Time, boost::shared_ptr<const M>>;
    template<class M> void Insert(Buffer<M> &buffer, const boost::shared_ptr<const M> &msg) {
        buffer[msg->header.stamp] = msg;
        const ros::Time latest = buffer.rbegin()->first;
        while (!buffer.empty() && (buffer.size() > static_cast<size_t>(buffer_size_) ||
               (latest - buffer.begin()->first).toSec() > buffer_duration_)) buffer.erase(buffer.begin());
    }
    template<class M> typename Buffer<M>::mapped_type Nearest(const Buffer<M> &buffer, const ros::Time &stamp) const {
        if (buffer.empty()) return {};
        auto hi = buffer.lower_bound(stamp), best = hi;
        if (hi != buffer.begin()) {
            auto lo = std::prev(hi);
            if (hi == buffer.end() || (stamp - lo->first) < (hi->first - stamp)) best = lo;
        }
        if (best == buffer.end() || std::abs((best->first - stamp).toSec()) > tolerance_) return {};
        return best->second;
    }
    bool FusionAt(const ros::Time &stamp, Pose &pose) const {
        auto hi = fusion_.lower_bound(stamp);
        if (hi != fusion_.end() && hi->first == stamp) { pose = DecodePose(hi->second->pose.pose); return true; }
        // No extrapolation: a delayed keyframe waits for a bounded bracketing pair.
        if (hi == fusion_.end() || hi == fusion_.begin()) return false;
        auto lo = std::prev(hi);
        const double gap = (hi->first - lo->first).toSec();
        if (gap > max_gap_) return false;
        const double alpha = (stamp - lo->first).toSec() / gap;
        Pose a = DecodePose(lo->second->pose.pose), b = DecodePose(hi->second->pose.pose);
        pose.p = (1 - alpha) * a.p + alpha * b.p;
        pose.q = a.q.slerp(alpha, b.q).normalized();
        return true;
    }
    bool ExtrinsicAt(const ros::Time &stamp) {
        // VIO publishes the estimate with the newest window timestamp, whereas the
        // marginalized keyframe is older. Select by measurement time, never arrival order.
        auto hi = extrinsics_.upper_bound(stamp);
        if (hi != extrinsics_.begin()) {
            auto lo = std::prev(hi);
            if ((stamp - lo->first).toSec() <= buffer_duration_) {
                Pose p = DecodePose(lo->second->pose.pose);
                tic = bridge_r_ * p.p + bridge_t_;
                qic = bridge_r_ * p.q.toRotationMatrix();
                return true;
            }
        }
        if (static_extrinsic_) { tic = static_t_; qic = static_r_; return true; }
        return false;
    }
    void OnFusion(const nav_msgs::OdometryConstPtr &msg) {
        if (msg->header.frame_id != MAP_FRAME || msg->child_frame_id != fusion_child_) {
            ROS_ERROR_THROTTLE(5, "fusion pose frame mismatch; expected configured world/physical IMU frames"); return;
        }
        try { DecodePose(msg->pose.pose); Insert(fusion_, msg); Process(); }
        catch (const std::exception &e) { ROS_WARN_STREAM_THROTTLE(5, e.what()); }
    }
    void OnKeyframe(const nav_msgs::OdometryConstPtr &msg) { Insert(poses_, msg); Process(); }
    void OnPoints(const sensor_msgs::PointCloudConstPtr &msg) { Insert(points_, msg); Process(); }
    void OnImage(const sensor_msgs::ImageConstPtr &msg) { Insert(images_, msg); Process(); }
    void OnDepth(const sensor_msgs::ImageConstPtr &msg) { Insert(depths_, msg); Process(); }
    void OnExtrinsic(const nav_msgs::OdometryConstPtr &msg) {
        try { DecodePose(msg->pose.pose); Insert(extrinsics_, msg); Process(); }
        catch (const std::exception &e) { ROS_WARN_STREAM_THROTTLE(5, e.what()); }
    }
    void Process() {
        for (auto it = poses_.begin(); it != poses_.end();) {
            const ros::Time stamp = it->first;
            if (!last_stamp_.isZero() && stamp <= last_stamp_) { it = poses_.erase(it); continue; }
            auto image = Nearest(images_, stamp);
            auto points = Nearest(points_, stamp);
            auto depth = depth_enabled_ ? Nearest(depths_, stamp) : sensor_msgs::ImageConstPtr();
            Pose fused;
            if (!image || !points || (depth_enabled_ && !depth) || !FusionAt(stamp, fused) || !ExtrinsicAt(stamp)) {
                // Bound all pending queues; an unmatchable old keyframe cannot block later ones.
                ++it; continue;
            }
            try { AddKeyframe(it->second, image, depth, points, fused); }
            catch (const std::exception &e) { ROS_WARN_STREAM("Dropping dense keyframe: " << e.what()); }
            last_stamp_ = stamp;
            it = poses_.erase(it);
        }
    }
    void AddKeyframe(const nav_msgs::OdometryConstPtr &vio, const sensor_msgs::ImageConstPtr &image,
                     const sensor_msgs::ImageConstPtr &depth, const sensor_msgs::PointCloudConstPtr &points, const Pose &fused) {
        if (skip_count_++ < skip_frames_) return;
        skip_count_ = 0;
        if (have_last_ && (fused.p - last_position_).norm() < skip_distance_) return;
        Pose original = DecodePose(vio->pose.pose);
        // Graph body axes are the physical LIO IMU. Landmarks remain consistent
        // with their original VIO camera observations under this single rigid correction.
        const Matrix3d fused_r = fused.q.toRotationMatrix();
        const Matrix3d correction_r = fused_r * bridge_r_ * original.q.toRotationMatrix().transpose();
        const Vector3d correction_t = fused.p + fused_r * bridge_t_ - correction_r * original.p;
        std::vector<cv::Point3f> landmarks;
        std::vector<cv::Point2f> pixels, normalized;
        std::vector<double> ids;
        if (points->channels.size() != points->points.size()) throw std::runtime_error("VIO point/channel count mismatch");
        for (size_t i = 0; i < points->points.size(); ++i) {
            const auto &p = points->points[i];
            const auto &c = points->channels[i].values;
            if (c.size() < 5) throw std::runtime_error("VIO feature channel requires nx,ny,u,v,id");
            const Vector3d transformed = correction_r * Vector3d(p.x, p.y, p.z) + correction_t;
            if (!transformed.allFinite()) continue;
            landmarks.emplace_back(transformed.x(), transformed.y(), transformed.z());
            normalized.emplace_back(c[0], c[1]); pixels.emplace_back(c[2], c[3]); ids.push_back(c[4]);
        }
        sensor_msgs::ImageConstPtr mono_source = image;
        sensor_msgs::ImagePtr relabeled;
        if (image->encoding == "8UC1") { relabeled.reset(new sensor_msgs::Image(*image)); relabeled->encoding = "mono8"; mono_source = relabeled; }
        cv::Mat gray = cv_bridge::toCvCopy(mono_source, "mono8")->image;
        if (gray.rows != ROW || gray.cols != COL) throw std::runtime_error("image dimensions differ from calibration");
        if (equalize_) cv::createCLAHE()->apply(gray, gray);
        std::vector<Eigen::Matrix<float,6,1>> rgbd;
        if (depth_enabled_) {
            auto color = cv_bridge::toCvShare(mono_source, "rgb8");
            auto d = cv_bridge::toCvShare(depth);
            if (d->image.size() != gray.size() || (d->image.type() != CV_16UC1 && d->image.type() != CV_32FC1))
                throw std::runtime_error("aligned depth must match RGB dimensions and be 16UC1 or 32FC1");
            for (int v = boundary_; v < ROW - boundary_; v += depth_stride_) {
                for (int u = boundary_; u < COL - boundary_; u += depth_stride_) {
                    const float z = d->image.type() == CV_16UC1 ? d->image.at<uint16_t>(v,u) * depth_scale_ : d->image.at<float>(v,u);
                    if (!std::isfinite(z) || z < min_depth_ || z > max_depth_) continue;
                    Vector3d ray;
                    m_camera->liftProjective(Eigen::Vector2d(u,v), ray);
                    if (std::abs(ray.z()) < 1e-8) continue;
                    const auto rgb = color->image.at<cv::Vec3b>(v,u);
                    Eigen::Matrix<float,6,1> p;
                    p << ray.x()/ray.z()*z, ray.y()/ray.z()*z, z, rgb[0], rgb[1], rgb[2];
                    rgbd.push_back(p);
                }
            }
        }
        Vector3d T = fused.p;
        Matrix3d R = fused.q.toRotationMatrix();
        auto keyframe = std::unique_ptr<KeyFrame>(new KeyFrame(vio->header.stamp.toSec(), frame_index_, T, R,
            gray, rgbd, landmarks, pixels, normalized, ids, sequence_));
        graph_.addKeyFrame(keyframe.release(), true);
        sensor_msgs::PointCloud corrected;
        corrected.header = vio->header; corrected.header.frame_id = MAP_FRAME;
        for (const auto &p : landmarks) { geometry_msgs::Point32 out; out.x = p.x; out.y = p.y; out.z = p.z; corrected.points.push_back(out); }
        graph_.rectifyCloud(corrected);
        pub_points_.publish(corrected);
        ++frame_index_; have_last_ = true; last_position_ = T;
    }
    void Persist() { graph_.savePoseGraph(); graph_.saveCloud(cloud_path_); }
    bool Save(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response) {
        try { Persist(); response.success = true; response.message = POSE_GRAPH_SAVE_PATH; }
        catch (const std::exception &e) { response.success = false; response.message = e.what(); }
        return true;
    }
    bool NewSequence(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response) {
        if (!have_last_) { response.success = false; response.message = "active sequence has no keyframes"; return true; }
        if (sequence_ >= 9) { response.success = false; response.message = "maximum 9 sequences"; return true; }
        ++sequence_; have_last_ = false; last_stamp_ = ros::Time();
        poses_.clear(); images_.clear(); depths_.clear(); points_.clear(); fusion_.clear(); extrinsics_.clear();
        response.success = true; response.message = "new sequence"; return true;
    }
    ros::NodeHandle n_;
    PoseGraph graph_;
    ros::Subscriber fusion_sub_, pose_sub_, point_sub_, image_sub_, depth_sub_, extrinsic_sub_;
    ros::Publisher pub_points_;
    ros::ServiceServer save_service_, sequence_service_;
    Buffer<nav_msgs::Odometry> fusion_, poses_, extrinsics_;
    Buffer<sensor_msgs::Image> images_, depths_;
    Buffer<sensor_msgs::PointCloud> points_;
    double tolerance_, max_gap_, buffer_duration_, depth_scale_, min_depth_, max_depth_, skip_distance_;
    int buffer_size_, depth_stride_, boundary_, skip_frames_, skip_count_ = 0, frame_index_ = 0, sequence_ = 1;
    bool equalize_, depth_enabled_, save_on_shutdown_, static_extrinsic_ = false, have_last_ = false;
    Vector3d static_t_ = tic, last_position_ = Vector3d::Zero();
    Matrix3d static_r_ = qic;
    Vector3d bridge_t_ = Vector3d::Zero();
    Matrix3d bridge_r_ = Matrix3d::Identity();
    ros::Time last_stamp_;
    std::string cloud_path_, fusion_child_;
};
} // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "dense_map");
    try {
        DenseMapping mapping(ros::NodeHandle("~"));
        // Single callback thread keeps all timestamp queues and original VIO extrinsics coherent.
        ros::spin();
    } catch (const std::exception &e) { ROS_FATAL_STREAM("dense_map: " << e.what()); return 1; }
    return 0;
}
