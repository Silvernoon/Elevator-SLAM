#include "mapping_runtime.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <opencv2/calib3d.hpp>

namespace {
struct Pose {
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::Isometry3d ToTransform() const {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = q.toRotationMatrix();
        result.translation() = t;
        return result;
    }
};

class ExternalMapping {
public:
    ExternalMapping() : private_nh_("~") {
        private_nh_.param<std::string>("world_frame", world_frame_, "fusion_world");
        private_nh_.param<std::string>("imu_frame", imu_frame_, "fusion_imu");
        private_nh_.param("color_enabled", color_enabled_, true);
        private_nh_.param("image_tolerance", image_tolerance_, 0.05);
        private_nh_.param("maximum_pose_gap", maximum_pose_gap_, 0.2);
        private_nh_.param("queue_size", queue_size_, 200);
        private_nh_.param("minimum_point_distance", minimum_pts, 0.1);
        private_nh_.param("meshing_voxel_size", g_meshing_voxel_size, 0.4);
        private_nh_.param("points_per_scan", appending_pts_frame, 5000);
        private_nh_.param<std::string>("output_mesh", output_mesh_, "/tmp/fusion_mesh.ply");
        if (queue_size_ < 2 || image_tolerance_ < 0 || maximum_pose_gap_ <= 0 ||
            minimum_pts <= 0 || g_meshing_voxel_size <= 0 || appending_pts_frame < 1)
            throw std::runtime_error("Invalid mapping synchronization or geometry parameters");
        g_map_rgb_pts_mesh.set_minimum_dis(minimum_pts);
        g_map_rgb_pts_mesh.set_voxel_resolution(g_meshing_voxel_size);
        std::vector<double> camera_k, extrinsic;
        if (color_enabled_) {
            if (!private_nh_.getParam("camera_k", camera_k) || camera_k.size() != 9 ||
                !private_nh_.getParam("imu_t_camera", extrinsic) || extrinsic.size() != 16)
                throw std::runtime_error("Color mapping needs camera_k[9] and row-major imu_t_camera[16]");
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) g_camera_K(r,c) = camera_k[r*3+c];
            Eigen::Matrix4d transform;
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c) transform(r,c) = extrinsic[r*4+c];
            if (!transform.allFinite() || !g_camera_K.allFinite() ||
                g_camera_K(0,0) <= 0 || g_camera_K(1,1) <= 0 ||
                (transform.row(3) - Eigen::RowVector4d(0,0,0,1)).norm() > 1e-6 ||
                (transform.block<3,3>(0,0).transpose()*transform.block<3,3>(0,0) - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
                std::abs(transform.block<3,3>(0,0).determinant() - 1.0) > 1e-5)
                throw std::runtime_error("Camera calibration is not a finite pinhole calibration / rigid T_imu_camera");
            imu_t_camera_.matrix() = transform;
            private_nh_.getParam("camera_distortion", distortion_);
        }
        std::string cloud_topic, odom_topic, image_topic;
        private_nh_.param<std::string>("cloud_topic", cloud_topic, "/fusion/cloud_world");
        private_nh_.param<std::string>("odom_topic", odom_topic, "/fusion/odometry");
        private_nh_.param<std::string>("image_topic", image_topic, "/camera/color/image_raw");
        cloud_sub_ = nh_.subscribe(cloud_topic, queue_size_, &ExternalMapping::OnCloud, this);
        pose_sub_ = nh_.subscribe(odom_topic, queue_size_, &ExternalMapping::OnOdometry, this);
        if (color_enabled_) image_sub_ = nh_.subscribe(image_topic, queue_size_, &ExternalMapping::OnImage, this);
        cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
        mesh_pub_ = private_nh_.advertise<visualization_msgs::Marker>("mesh", 1, true);
        save_service_ = private_nh_.advertiseService("save_mesh", &ExternalMapping::Save, this);
    }

private:
    template<class T> void Bound(std::map<double,T>& cache) {
        while (cache.size() > static_cast<std::size_t>(queue_size_)) cache.erase(cache.begin());
    }
    bool PoseAt(double stamp, Pose& result) const {
        auto upper = poses_.lower_bound(stamp);
        if (upper != poses_.end() && std::abs(upper->first - stamp) < 1e-9) {
            result = upper->second;
            return true;
        }
        if (upper == poses_.begin() || upper == poses_.end()) return false;
        const auto lower = std::prev(upper);
        const double gap = upper->first - lower->first;
        if (gap > maximum_pose_gap_) return false;
        const double alpha = (stamp - lower->first) / gap;
        result.q = lower->second.q.slerp(alpha, upper->second.q).normalized();
        result.t = (1-alpha) * lower->second.t + alpha * upper->second.t;
        return true;
    }
    void OnOdometry(const nav_msgs::OdometryConstPtr& msg) {
        if (msg->header.frame_id != world_frame_ || msg->child_frame_id != imu_frame_) {
            ROS_ERROR_THROTTLE(2, "Mesh odometry must be world_frame -> imu_frame");
            return;
        }
        const auto& p = msg->pose.pose;
        Pose pose{Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z),
                  Eigen::Vector3d(p.position.x,p.position.y,p.position.z)};
        if (!pose.q.coeffs().allFinite() || pose.q.norm() < 1e-9 || !pose.t.allFinite()) return;
        pose.q.normalize();
        poses_[msg->header.stamp.toSec()] = pose;
        Bound(poses_);
        Process();
    }
    void OnCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if (msg->header.frame_id != world_frame_) {
            ROS_ERROR_THROTTLE(2, "Mesh input cloud must already be in world_frame");
            return;
        }
        clouds_[msg->header.stamp.toSec()] = msg;
        Bound(clouds_);
        Process();
    }
    void OnImage(const sensor_msgs::ImageConstPtr& msg) {
        images_[msg->header.stamp.toSec()] = msg;
        Bound(images_);
        Process();
    }
    void Process() {
        while (!clouds_.empty() && !poses_.empty()) {
            auto scan = clouds_.begin();
            const double stamp = scan->first;
            if (stamp <= last_scan_stamp_) { clouds_.erase(scan); continue; }
            Pose imu_pose;
            if (!PoseAt(stamp, imu_pose)) {
                if (poses_.begin()->first > stamp || poses_.rbegin()->first > stamp + maximum_pose_gap_) {
                    ROS_WARN_THROTTLE(2, "Dropping mesh scan without timestamp-bracketed IMU pose");
                    clouds_.erase(scan);
                    continue;
                }
                return;
            }
            cv::Mat bgr;
            Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
            double image_stamp = stamp;
            if (color_enabled_) {
                if (images_.empty()) return;
                auto selected = images_.lower_bound(stamp);
                if (selected != images_.begin()) {
                    auto before = std::prev(selected);
                    if (selected == images_.end() || stamp-before->first <= selected->first-stamp) selected = before;
                }
                if (selected == images_.end() || std::abs(selected->first-stamp) > image_tolerance_) {
                    if (images_.rbegin()->first > stamp + image_tolerance_) {
                        ROS_WARN_THROTTLE(2, "Dropping mesh scan without time-matched camera image");
                        clouds_.erase(scan);
                        continue;
                    }
                    return;
                }
                Pose image_imu;
                image_stamp = selected->first;
                if (!PoseAt(image_stamp, image_imu)) {
                    if (poses_.begin()->first > image_stamp || poses_.rbegin()->first > image_stamp + maximum_pose_gap_) {
                        clouds_.erase(scan);
                        continue;
                    }
                    return;
                }
                camera_pose = image_imu.ToTransform() * imu_t_camera_;
                try {
                    const auto converted = cv_bridge::toCvShare(selected->second, "bgr8");
                    if (distortion_.empty()) bgr = converted->image.clone();
                    else {
                        cv::Mat intrinsics;
                        cv::eigen2cv(g_camera_K, intrinsics);
                        cv::undistort(converted->image, bgr, intrinsics, distortion_);
                    }
                } catch (const cv::Exception& error) {
                    ROS_ERROR("Mesh image conversion: %s", error.what());
                    clouds_.erase(scan);
                    continue;
                } catch (const cv_bridge::Exception& error) {
                    ROS_ERROR("Mesh image encoding: %s", error.what());
                    clouds_.erase(scan);
                    continue;
                }
            }
            pcl::PointCloud<pcl::PointXYZI> points;
            pcl::fromROSMsg(*scan->second, points);
            {
                std::lock_guard<std::mutex> lock(g_mapping_mutex);
                if (g_flag_pause) return;
                ReconstructScan(points, imu_pose.ToTransform(), bgr, camera_pose, image_stamp);
                Publish(scan->second->header);
            }
            last_scan_stamp_ = stamp;
            clouds_.erase(scan);
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB> ColoredPoints() const {
        pcl::PointCloud<pcl::PointXYZRGB> result;
        result.reserve(g_map_rgb_pts_mesh.m_rgb_pts_vec.size());
        for (const auto& point : g_map_rgb_pts_mesh.m_rgb_pts_vec) {
            const auto position = point->get_pos();
            const auto bgr = point->get_rgb();
            pcl::PointXYZRGB p;
            p.x = position.x(); p.y = position.y(); p.z = position.z();
            p.r = cv::saturate_cast<unsigned char>(bgr.z());
            p.g = cv::saturate_cast<unsigned char>(bgr.y());
            p.b = cv::saturate_cast<unsigned char>(bgr.x());
            result.push_back(p);
        }
        return result;
    }
    void GetTriangles(std::vector<Triangle_set>& sets) const {
        g_triangles_manager.get_all_triangle_list(sets, nullptr, 0);
    }
    void Publish(const std_msgs::Header& header) {
        const bool cloud_needed = cloud_pub_.getNumSubscribers() > 0;
        const bool mesh_needed = mesh_pub_.getNumSubscribers() > 0;
        if (!cloud_needed && !mesh_needed) return;
        const auto points = ColoredPoints();
        if (cloud_needed) {
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(points, output);
            output.header = header;
            cloud_pub_.publish(output);
        }
        if (mesh_needed) {
            visualization_msgs::Marker mesh;
            mesh.header = header;
            mesh.ns = "fusion_mesh";
            mesh.id = 0;
            mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;
            mesh.action = visualization_msgs::Marker::ADD;
            mesh.pose.orientation.w = 1;
            mesh.scale.x = mesh.scale.y = mesh.scale.z = 1;
            mesh.color.a = 1;
            std::vector<Triangle_set> sets;
            GetTriangles(sets);
            for (const auto& set : sets) for (const auto& triangle : set) {
                if (triangle->m_vis_score < 0) continue;
                for (int v = 0; v < 3; ++v) {
                    const int index = triangle->m_index_flip ? v : (v == 0 ? 0 : 3-v);
                    const auto& p = points[triangle->m_tri_pts_id[index]];
                    geometry_msgs::Point position;
                    position.x = p.x; position.y = p.y; position.z = p.z;
                    std_msgs::ColorRGBA color;
                    color.r = p.r/255.0; color.g = p.g/255.0; color.b = p.b/255.0; color.a = 1;
                    mesh.points.push_back(position);
                    mesh.colors.push_back(color);
                }
            }
            mesh_pub_.publish(mesh);
        }
    }
    bool Save(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
        std::lock_guard<std::mutex> lock(g_mapping_mutex);
        if (g_map_rgb_pts_mesh.m_rgb_pts_vec.empty()) {
            response.success = false;
            response.message = "Map is empty";
            return true;
        }
        pcl::PolygonMesh output;
        pcl::toPCLPointCloud2(ColoredPoints(), output.cloud);
        std::vector<Triangle_set> sets;
        GetTriangles(sets);
        for (const auto& set : sets) for (const auto& triangle : set) {
            if (triangle->m_vis_score < 0) continue;
            pcl::Vertices face;
            for (int v = 0; v < 3; ++v) {
                const int index = triangle->m_index_flip ? v : (v == 0 ? 0 : 3-v);
                face.vertices.push_back(triangle->m_tri_pts_id[index]);
            }
            output.polygons.push_back(std::move(face));
        }
        response.success = pcl::io::savePLYFileBinary(output_mesh_, output) == 0;
        response.message = response.success ? output_mesh_ : "Failed writing " + output_mesh_;
        return true;
    }
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber pose_sub_, cloud_sub_, image_sub_;
    ros::Publisher cloud_pub_, mesh_pub_;
    ros::ServiceServer save_service_;
    std::map<double,Pose> poses_;
    std::map<double,sensor_msgs::PointCloud2ConstPtr> clouds_;
    std::map<double,sensor_msgs::ImageConstPtr> images_;
    Eigen::Isometry3d imu_t_camera_ = Eigen::Isometry3d::Identity();
    std::vector<double> distortion_;
    std::string world_frame_, imu_frame_, output_mesh_;
    int queue_size_;
    bool color_enabled_;
    double image_tolerance_, maximum_pose_gap_, last_scan_stamp_ = -1;
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "external_mapping");
    try {
        ExternalMapping mapping;
        bool show_gui = false;
        ros::NodeHandle("~").param("show_gui", show_gui, false);
#ifdef IM_MESH_WITH_GUI
        if (show_gui) {
            std::thread spinner([] { ros::spin(); });
            try {
                RunMeshGui();
            } catch (...) {
                ros::shutdown();
                spinner.join();
                throw;
            }
            ros::shutdown();
            spinner.join();
        } else ros::spin();
#else
        if (show_gui) throw std::runtime_error("GUI requested but im_mesh was built with IM_MESH_BUILD_GUI=OFF");
        ros::spin();
#endif
    } catch (const std::exception& error) {
        ROS_FATAL("External mapping failed: %s", error.what());
        return 1;
    }
    return 0;
}
