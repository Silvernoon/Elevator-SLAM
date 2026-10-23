#include "mapping_runtime.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace {
using Odom = nav_msgs::msg::Odometry;
using Image = sensor_msgs::msg::Image;
using Cloud = sensor_msgs::msg::PointCloud2;
using Marker = visualization_msgs::msg::Marker;
struct Pose {
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::Isometry3d ToTransform() const {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = q.toRotationMatrix(); result.translation() = t; return result;
    }
};
class ExternalMapping : public rclcpp::Node {
public:
    ExternalMapping() : Node("external_mapping") {
        world_frame_ = declare_parameter<std::string>("world_frame", "fusion_world");
        imu_frame_ = declare_parameter<std::string>("imu_frame", "fusion_imu");
        color_enabled_ = declare_parameter<bool>("color_enabled", true);
        image_tolerance_ = declare_parameter<double>("image_tolerance", 0.05);
        maximum_pose_gap_ = declare_parameter<double>("maximum_pose_gap", 0.2);
        queue_size_ = declare_parameter<int>("queue_size", 200);
        minimum_pts = declare_parameter<double>("minimum_point_distance", 0.1);
        g_meshing_voxel_size = declare_parameter<double>("meshing_voxel_size", 0.4);
        appending_pts_frame = declare_parameter<int>("points_per_scan", 5000);
        output_mesh_ = declare_parameter<std::string>("output_mesh", "/tmp/fusion_mesh.ply");
        if (queue_size_ < 2 || image_tolerance_ < 0 || maximum_pose_gap_ <= 0 || minimum_pts <= 0 || g_meshing_voxel_size <= 0 || appending_pts_frame < 1)
            throw std::runtime_error("Invalid mapping synchronization or geometry parameters");
        g_map_rgb_pts_mesh.set_minimum_dis(minimum_pts);
        g_map_rgb_pts_mesh.set_voxel_resolution(g_meshing_voxel_size);
        if (color_enabled_) {
            const auto camera_k = declare_parameter<std::vector<double>>("camera_k", std::vector<double>{});
            const auto extrinsic = declare_parameter<std::vector<double>>("imu_t_camera", std::vector<double>{});
            if (camera_k.size() != 9 || extrinsic.size() != 16) throw std::runtime_error("Color mapping needs camera_k[9] and row-major imu_t_camera[16]");
            for (int r=0;r<3;++r) for(int c=0;c<3;++c) g_camera_K(r,c)=camera_k[r*3+c];
            Eigen::Matrix4d transform;
            for(int r=0;r<4;++r) for(int c=0;c<4;++c) transform(r,c)=extrinsic[r*4+c];
            if (!transform.allFinite() || !g_camera_K.allFinite() || g_camera_K(0,0)<=0 || g_camera_K(1,1)<=0 ||
                (transform.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-6 ||
                (transform.block<3,3>(0,0).transpose()*transform.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()>1e-5 ||
                std::abs(transform.block<3,3>(0,0).determinant()-1.0)>1e-5)
                throw std::runtime_error("Camera calibration is not a finite pinhole calibration / rigid T_imu_camera");
            imu_t_camera_.matrix()=transform;
            distortion_=declare_parameter<std::vector<double>>("camera_distortion", std::vector<double>{});
        }
        const auto cloud_topic=declare_parameter<std::string>("cloud_topic", "/fusion/cloud_world");
        const auto odom_topic=declare_parameter<std::string>("odom_topic", "/fusion/odometry");
        const auto image_topic=declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
        auto sensor_qos=rclcpp::QoS(rclcpp::KeepLast(queue_size_)).best_effort();
        cloud_sub_=create_subscription<Cloud>(cloud_topic,sensor_qos,[this](Cloud::ConstSharedPtr m){OnCloud(m);});
        pose_sub_=create_subscription<Odom>(odom_topic,sensor_qos,[this](Odom::ConstSharedPtr m){OnOdometry(m);});
        if(color_enabled_) image_sub_=create_subscription<Image>(image_topic,sensor_qos,[this](Image::ConstSharedPtr m){OnImage(m);});
        auto output_qos=rclcpp::QoS(1).reliable().transient_local();
        cloud_pub_=create_publisher<Cloud>("~/cloud",output_qos);
        mesh_pub_=create_publisher<Marker>("~/mesh",output_qos);
        save_service_=create_service<std_srvs::srv::Trigger>("~/save_mesh",[this](const std_srvs::srv::Trigger::Request::SharedPtr, const std_srvs::srv::Trigger::Response::SharedPtr response){Save(*response);});
    }
private:
    template<class T> void Bound(std::map<double,T>& cache){while(cache.size()>static_cast<std::size_t>(queue_size_))cache.erase(cache.begin());}
    bool PoseAt(double stamp,Pose& result) const {
        auto upper=poses_.lower_bound(stamp);
        if(upper!=poses_.end()&&std::abs(upper->first-stamp)<1e-9){result=upper->second;return true;}
        if(upper==poses_.begin()||upper==poses_.end())return false;
        const auto lower=std::prev(upper); const double gap=upper->first-lower->first;if(gap>maximum_pose_gap_)return false;
        const double alpha=(stamp-lower->first)/gap;result.q=lower->second.q.slerp(alpha,upper->second.q).normalized();result.t=(1-alpha)*lower->second.t+alpha*upper->second.t;return true;
    }
    void OnOdometry(Odom::ConstSharedPtr msg){
        if(msg->header.frame_id!=world_frame_||msg->child_frame_id!=imu_frame_){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Mesh odometry must be world_frame -> imu_frame");return;}
        const auto&p=msg->pose.pose;Pose pose{Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z),Eigen::Vector3d(p.position.x,p.position.y,p.position.z)};
        if(!pose.q.coeffs().allFinite()||pose.q.norm()<1e-9||!pose.t.allFinite())return;pose.q.normalize();poses_[rclcpp::Time(msg->header.stamp).seconds()]=pose;Bound(poses_);Process();
    }
    void OnCloud(Cloud::ConstSharedPtr msg){if(msg->header.frame_id!=world_frame_){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Mesh input cloud must already be in world_frame");return;}clouds_[rclcpp::Time(msg->header.stamp).seconds()]=msg;Bound(clouds_);Process();}
    void OnImage(Image::ConstSharedPtr msg){images_[rclcpp::Time(msg->header.stamp).seconds()]=msg;Bound(images_);Process();}
    void Process(){
        while(!clouds_.empty()&&!poses_.empty()){
            auto scan=clouds_.begin();const double stamp=scan->first;if(stamp<=last_scan_stamp_){clouds_.erase(scan);continue;}Pose imu_pose;
            if(!PoseAt(stamp,imu_pose)){if(poses_.begin()->first>stamp||poses_.rbegin()->first>stamp+maximum_pose_gap_){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Dropping mesh scan without timestamp-bracketed IMU pose");clouds_.erase(scan);continue;}return;}
            cv::Mat bgr;Eigen::Isometry3d camera_pose=Eigen::Isometry3d::Identity();double image_stamp=stamp;
            if(color_enabled_){if(images_.empty())return;auto selected=images_.lower_bound(stamp);if(selected!=images_.begin()){auto before=std::prev(selected);if(selected==images_.end()||stamp-before->first<=selected->first-stamp)selected=before;}
                if(selected==images_.end()||std::abs(selected->first-stamp)>image_tolerance_){if(images_.rbegin()->first>stamp+image_tolerance_){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Dropping mesh scan without time-matched camera image");clouds_.erase(scan);continue;}return;}
                Pose image_imu;image_stamp=selected->first;if(!PoseAt(image_stamp,image_imu)){if(poses_.begin()->first>image_stamp||poses_.rbegin()->first>image_stamp+maximum_pose_gap_){clouds_.erase(scan);continue;}return;}
                camera_pose=image_imu.ToTransform()*imu_t_camera_;try{const auto converted=cv_bridge::toCvShare(selected->second,"bgr8");if(distortion_.empty())bgr=converted->image.clone();else{cv::Mat intrinsics;cv::eigen2cv(g_camera_K,intrinsics);cv::undistort(converted->image,bgr,intrinsics,distortion_);}}
                catch(const cv::Exception&e){RCLCPP_ERROR(get_logger(),"Mesh image conversion: %s",e.what());clouds_.erase(scan);continue;}catch(const cv_bridge::Exception&e){RCLCPP_ERROR(get_logger(),"Mesh image encoding: %s",e.what());clouds_.erase(scan);continue;}}
            pcl::PointCloud<pcl::PointXYZI> points;pcl::fromROSMsg(*scan->second,points);{std::lock_guard<std::mutex>lock(g_mapping_mutex);if(g_flag_pause)return;ReconstructScan(points,imu_pose.ToTransform(),bgr,camera_pose,image_stamp);Publish(scan->second->header);}last_scan_stamp_=stamp;clouds_.erase(scan);
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB> ColoredPoints()const{pcl::PointCloud<pcl::PointXYZRGB> result;result.reserve(g_map_rgb_pts_mesh.m_rgb_pts_vec.size());for(const auto&point:g_map_rgb_pts_mesh.m_rgb_pts_vec){const auto position=point->get_pos();const auto bgr=point->get_rgb();pcl::PointXYZRGB p;p.x=position.x();p.y=position.y();p.z=position.z();p.r=cv::saturate_cast<unsigned char>(bgr.z());p.g=cv::saturate_cast<unsigned char>(bgr.y());p.b=cv::saturate_cast<unsigned char>(bgr.x());result.push_back(p);}return result;}
    void GetTriangles(std::vector<Triangle_set>&sets)const{g_triangles_manager.get_all_triangle_list(sets,nullptr,0);}
    void Publish(const std_msgs::msg::Header&header){if(cloud_pub_->get_subscription_count()==0&&mesh_pub_->get_subscription_count()==0)return;const auto points=ColoredPoints();if(cloud_pub_->get_subscription_count()>0){Cloud output;pcl::toROSMsg(points,output);output.header=header;cloud_pub_->publish(output);}if(mesh_pub_->get_subscription_count()>0){Marker mesh;mesh.header=header;mesh.ns="fusion_mesh";mesh.id=0;mesh.type=Marker::TRIANGLE_LIST;mesh.action=Marker::ADD;mesh.pose.orientation.w=1;mesh.scale.x=mesh.scale.y=mesh.scale.z=1;mesh.color.a=1;std::vector<Triangle_set>sets;GetTriangles(sets);for(const auto&set:sets)for(const auto&triangle:set){if(triangle->m_vis_score<0)continue;for(int v=0;v<3;++v){const int index=triangle->m_index_flip?v:(v==0?0:3-v);const auto&p=points[triangle->m_tri_pts_id[index]];geometry_msgs::msg::Point position;position.x=p.x;position.y=p.y;position.z=p.z;std_msgs::msg::ColorRGBA color;color.r=p.r/255.0;color.g=p.g/255.0;color.b=p.b/255.0;color.a=1;mesh.points.push_back(position);mesh.colors.push_back(color);}}mesh_pub_->publish(mesh);}}
    void Save(std_srvs::srv::Trigger::Response&response){std::lock_guard<std::mutex>lock(g_mapping_mutex);if(g_map_rgb_pts_mesh.m_rgb_pts_vec.empty()){response.success=false;response.message="Map is empty";return;}pcl::PolygonMesh output;pcl::toPCLPointCloud2(ColoredPoints(),output.cloud);std::vector<Triangle_set>sets;GetTriangles(sets);for(const auto&set:sets)for(const auto&triangle:set){if(triangle->m_vis_score<0)continue;pcl::Vertices face;for(int v=0;v<3;++v){const int index=triangle->m_index_flip?v:(v==0?0:3-v);face.vertices.push_back(triangle->m_tri_pts_id[index]);}output.polygons.push_back(std::move(face));}response.success=pcl::io::savePLYFileBinary(output_mesh_,output)==0;response.message=response.success?output_mesh_:"Failed writing "+output_mesh_;}
    rclcpp::Subscription<Odom>::SharedPtr pose_sub_;rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;rclcpp::Subscription<Image>::SharedPtr image_sub_;rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;rclcpp::Publisher<Marker>::SharedPtr mesh_pub_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    std::map<double,Pose>poses_;std::map<double,Cloud::ConstSharedPtr>clouds_;std::map<double,Image::ConstSharedPtr>images_;Eigen::Isometry3d imu_t_camera_=Eigen::Isometry3d::Identity();std::vector<double>distortion_;std::string world_frame_,imu_frame_,output_mesh_;int queue_size_;bool color_enabled_;double image_tolerance_,maximum_pose_gap_,last_scan_stamp_=-1;
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);try{auto mapping=std::make_shared<ExternalMapping>();const bool show_gui=mapping->declare_parameter<bool>("show_gui",false);
#ifdef IM_MESH_WITH_GUI
 if(show_gui){std::thread executor([mapping]{rclcpp::spin(mapping);});try{RunMeshGui(mapping.get());}catch(...){rclcpp::shutdown();executor.join();throw;}rclcpp::shutdown();executor.join();}else rclcpp::spin(mapping);
#else
 if(show_gui)throw std::runtime_error("GUI requested but im_mesh was built with IM_MESH_BUILD_GUI=OFF");rclcpp::spin(mapping);
#endif
}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("external_mapping"),"External mapping failed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
