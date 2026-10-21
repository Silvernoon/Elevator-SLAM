#pragma once

#include <mutex>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "meshing/mesh_rec_geometry.hpp"

extern Global_map g_map_rgb_pts_mesh;
extern Triangle_manager g_triangles_manager;
extern std::mutex g_mapping_mutex;
extern Eigen::Matrix3d g_camera_K;
extern int g_current_frame;
extern double minimum_pts;
extern double g_meshing_voxel_size;
extern int appending_pts_frame;
extern bool g_flag_pause;

// All poses map local coordinates into fusion_world. Cloud is already in fusion_world.
// Caller holds g_mapping_mutex, also used by the optional GUI.
void ReconstructScan(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                      const Eigen::Isometry3d& world_t_imu,
                      const cv::Mat& image,
                      const Eigen::Isometry3d& world_t_camera,
                      double image_stamp);
#ifdef IM_MESH_WITH_GUI
void RunMeshGui();
#endif
