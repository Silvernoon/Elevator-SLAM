/*
 * External-trajectory reconstruction adapted from ImMesh_mesh_reconstruction.cpp.
 * ImMesh: Jiarong Lin et al., IEEE Transactions on Robotics, 2023.
 * Original copyright, redistribution terms and disclaimers are retained in
 * LICENSE.ImMesh and the imported geometry/R3LIVE source files.
 */
#include "mapping_runtime.hpp"
#include "meshing/r3live/image_frame.hpp"
#include <cmath>
#include <set>

Global_map g_map_rgb_pts_mesh;
Triangle_manager g_triangles_manager(&g_map_rgb_pts_mesh);
std::mutex g_mapping_mutex;
Eigen::Matrix3d g_camera_K = Eigen::Matrix3d::Identity();
int g_current_frame = -1;
double minimum_pts = 0.1;
double g_meshing_voxel_size = 0.4;
int appending_pts_frame = 5000;
bool g_flag_pause = false;

void ReconstructScan(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                      const Eigen::Isometry3d& world_t_imu,
                      const cv::Mat& image,
                      const Eigen::Isometry3d& world_t_camera,
                      double image_stamp)
{
    const int frame = g_current_frame + 1;

    std::unordered_set<RGB_voxel_ptr> visited;
    KDtree_pt_vector nearest;
    std::vector<float> distances;
    KDtree_pt_vector insertion(1);
    const std::size_t step = std::max<std::size_t>(1,
        (cloud.size() + appending_pts_frame - 1) / appending_pts_frame);
    for (std::size_t i = 0; i < cloud.size(); i += step) {
        const auto& p = cloud[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const long gx = std::lround(p.x / minimum_pts);
        const long gy = std::lround(p.y / minimum_pts);
        const long gz = std::lround(p.z / minimum_pts);
        const long vx = std::lround(p.x / g_meshing_voxel_size);
        const long vy = std::lround(p.y / g_meshing_voxel_size);
        const long vz = std::lround(p.z / g_meshing_voxel_size);
        auto voxel_slot = g_map_rgb_pts_mesh.m_hashmap_voxels.get_data(vx, vy, vz);
        RGB_voxel_ptr voxel;
        if (!voxel_slot) {
            voxel = std::make_shared<RGB_Voxel>(vx, vy, vz);
            g_map_rgb_pts_mesh.m_hashmap_voxels.insert(vx, vy, vz, voxel);
            g_map_rgb_pts_mesh.m_voxel_vec.push_back(voxel);
        } else {
            voxel = *voxel_slot;
        }
        visited.insert(voxel);
        voxel->m_last_visited_time = frame;
        if (g_map_rgb_pts_mesh.m_hashmap_3d_pts.get_data(gx, gy, gz)) continue;
        KDtree_pt indexed(vec_3(p.x, p.y, p.z), 0);
        if (g_map_rgb_pts_mesh.m_kdtree.Root_Node) {
            g_map_rgb_pts_mesh.m_kdtree.Nearest_Search(indexed, 1, nearest, distances);
            if (!distances.empty() && distances.front() < minimum_pts * minimum_pts) continue;
        }
        auto point = std::make_shared<RGB_pts>();
        point->set_pos(vec_3(p.x, p.y, p.z));
        point->m_pt_index = g_map_rgb_pts_mesh.m_rgb_pts_vec.size();
        indexed.m_pt_idx = point->m_pt_index;
        g_map_rgb_pts_mesh.m_rgb_pts_vec.push_back(point);
        g_map_rgb_pts_mesh.m_hashmap_3d_pts.insert(gx, gy, gz, point);
        voxel->m_pts_in_grid.push_back(point);
        ++voxel->m_new_added_pts_count;
        voxel->m_meshing_times = 0;
        insertion[0] = indexed;
        g_map_rgb_pts_mesh.m_kdtree.Add_Points(insertion, false);
    }
    g_map_rgb_pts_mesh.m_voxels_recent_visited = visited;

    if (!image.empty()) {
        Image_frame camera(g_camera_K);
        // Image_frame's legacy w2c names actually store the camera pose in world;
        // project_3d_to_2d explicitly inverts this pose.
        camera.set_pose(Eigen::Quaterniond(world_t_camera.rotation()), world_t_camera.translation());
        camera.m_timestamp = image_stamp;
        camera.m_img = image;
        camera.init_cubic_interpolation();
        camera.image_equalize();
        for (const auto& voxel : visited) {
            for (const auto& point : voxel->m_pts_in_grid) {
                const vec_3 position = point->get_pos();
                double u, v;
                if (!camera.project_3d_point_in_this_img(position, u, v)) continue;
                const double distance = (position - world_t_camera.translation()).norm();
                point->update_rgb(camera.get_rgb(u, v, 0), distance,
                                  vec_3(1.5, 1.5, 1.5), image_stamp);
            }
        }
    }

    // Preserve ImMesh's voxel-wise pull, local PCA/Delaunay, and triangle push.
    // A scan is processed atomically: adjacent voxel updates cannot race each other.
    for (const auto& voxel : visited) {
        if (voxel->m_meshing_times >= 1 || voxel->m_pts_in_grid.size() < 5) continue;
        std::unordered_set<RGB_voxel_ptr> neighborhood{voxel};
        auto points = retrieve_pts_in_voxels(neighborhood);
        if (points.size() < 3) continue;
        points = retrieve_neighbor_pts_kdtree(points);
        points = remove_outlier_pts(points, voxel);
        std::set<long> relative_indices;
        for (const auto& point : points) relative_indices.insert(point->m_pt_index);
        points.clear();
        for (long index : relative_indices) points.push_back(g_map_rgb_pts_mesh.m_rgb_pts_vec[index]);
        std::set<long> hull, inner;
        voxel->m_short_axis.setZero();
        auto indices = delaunay_triangulation(points, voxel->m_long_axis, voxel->m_mid_axis,
                                             voxel->m_short_axis, hull, inner);
        auto old_triangles = g_triangles_manager.find_relative_triangulation_combination(relative_indices);
        Triangle_set removed, added, existing;
        triangle_compare(old_triangles, indices, removed, added, &existing);
        for (auto triangle : added)
            correct_triangle_index(triangle, world_t_imu.translation(), voxel->m_short_axis);
        for (auto triangle : existing)
            correct_triangle_index(triangle, world_t_imu.translation(), voxel->m_short_axis);
        g_triangles_manager.remove_triangle_list(removed);
        for (const auto& triangle : added) {
            auto inserted = g_triangles_manager.insert_triangle(triangle->m_tri_pts_id[0],
                triangle->m_tri_pts_id[1], triangle->m_tri_pts_id[2], 1);
            inserted->m_index_flip = triangle->m_index_flip;
        }
        ++voxel->m_meshing_times;
        voxel->m_new_added_pts_count = 0;
    }
    g_current_frame = frame;
}
