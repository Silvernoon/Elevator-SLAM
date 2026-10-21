/*
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored,
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored,
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package."
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping."
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#include "pointcloud_rgbd.hpp"
#include <cmath>

// No internal sensor-processing worker or ROS publisher: the external adapter owns execution.
double g_voxel_resolution = 0.1;

void RGB_pts::set_pos(const vec_3& pos) {
    for (int i = 0; i < 3; ++i) m_pos[i] = m_pos_aft_smooth[i] = pos(i);
}
void RGB_pts::set_smooth_pos(const vec_3& pos) {
    for (int i = 0; i < 3; ++i) m_pos_aft_smooth[i] = pos(i);
    m_smoothed = true;
}
vec_3 RGB_pts::get_pos(bool smooth) const {
    const double* p = smooth ? m_pos_aft_smooth : m_pos;
    return vec_3(p[0], p[1], p[2]);
}
vec_3 RGB_pts::get_rgb() const { return vec_3(m_rgb[0], m_rgb[1], m_rgb[2]); }

int RGB_pts::update_rgb(const vec_3& rgb, double distance, const vec_3 sigma, double stamp) {
    if (m_obs_dis != 0 && distance > m_obs_dis * 1.2) return 0;
    if (m_N_rgb == 0) {
        m_last_obs_time = stamp;
        m_obs_dis = distance;
        for (int i = 0; i < 3; ++i) { m_rgb[i] = rgb(i); m_cov_rgb[i] = sigma(i); }
        m_N_rgb = 1;
        return 0;
    }
    for (int i = 0; i < 3; ++i) {
        const double prior = m_cov_rgb[i] + 0.15 * (stamp - m_last_obs_time);
        const double variance = 1.0 / (1.0 / (prior * prior) + 1.0 / (sigma(i) * sigma(i)));
        m_rgb[i] = variance * (m_rgb[i] / (prior * prior) + rgb(i) / (sigma(i) * sigma(i)));
        m_cov_rgb[i] = std::sqrt(variance);
    }
    m_obs_dis = std::min(m_obs_dis, distance);
    m_last_obs_time = stamp;
    ++m_N_rgb;
    return 1;
}
void Global_map::set_minimum_dis(double distance) {
    m_minimum_pts_size = distance;
    m_kdtree.set_downsample_param(distance);
}
void Global_map::set_voxel_resolution(double resolution) {
    m_voxel_resolution = resolution;
    g_voxel_resolution = resolution;
}
vec_3 Global_map::smooth_pts(RGB_pt_ptr& point, double factor, double neighbors, double maximum_distance) {
    const vec_3 position = point->get_pos();
    if (maximum_distance <= 0) maximum_distance = m_voxel_resolution * 0.8;
    KDtree_pt_vector near;
    std::vector<float> distances;
    m_kdtree.Nearest_Search(KDtree_pt(position), static_cast<int>(neighbors), near, distances);
    vec_3 sum = vec_3::Zero();
    std::size_t count = 0;
    for (std::size_t i = 1; i < near.size(); ++i) {
        if (distances[i] < maximum_distance * maximum_distance) {
            sum += m_rgb_pts_vec[near[i].m_pt_idx]->get_pos();
            ++count;
        }
    }
    const vec_3 smoothed = count ? vec_3(position * (1 - factor) + sum * (factor / count)) : position;
    point->set_smooth_pos(smoothed);
    return smoothed;
}
std::vector<RGB_pt_ptr> retrieve_pts_in_voxels(std::unordered_set<RGB_voxel_ptr>& voxels) {
    std::size_t count = 0;
    for (const auto& voxel : voxels) count += voxel->m_pts_in_grid.size();
    std::vector<RGB_pt_ptr> points;
    points.reserve(count);
    for (const auto& voxel : voxels) points.insert(points.end(), voxel->m_pts_in_grid.begin(), voxel->m_pts_in_grid.end());
    return points;
}
