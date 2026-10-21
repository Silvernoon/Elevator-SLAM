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
#pragma once
#include <memory>
#include <unordered_set>
#include <vector>
#include "tools_eigen.hpp"
#include "tools_kd_hash.hpp"
#include "tools_graphics.hpp"
#include "ikd_Tree.h"

extern double g_voxel_resolution;
class RGB_Voxel;
using RGB_voxel_ptr = std::shared_ptr<RGB_Voxel>;

// Retained R3LIVE point/color estimator; trajectory and sensor transport live elsewhere.
class RGB_pts {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double m_pos[3] = {};
    double m_pos_aft_smooth[3] = {};
    double m_rgb[3] = {255, 255, 255};
    double m_cov_rgb[3] = {};
    int m_N_rgb = 0;
    int m_pt_index = 0;
    bool m_smoothed = false;
    int m_is_inner_pt = 0;
    RGB_voxel_ptr m_parent_voxel;
    double m_obs_dis = 0;
    double m_last_obs_time = 0;

    void set_pos(const vec_3& pos);
    void set_smooth_pos(const vec_3& pos);
    vec_3 get_pos(bool smooth = false) const;
    vec_3 get_rgb() const;
    int update_rgb(const vec_3& rgb, double distance, const vec_3 sigma, double stamp);
};
using RGB_pt_ptr = std::shared_ptr<RGB_pts>;

class RGB_Voxel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double m_last_visited_time = 0;
    long m_pos[3];
    vec_3 m_center;
    vec_3 m_long_axis = vec_3::Zero();
    vec_3 m_mid_axis = vec_3::Zero();
    vec_3 m_short_axis = vec_3::Zero();
    long m_meshing_times = 0;
    long m_new_added_pts_count = 0;
    std::vector<RGB_pt_ptr> m_pts_in_grid;
    RGB_Voxel(long x, long y, long z)
        : m_pos{x, y, z}, m_center(x * g_voxel_resolution, y * g_voxel_resolution, z * g_voxel_resolution) {}
};

using KDtree_pt = ikdTree_PointType;
using KDtree_pt_vector = KD_TREE<KDtree_pt>::PointVector;
struct Global_map {
    std::vector<RGB_pt_ptr> m_rgb_pts_vec;
    std::vector<RGB_voxel_ptr> m_voxel_vec;
    KD_TREE<KDtree_pt> m_kdtree;
    Hash_map_3d<long, RGB_pt_ptr> m_hashmap_3d_pts;
    Hash_map_3d<long, RGB_voxel_ptr> m_hashmap_voxels;
    std::unordered_set<RGB_voxel_ptr> m_voxels_recent_visited;
    double m_minimum_pts_size = 0.05;
    double m_voxel_resolution = 0.1;
    void set_minimum_dis(double distance);
    void set_voxel_resolution(double resolution);
    vec_3 smooth_pts(RGB_pt_ptr& point, double factor, double neighbors = 20, double maximum_distance = 0);
};
std::vector<RGB_pt_ptr> retrieve_pts_in_voxels(std::unordered_set<RGB_voxel_ptr>& voxels);
