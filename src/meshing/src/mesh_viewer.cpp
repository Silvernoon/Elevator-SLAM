/* External-trajectory viewer for retained ImMesh geometry. Original reconstruction
 * attribution and usage terms are reproduced in LICENSE.ImMesh. */
#include "mapping_runtime.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <thread>

namespace {
struct ViewerState {
    pcl::PolygonMesh mesh;
    std::string output_path;
    rclcpp::Logger logger{rclcpp::get_logger("mesh_viewer")};
    bool wireframe = false;
    bool representation_changed = false;
};
void OnKeyboard(const pcl::visualization::KeyboardEvent& event, void* data) {
    if (!event.keyDown()) return;
    auto& state = *static_cast<ViewerState*>(data);
    if (event.getKeySym() == "space") {
        std::lock_guard<std::mutex> lock(g_mapping_mutex); g_flag_pause = !g_flag_pause;
    } else if (event.getKeySym() == "w") {
        state.wireframe = !state.wireframe; state.representation_changed = true;
    } else if (event.getKeySym() == "s") {
        if (state.mesh.polygons.empty()) RCLCPP_WARN(state.logger, "No mesh triangles to save");
        else if (pcl::io::savePLYFileBinary(state.output_path, state.mesh) == 0) RCLCPP_INFO(state.logger, "Saved mesh to %s", state.output_path.c_str());
        else RCLCPP_ERROR(state.logger, "Cannot save mesh to %s", state.output_path.c_str());
    }
}
void SnapshotMesh(pcl::PointCloud<pcl::PointXYZRGB>& points, pcl::PolygonMesh& mesh) {
    points.resize(g_map_rgb_pts_mesh.m_rgb_pts_vec.size());
    for (std::size_t i=0;i<points.size();++i) {
        const auto p=g_map_rgb_pts_mesh.m_rgb_pts_vec[i]->get_pos();const auto bgr=g_map_rgb_pts_mesh.m_rgb_pts_vec[i]->get_rgb();auto&out=points[i];
        out.x=p.x();out.y=p.y();out.z=p.z();out.r=cv::saturate_cast<unsigned char>(bgr.z());out.g=cv::saturate_cast<unsigned char>(bgr.y());out.b=cv::saturate_cast<unsigned char>(bgr.x());
    }
    pcl::toPCLPointCloud2(points,mesh.cloud);mesh.polygons.clear();std::vector<Triangle_set>sets;const auto count=g_triangles_manager.get_all_triangle_list(sets,nullptr,0);mesh.polygons.reserve(count);
    for(const auto&set:sets)for(const auto&triangle:set){if(triangle->m_vis_score<0)continue;pcl::Vertices face;face.vertices.resize(3);for(int v=0;v<3;++v){const int index=triangle->m_index_flip?v:(v==0?0:3-v);face.vertices[v]=triangle->m_tri_pts_id[index];}mesh.polygons.push_back(std::move(face));}
}
}
void RunMeshGui(rclcpp::Node* node) {
    pcl::visualization::PCLVisualizer viewer("Elevator-SLAM mesh: space pause, w wireframe, s save");viewer.setBackgroundColor(0.08,0.08,0.08);viewer.addCoordinateSystem(1.0);
    ViewerState state;state.output_path=node->get_parameter("output_mesh").as_string();state.logger=node->get_logger();viewer.registerKeyboardCallback(OnKeyboard,&state);
    pcl::PointCloud<pcl::PointXYZRGB>points;int shown_frame=-1;bool first_view=true;
    while(rclcpp::ok()&&!viewer.wasStopped()){
        bool changed=false;{std::lock_guard<std::mutex>lock(g_mapping_mutex);if(shown_frame!=g_current_frame){SnapshotMesh(points,state.mesh);shown_frame=g_current_frame;changed=true;}}
        if(changed&&!state.mesh.polygons.empty()){viewer.removePolygonMesh("fusion_mesh");viewer.addPolygonMesh(state.mesh,"fusion_mesh");if(first_view){viewer.resetCamera();first_view=false;}state.representation_changed=true;}
        if(state.representation_changed){if(state.wireframe)viewer.setRepresentationToWireframeForAllActors();else viewer.setRepresentationToSurfaceForAllActors();state.representation_changed=false;}
        viewer.getRenderWindow()->Render();
        viewer.getRenderWindow()->GetInteractor()->ProcessEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
