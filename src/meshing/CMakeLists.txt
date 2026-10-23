cmake_minimum_required(VERSION 3.16)
project(im_mesh LANGUAGES C CXX)
option(IM_MESH_BUILD_ROS "Build the external fused-trajectory ROS adapter" ON)
option(IM_MESH_BUILD_GUI "Build the interactive PCL mesh viewer" OFF)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
find_package(CGAL REQUIRED)
find_package(TBB REQUIRED)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem iostreams system serialization)
set(MESH_PCL_COMPONENTS common io kdtree)
if(IM_MESH_BUILD_GUI)
    list(APPEND MESH_PCL_COMPONENTS visualization)
endif()
find_package(PCL REQUIRED COMPONENTS ${MESH_PCL_COMPONENTS})
list(REMOVE_ITEM PCL_LIBRARIES QHULL::QHULL)

add_library(im_mesh_geometry
    src/reconstruction.cpp src/mesh_rec_geometry.cpp
    src/r3live/pointcloud_rgbd.cpp src/r3live/image_frame.cpp src/r3live/triangle.cpp
    include/ikd-Tree/ikd_Tree.cpp)
target_compile_features(im_mesh_geometry PUBLIC cxx_std_17)
target_include_directories(im_mesh_geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tools>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ikd-Tree>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/meshing>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/meshing/r3live>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(im_mesh_geometry PUBLIC Eigen3::Eigen CGAL::CGAL TBB::tbb fmt::fmt
    Threads::Threads ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

if(IM_MESH_BUILD_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(pcl_conversions REQUIRED)
    add_executable(external_mapping src/external_mapping.cpp)
    target_link_libraries(external_mapping PRIVATE im_mesh_geometry)
    ament_target_dependencies(external_mapping PUBLIC rclcpp std_msgs std_srvs nav_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge pcl_conversions)
    if(IM_MESH_BUILD_GUI)
        target_sources(external_mapping PRIVATE src/mesh_viewer.cpp)
        target_compile_definitions(external_mapping PRIVATE IM_MESH_WITH_GUI)
    endif()
    install(TARGETS im_mesh_geometry external_mapping
        EXPORT export_im_mesh
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY include/ DESTINATION include)
    install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
    install(FILES LICENSE.ImMesh DESTINATION share/${PROJECT_NAME})
    ament_export_targets(export_im_mesh HAS_LIBRARY_TARGET)
    ament_export_dependencies(Eigen3 OpenCV CGAL TBB fmt Boost rclcpp std_msgs std_srvs nav_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge pcl_conversions)
    ament_package()
endif()
