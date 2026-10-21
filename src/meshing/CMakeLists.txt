cmake_minimum_required(VERSION 3.12)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tools ${CMAKE_CURRENT_SOURCE_DIR}/include/ikd-Tree
    ${CMAKE_CURRENT_SOURCE_DIR}/include/meshing ${CMAKE_CURRENT_SOURCE_DIR}/include/meshing/r3live
    ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(im_mesh_geometry PUBLIC Eigen3::Eigen CGAL::CGAL TBB::tbb fmt::fmt
    Threads::Threads ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

if(IM_MESH_BUILD_ROS)
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs nav_msgs sensor_msgs
        geometry_msgs visualization_msgs cv_bridge pcl_ros pcl_conversions)
    catkin_package()
    add_executable(external_mapping src/external_mapping.cpp)
    target_include_directories(external_mapping PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(external_mapping PRIVATE im_mesh_geometry ${catkin_LIBRARIES})
    add_dependencies(external_mapping ${catkin_EXPORTED_TARGETS})
    if(IM_MESH_BUILD_GUI)
        target_sources(external_mapping PRIVATE src/mesh_viewer.cpp)
        target_compile_definitions(external_mapping PRIVATE IM_MESH_WITH_GUI)
    endif()
    install(TARGETS im_mesh_geometry external_mapping
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    install(FILES LICENSE.ImMesh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
