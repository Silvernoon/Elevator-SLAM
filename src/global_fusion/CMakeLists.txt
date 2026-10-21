cmake_minimum_required(VERSION 3.10)
project(global_fusion LANGUAGES CXX)
option(GLOBAL_FUSION_BUILD_ROS "Build the optional ROS absolute-position transport" ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)

add_library(global_pose_graph src/global_optimization.cpp)
target_compile_features(global_pose_graph PUBLIC cxx_std_17)
target_include_directories(global_pose_graph PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(global_pose_graph PUBLIC Eigen3::Eigen ${CERES_LIBRARIES})

if(GLOBAL_FUSION_BUILD_ROS)
    find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs)
    catkin_package()
    add_executable(global_fusion_node src/global_fusion_node.cpp)
    target_include_directories(global_fusion_node PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(global_fusion_node PRIVATE global_pose_graph ${catkin_LIBRARIES})
    add_dependencies(global_fusion_node ${catkin_EXPORTED_TARGETS})
    install(TARGETS global_pose_graph global_fusion_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(global_pose_graph_test test/position_test.cpp)
    target_link_libraries(global_pose_graph_test PRIVATE global_pose_graph)
    add_test(NAME asynchronous_position_anchors COMMAND global_pose_graph_test)
endif()
