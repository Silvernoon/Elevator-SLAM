cmake_minimum_required(VERSION 3.8)
project(global_fusion LANGUAGES CXX)

option(GLOBAL_FUSION_BUILD_ROS "Build the ROS 2 absolute-position transport" ON)
find_package(Ceres REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(global_pose_graph src/global_optimization.cpp)
target_compile_features(global_pose_graph PUBLIC cxx_std_17)
target_include_directories(global_pose_graph PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${CERES_INCLUDE_DIRS})
target_link_libraries(global_pose_graph PUBLIC Eigen3::Eigen ${CERES_LIBRARIES})

if(GLOBAL_FUSION_BUILD_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(rclcpp REQUIRED)

    add_executable(global_fusion_node src/global_fusion_node.cpp)
    target_compile_features(global_fusion_node PRIVATE cxx_std_17)
    target_link_libraries(global_fusion_node PRIVATE global_pose_graph)
    ament_target_dependencies(global_fusion_node PUBLIC nav_msgs rclcpp)

    install(TARGETS global_pose_graph global_fusion_node
        EXPORT export_global_fusion
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})
    install(FILES src/global_optimization.h src/Factors.h DESTINATION include/${PROJECT_NAME})
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
    ament_export_targets(export_global_fusion HAS_LIBRARY_TARGET)
    ament_export_include_directories(include/${PROJECT_NAME})
    ament_export_dependencies(Eigen3)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(global_pose_graph_test test/position_test.cpp)
    target_link_libraries(global_pose_graph_test PRIVATE global_pose_graph)
    add_test(NAME asynchronous_position_anchors COMMAND global_pose_graph_test)
endif()

if(GLOBAL_FUSION_BUILD_ROS)
    ament_package()
endif()
