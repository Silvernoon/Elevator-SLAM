cmake_minimum_required(VERSION 3.8)
project(elevator_fusion LANGUAGES C CXX)

option(ELEVATOR_FUSION_BUILD_ROS "Build the ROS 2 scan transport and launch files" ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(elevator_fusion_core src/core.cpp)
target_compile_features(elevator_fusion_core PUBLIC cxx_std_17)
target_include_directories(elevator_fusion_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(elevator_fusion_core PUBLIC Eigen3::Eigen)

if(ELEVATOR_FUSION_BUILD_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(lio REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(pcl_ros REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/FusionStatus.msg"
        DEPENDENCIES std_msgs)

    add_executable(trajectory_fusion src/ros_node.cpp)
    target_compile_features(trajectory_fusion PRIVATE cxx_std_17)
    target_link_libraries(trajectory_fusion PRIVATE elevator_fusion_core)
    ament_target_dependencies(trajectory_fusion PUBLIC
        geometry_msgs lio nav_msgs pcl_ros rclcpp sensor_msgs std_msgs tf2_ros)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(trajectory_fusion PRIVATE "${cpp_typesupport_target}")

    install(TARGETS elevator_fusion_core trajectory_fusion
        EXPORT export_elevator_fusion
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY include/ DESTINATION include)
    install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
    ament_export_targets(export_elevator_fusion HAS_LIBRARY_TARGET)
    ament_export_include_directories(include)
    ament_export_dependencies(Eigen3)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(fusion_core_test test/core_test.cpp)
    target_link_libraries(fusion_core_test PRIVATE elevator_fusion_core)
    add_test(NAME fusion_core_transitions COMMAND fusion_core_test)
endif()

if(ELEVATOR_FUSION_BUILD_ROS)
    ament_package()
endif()
