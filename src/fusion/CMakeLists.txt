cmake_minimum_required(VERSION 3.10)
project(elevator_fusion LANGUAGES CXX)

option(ELEVATOR_FUSION_BUILD_ROS "Build the ROS 1 scan transport and launch files" ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(elevator_fusion_core src/core.cpp)
target_compile_features(elevator_fusion_core PUBLIC cxx_std_17)
target_include_directories(elevator_fusion_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(elevator_fusion_core PUBLIC Eigen3::Eigen)

if(ELEVATOR_FUSION_BUILD_ROS)
    find_package(catkin REQUIRED COMPONENTS
        roscpp std_msgs geometry_msgs nav_msgs sensor_msgs
        pcl_ros tf2_ros lio message_generation)
    add_message_files(FILES FusionStatus.msg)
    generate_messages(DEPENDENCIES std_msgs)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES elevator_fusion_core
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs
                       pcl_ros tf2_ros lio message_runtime
        DEPENDS EIGEN3)
    add_executable(trajectory_fusion src/ros_node.cpp)
    target_compile_features(trajectory_fusion PRIVATE cxx_std_17)
    target_include_directories(trajectory_fusion PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(trajectory_fusion PRIVATE elevator_fusion_core ${catkin_LIBRARIES})
    add_dependencies(trajectory_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    install(TARGETS trajectory_fusion elevator_fusion_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(fusion_core_test test/core_test.cpp)
    target_link_libraries(fusion_core_test PRIVATE elevator_fusion_core)
    add_test(NAME fusion_core_transitions COMMAND fusion_core_test)
endif()
