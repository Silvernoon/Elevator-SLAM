cmake_minimum_required(VERSION 3.16)
project(gnss_comm LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/GnssTimeMsg.msg
  msg/GnssTimePulseInfoMsg.msg
  msg/StampedFloat64Array.msg
  msg/GnssSvsMsg.msg
  msg/GnssEphemMsg.msg
  msg/GnssGloEphemMsg.msg
  msg/GnssMeasMsg.msg
  msg/GnssObsMsg.msg
  msg/GnssPVTSolnMsg.msg
  msg/GnssBestXYZMsg.msg
  DEPENDENCIES std_msgs
)

add_library(gnss_comm_lib
  src/rinex_helper.cpp
  src/gnss_utility.cpp
  src/gnss_spp.cpp
  src/gnss_ros.cpp)
set_target_properties(gnss_comm_lib PROPERTIES EXPORT_NAME gnss_comm OUTPUT_NAME gnss_comm)
target_include_directories(gnss_comm_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(gnss_comm_lib PRIVATE include/gnss_comm)
ament_target_dependencies(gnss_comm_lib PUBLIC rclcpp std_msgs builtin_interfaces)
target_link_libraries(gnss_comm_lib PUBLIC Eigen3::Eigen glog::glog)
rosidl_get_typesupport_target(gnss_comm_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(gnss_comm_lib PUBLIC "${gnss_comm_typesupport}")

install(TARGETS gnss_comm_lib
  EXPORT export_gnss_comm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_gnss_comm HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs builtin_interfaces Eigen3 glog)
ament_package()
