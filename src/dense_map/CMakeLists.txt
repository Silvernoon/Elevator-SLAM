cmake_minimum_required(VERSION 3.16)
project(dense_map LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(camera_models REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PCL REQUIRED COMPONENTS common io octree filters)
find_package(Threads REQUIRED)

add_executable(dense_map_node
  src/pose_graph_node.cpp
  src/pose_graph.cpp
  src/keyframe.cpp
  src/utility/CameraPoseVisualization.cpp
  src/ThirdParty/DBoW/BowVector.cpp
  src/ThirdParty/DBoW/FBrief.cpp
  src/ThirdParty/DBoW/FeatureVector.cpp
  src/ThirdParty/DBoW/QueryResults.cpp
  src/ThirdParty/DBoW/ScoringObject.cpp
  src/ThirdParty/DUtils/Random.cpp
  src/ThirdParty/DUtils/Timestamp.cpp
  src/ThirdParty/DVision/BRIEF.cpp
  src/ThirdParty/VocabularyBinary.cpp)
target_compile_features(dense_map_node PRIVATE cxx_std_17)
target_include_directories(dense_map_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_compile_options(dense_map_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(dense_map_node PUBLIC
  ament_index_cpp rclcpp std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs
  visualization_msgs camera_models cv_bridge pcl_conversions)
target_link_libraries(dense_map_node PRIVATE
  Eigen3::Eigen ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}
  Boost::filesystem Boost::system ${PCL_LIBRARIES} Threads::Threads)

install(TARGETS dense_map_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch resources
  DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE CHANGELOG.rst
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/ThirdParty/
  DESTINATION share/${PROJECT_NAME}/licenses
  FILES_MATCHING PATTERN "LICENSE.txt")

ament_export_dependencies(
  ament_index_cpp rclcpp std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs
  visualization_msgs camera_models cv_bridge pcl_conversions Eigen3 OpenCV Ceres
  Boost PCL)
ament_package()
