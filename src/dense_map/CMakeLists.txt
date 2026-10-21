cmake_minimum_required(VERSION 3.10)
project(dense_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs
  visualization_msgs camera_models cv_bridge pcl_ros pcl_conversions)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PCL REQUIRED COMPONENTS common io octree filters)
find_package(Threads REQUIRED)

catkin_package(CATKIN_DEPENDS
  roscpp roslib std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs
  visualization_msgs camera_models cv_bridge pcl_ros pcl_conversions)

add_executable(dense_map_node
  src/pose_graph_node.cpp src/pose_graph.cpp src/keyframe.cpp
  src/utility/CameraPoseVisualization.cpp
  src/ThirdParty/DBoW/BowVector.cpp src/ThirdParty/DBoW/FBrief.cpp
  src/ThirdParty/DBoW/FeatureVector.cpp src/ThirdParty/DBoW/QueryResults.cpp
  src/ThirdParty/DBoW/ScoringObject.cpp src/ThirdParty/DUtils/Random.cpp
  src/ThirdParty/DUtils/Timestamp.cpp src/ThirdParty/DVision/BRIEF.cpp
  src/ThirdParty/VocabularyBinary.cpp)
target_compile_features(dense_map_node PRIVATE cxx_std_17)
target_include_directories(dense_map_node PRIVATE
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_compile_options(dense_map_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(dense_map_node PRIVATE
  Eigen3::Eigen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}
  Boost::filesystem Boost::system ${PCL_LIBRARIES} Threads::Threads)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(dense_map_node ${catkin_EXPORTED_TARGETS})
endif()
install(TARGETS dense_map_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE CHANGELOG.rst DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY src/ThirdParty/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/licenses
  FILES_MATCHING PATTERN "LICENSE.txt")
