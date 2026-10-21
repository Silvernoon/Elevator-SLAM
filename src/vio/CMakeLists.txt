cmake_minimum_required(VERSION 3.12)
project(vins LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf cv_bridge camera_models image_transport actionlib darknet_ros_msgs rosbag message_generation gnss_comm)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED CONFIG)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)
add_message_files(FILES LocalSensorExternalTrigger.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf cv_bridge camera_models image_transport actionlib darknet_ros_msgs rosbag message_runtime gnss_comm)

add_subdirectory(src/linefeatureTracker/line_descriptor)
add_library(vins_estimator
  src/estimator/parameters.cpp src/estimator/estimator.cpp src/estimator/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp src/factor/pose_subset_parameterization.cpp
  src/factor/orientation_local_parameterization.cpp src/factor/orientation_subset_parameterization.cpp
  src/factor/projectionTwoFrameOneCamFactor.cpp src/factor/projectionTwoFrameTwoCamFactor.cpp
  src/factor/projectionOneFrameTwoCamFactor.cpp src/factor/marginalization_factor.cpp
  src/utility/utility.cpp src/utility/visualization.cpp src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp src/initial/initial_aligment.cpp src/initial/initial_sfm.cpp src/initial/initial_ex_rotation.cpp
  src/featureTracker/feature_tracker.cpp src/linefeatureTracker/linefeature_tracker.cpp
  src/utility/line_geometry.cpp src/factor/line_projection_factor.cpp src/factor/line_parameterization.cpp src/factor/projection_factor.cpp
  src/factor/gnss_psr_dopp_factor.cpp src/factor/gnss_dt_ddt_factor.cpp src/factor/gnss_dt_anchor_factor.cpp
  src/factor/gnss_ddt_smooth_factor.cpp src/factor/pos_vel_factor.cpp src/factor/pose_anchor_factor.cpp src/initial/gnss_vi_initializer.cpp)
target_include_directories(vins_estimator PUBLIC src ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(vins_estimator PUBLIC vins_line_descriptor Eigen3::Eigen Sophus::Sophus fmt::fmt ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} Threads::Threads)
add_dependencies(vins_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(vins_node src/vio_node.cpp)
target_link_libraries(vins_node PRIVATE vins_estimator)
add_dependencies(vins_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS vins_estimator vins_node vins_line_descriptor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Detector is a separate CPU-capable target. CUDA is opt-in and never leaks into VIO.
option(VINS_BUILD_YOLO "Build the standalone Darknet detector" ON)
option(VINS_YOLO_CUDA "Use CUDA in the standalone detector" OFF)
if(VINS_BUILD_YOLO)
  find_package(Boost REQUIRED COMPONENTS thread)
  set(DARKNET_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/darknet)
  file(GLOB DARKNET_C_SOURCES CONFIGURE_DEPENDS ${DARKNET_PATH}/src/*.c)
  list(REMOVE_ITEM DARKNET_C_SOURCES ${DARKNET_PATH}/src/compare.c)
  if(VINS_YOLO_CUDA)
    find_package(CUDA REQUIRED)
    file(GLOB DARKNET_CUDA_SOURCES ${DARKNET_PATH}/src/*.cu)
    cuda_add_library(vins_darknet ${DARKNET_C_SOURCES} ${DARKNET_PATH}/src/image_opencv.cpp ${DARKNET_CUDA_SOURCES})
    target_compile_definitions(vins_darknet PUBLIC GPU)
    target_include_directories(vins_darknet PUBLIC ${CUDA_INCLUDE_DIRS})
    target_link_libraries(vins_darknet PUBLIC ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY})
  else()
    add_library(vins_darknet ${DARKNET_C_SOURCES} ${DARKNET_PATH}/src/image_opencv.cpp)
  endif()
  target_compile_definitions(vins_darknet PUBLIC OPENCV)
  target_include_directories(vins_darknet PUBLIC ${DARKNET_PATH}/src ${DARKNET_PATH}/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(vins_darknet PUBLIC ${OpenCV_LIBS} Threads::Threads m)
  add_library(darknet_ros_lib src/YoloObjectDetector.cpp src/image_interface.cpp)
  target_include_directories(darknet_ros_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  target_link_libraries(darknet_ros_lib PUBLIC vins_darknet ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(darknet_ros_lib ${catkin_EXPORTED_TARGETS})
  add_executable(darknet_ros src/yolo_object_detector_node.cpp)
  target_link_libraries(darknet_ros PRIVATE darknet_ros_lib)
  add_dependencies(darknet_ros ${catkin_EXPORTED_TARGETS})
  install(TARGETS vins_darknet darknet_ros_lib darknet_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
install(DIRECTORY config launch yolo_network_config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
