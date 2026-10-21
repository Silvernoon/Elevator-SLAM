cmake_minimum_required(VERSION 3.10)
project(camera_models)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES camera_models CATKIN_DEPENDS roscpp std_msgs DEPENDS OpenCV Ceres)
add_library(camera_models
  src/chessboard/Chessboard.cc src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc src/camera_models/CameraFactory.cc src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc src/camera_models/PinholeFullCamera.cc
  src/camera_models/CataCamera.cc src/camera_models/EquidistantCamera.cc src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc src/gpl/gpl.cc src/gpl/EigenQuaternionParameterization.cc)
target_include_directories(camera_models PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(camera_models PUBLIC Eigen3::Eigen ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(camera_models PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_executable(Calibrations src/intrinsic_calib.cc)
target_link_libraries(Calibrations PRIVATE camera_models)
install(TARGETS camera_models Calibrations ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
