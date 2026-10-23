cmake_minimum_required(VERSION 3.16)
project(camera_models)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(camera_models
  src/chessboard/Chessboard.cc
  src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/PinholeFullCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc)
target_include_directories(camera_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})
target_link_libraries(camera_models PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES})
ament_target_dependencies(camera_models PUBLIC Boost OpenCV Ceres Eigen3)
set_target_properties(camera_models PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(Calibrations src/intrinsic_calib.cc)
target_link_libraries(Calibrations PRIVATE camera_models)

install(TARGETS camera_models
  EXPORT export_camera_models
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS Calibrations RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_targets(export_camera_models HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost OpenCV Ceres Eigen3)
ament_package(CONFIG_EXTRAS cmake/camera_models-extras.cmake)
