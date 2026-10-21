add_library(vins_line_descriptor
  src/binary_descriptor_custom.cpp src/binary_descriptor_matcher.cpp
  src/draw_custom.cpp src/LSDDetector_custom.cpp)
target_include_directories(vins_line_descriptor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vins_line_descriptor PUBLIC ${OpenCV_LIBS})
