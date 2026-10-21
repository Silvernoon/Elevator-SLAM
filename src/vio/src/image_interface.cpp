/* Copyright (c) 2017 Marko Bjelonic, ETH Zurich. BSD-3-Clause.
 * OpenCV-generated label glyphs replace Darknet's external PNG alphabet assets.
 */
#include "darknet_ros/image_interface.hpp"

image** load_alphabet_with_file(char*) {
  constexpr int sizes = 8;
  image** alphabet = static_cast<image**>(calloc(sizes, sizeof(image*)));
  for (int size = 0; size < sizes; ++size) {
    alphabet[size] = static_cast<image*>(calloc(128, sizeof(image)));
    for (int character = 32; character < 127; ++character) {
      const std::string text(1, static_cast<char>(character));
      int baseline = 0;
      const double scale = 0.4 + 0.15 * size;
      const cv::Size extent = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
      cv::Mat glyph(extent.height + baseline + 4, std::max(1, extent.width + 2), CV_8UC3, cv::Scalar(255, 255, 255));
      cv::putText(glyph, text, cv::Point(1, extent.height + 1), cv::FONT_HERSHEY_SIMPLEX,
                  scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      alphabet[size][character] = mat_to_image(glyph);
    }
  }
  return alphabet;
}

void generate_image(image source, cv::Mat& destination) {
  destination = image_to_mat(source);
}
