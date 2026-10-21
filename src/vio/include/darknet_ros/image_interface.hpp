/*
 * image_interface.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef IMAGE_INTERFACE_H
#define IMAGE_INTERFACE_H

#include "image.h"
#include "opencv2/opencv.hpp"

extern "C" cv::Mat image_to_mat(image im);
extern "C" image mat_to_image(cv::Mat m);
image** load_alphabet_with_file(char* datafile);
void generate_image(image p, cv::Mat& disp);

#endif
