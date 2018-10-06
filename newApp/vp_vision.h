
#ifndef VP_VISION_H
#define VP_VISION_H

#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>

bool hough_line_detect(cv::Mat & image, cv::Mat & cdst, std::vector<cv::Vec2f> & left_lines, std::vector<cv::Vec2f> & right_lines);
cv::Point vanish_point_detection(cv::Mat & image, cv::Mat & cdst);
cv::Point get_vanish_point(std::vector<cv::Point> & points, cv::Mat & cdst);

#endif // VP_VISION_H
