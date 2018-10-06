#ifndef __AUTO_CANNY__
#define __AUTO_CANNY__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

double medianMat(cv::Mat Input);
cv::Mat auto_canny(cv::Mat &image,cv::Mat &output,float sigma);


#endif
