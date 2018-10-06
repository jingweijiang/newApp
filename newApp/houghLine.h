#ifndef HOUGHLINE_H
#define HOUGHLINE_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

/**
 * @brief hough_line_detect     检测直线，左右水平直线以及修正后的直线
 * @param image                 输入图像
 * @param horizontal_lines      检测到的所有水平线
 * @return
 */
bool hough_line_detect(Mat & image,Mat & cdst, vector<Vec2f> & horizontal_lines);
/**
 * @brief draw_line   划线
 * @param image       输入图形
 * @param vec_lines   输入直线
 * @param color       颜色
 * @return
 */
bool draw_line(Mat & image, vector<Vec2f> & vec_lines, Scalar color);
/**
 * @brief findParall        寻找相互平行线
 * @param left
 * @param right
 * @param deal_left_lines
 * @param deal_right_lines
 * @param Angle             阈值
 * @return
 */
bool findParall(vector<Vec2f> &left , vector<Vec2f> &right ,
                vector<Vec2f> & deal_left_lines, vector<Vec2f> & deal_right_lines, int Angle);
/**
 * @brief ClassHorizationalLines
 * @param horizontalLines
 * @param Obstacle
 * @return
 */
bool ClassHorizationalLines(vector<Vec2f> &horizontalLines,vector<Vec2f> Obstacle);

#endif // HOUGHLINE_H
