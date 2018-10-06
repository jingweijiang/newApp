#include "houghLine.h"
#include "auto_canny.h"
// 根据theta 确定 horizontal lines
bool hough_line_detect(Mat & image, Mat & cdst,  vector<Vec2f> & horizontal_lines)
{
    Mat dst;
    //初始化变量
    auto_canny(image,dst,0.5);
    cvtColor(dst, cdst, CV_GRAY2BGR);

    vector<Vec2f> lines;
    Vec2f max;
    float rho = 0,theta;

    HoughLines(dst, lines, 1, CV_PI /180, 100, 0, 0);
    for (size_t i = 0; i < lines.size(); i++)
    {
        if(rho<lines[i][0])
        {
            rho = lines[i][0];
            max[0] = rho;
            max[1] = lines[i][1];
        }

        theta = lines[i][1];


        if(70 * CV_PI / 180 < theta && theta < 110 * CV_PI / 180)
        {
           horizontal_lines.push_back(lines[i]);
        }
    }

    cv::circle(cdst,Point(320,(max[0]-320*cos(max[1]))/sin(max[1])),8,CV_RGB(255,0,0),2);

    draw_line(image, horizontal_lines, Scalar(0, 255,0));      //red
    horizontal_lines.clear();

    return true;
}
/**
 * @brief draw_line draw lines reserved in the vector vec_lines
 * @param image     target image
 * @param vec_lines container
 * @param color
 * @return
 */
bool draw_line(Mat & image, vector<Vec2f> & vec_lines, Scalar color)
{
    for (size_t i = 0; i < vec_lines.size(); ++i)
    {
        float rho = vec_lines[i][0], theta = vec_lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 10000 * (-b));//对一个double型的数进行四舍五入
        pt1.y = cvRound(y0 + 10000 * (a));
        pt2.x = cvRound(x0 - 10000 * (-b));
        pt2.y = cvRound(y0 - 10000 * (a));

        cv::line(image, pt1, pt2, color, 3, CV_AA);
    }
    return true;
}
/**
 * @brief findParall        寻找直线对，当pi-l-r<theta 时，认定为平行线，每条直线都有唯一的直线与之对应
 * @param left              左侧直线
 * @param right             右侧直线
 * @param deal_left_lines   直线对左侧直线
 * @param deal_right_lines  直线对右侧直线
 * @return                  返回为真
 */
bool findParall(vector<Vec2f> &left ,vector<Vec2f> &right , vector<Vec2f> & deal_left_lines, vector<Vec2f> & deal_right_lines,int Angle)
{
    for(size_t l = 0;l<left.size();l++)
    {
        for(size_t r = 0;r<right.size();r++)
        {
            if((CV_PI-left[l][1]-right[r][1]<Angle * CV_PI / 180)&&(CV_PI-left[l][1]-right[r][1]>-Angle * CV_PI / 180))
            {
                if(deal_left_lines[l][1]!=left[l][1])
                {
                    deal_left_lines.push_back(left[l]);
                    deal_right_lines.push_back(right[r]);
                }
                if(deal_right_lines[r][1]!=right[r][1])
                {
                    if(abs(CV_PI-left[l][1]-right[r][1])<=abs(CV_PI-left[l][1]-deal_right_lines[r][1]))
                    {
                        deal_right_lines.pop_back();
                        deal_right_lines.push_back(right[r]);
                    }
                }
            }
        }
    }
    return true;
}
