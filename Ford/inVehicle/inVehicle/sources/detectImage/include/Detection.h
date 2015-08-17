/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  detection.h
* @brief Road furniture detection header file
*
* Change Log:
*      Date                Who             What
*      2015/1/13         Bingtao Gao      Create
*******************************************************************************
*/
#ifndef DTECTION_H
#define DTECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace ns_detection
{

class TS_Structure
{
public:
    int totalNumber;
    int TS_type[10];
    int TS_area[10];
    cv::Rect TS_rect[10]; 
    cv::Point2f TS_center[10];
};

class Detector{
private:

protected:
    enum Shape{triangles, rectangles, octangles, circles, unknown};

    const double _PI_DIV_180;
    const double _PI;

    // Common private functions
    double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
    void calculateGradient(cv::Mat &src, cv::Mat &dxImg, cv::Mat &dyImg);

    void setTitle(cv::Mat& im, const std::string label);
    void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

    void drawLine(cv::Mat image, std::vector<cv::Point> &approx, cv::Scalar color);

    void loadFeat(std::vector<int> &feat, char *fileName);

public:
    Detector();

    void trafficSignDetect(cv::Mat image, TS_Structure &target);
};

}
#endif
