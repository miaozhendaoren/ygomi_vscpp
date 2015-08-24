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

#include <vector>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "database.h"
#include "databaseInVehicle.h"
#include "roadScan.h"

namespace ns_detection
{
class TS_Structure
{
    public: 
        
    struct TS_element{
        int type;
        int area;
        cv::Rect rect;
        cv::Point2f center;
        std::vector<float> offset;
        std::vector<ns_database::point3D_t> position;
    };
    std::vector<TS_element> trafficSign;
};

class Detector{
private:

protected:
    enum Shape{triangles, rectangles, octangles, circles, unknown};

    const double _PI_DIV_180;
    const double _PI;
    const int _OFFSET_NUMBER;
    const double _DIST_PER_PIEXL;
    const int _HORIZON_LINE_PIEXL;

    // Common private functions
    double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
    void calculateGradient(cv::Mat &src, cv::Mat &dxImg, cv::Mat &dyImg);

    void setTitle(cv::Mat& im, const std::string label);
    void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

    void drawLine(cv::Mat image, std::vector<cv::Point> &approx, cv::Scalar color);

    void loadFeat(std::vector<int> &feat, char *fileName);

public:
    std::vector<float> offset;
    Detector(float highStep, double dist_per_piexl, int horizon_line);
    virtual void trafficSignDetect(cv::Mat image, TS_Structure &target) = 0;
    void positionMeasure(ns_roadScan::Parameters &inParam, cv::Point2d &GPS_current, cv::Point2d &GPS_next, cv::Mat &imageIn, TS_Structure &target);
};

}
#endif
