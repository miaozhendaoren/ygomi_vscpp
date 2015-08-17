/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  lineDetection.h
* @brief Road detection header file
*
* Change Log:
*      Date                Who             What
*      2015/05/20         Bingtao Gao      Create
*******************************************************************************
*/

#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include <opencv2\opencv.hpp>
#include <stdio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace std;

#define GREEN	CV_RGB(0,255,0)
#define RED		CV_RGB(255,0,0)
#define BLUE	CV_RGB(0,0,255)
#define PURPLE	CV_RGB(255,0,255)
#define YELLOW	CV_RGB(255,255,0)

class LineDetect{
public:
	void LineDetect::initialVanishPointKF(KalmanFilter &KF,Size S);
    void LineDetect::iniLaneMarkKF(KalmanFilter &KF,Size S);
    int LineDetect::line_Detection(cv::Mat image, KalmanFilter &LaneMarkKF, KalmanFilter &KF, Mat_<float> &measurement, Size &S, Mat_<float> &measLandMark,
								int *lineIdx, double *leftW, double *rightW);
};
#endif