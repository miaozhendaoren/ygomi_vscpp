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
#if !defined DTECTION_H
#define DTECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

enum Shape{triangles, rectangles, octangles, circles, unknown};

struct TS_Structure
{
	int totalNumber;
	int TS_type[10];
	int TS_area[10];
	cv::Rect TS_rect[10]; 
	cv::Point2f TS_center[10];
};

class Detector{
private:	
	cv::HOGDescriptor d;	
	
	int kernel_size;
	double canny_low_Threshold;
	double canny_high_Threshold;
	
	double sw;
	
	double saturation_Threshold;
	
	int ratioT; 
	int areaT; 
public:
	Detector()
	{
		sw = 32;
		d.winSize = Size(sw, sw);
		d.blockSize = Size(sw/4, sw/4);
		d.blockStride =  Size(sw/8, sw/8);
		d.cellSize = Size(sw/8, sw/8);
		d.nbins = 8;

		kernel_size = 3;
		canny_low_Threshold = 50;
		canny_high_Threshold = 300;
		saturation_Threshold = 50;

		ratioT = 30; 
		areaT = 250; 
	}	
	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);	
	void setTitle(cv::Mat& im, const std::string label);
	void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);
	int contoursSelect(vector<vector<Point>> &contours,int *validIdx, double*validVal, double ratioT, int *maxIndex, double *maxValue); 
	 
	Mat imageFilter(const cv::Mat& image);	
	Shape ShapeDetect(std::vector<cv::Point> &approxH,InputArray &curve); 
	void CannyThreshold(Mat src_gray,Mat &dst);
	int Detector::targetClassify(cv::Mat image,InputArray& curve,const int *feat,struct svm_model *model);
	int Detector::TS_Detect_withTrack(cv::Mat image,Detector detector,TS_Structure &target,int frameNumber); 
};

#endif