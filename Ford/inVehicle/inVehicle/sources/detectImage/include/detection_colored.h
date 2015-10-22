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
#ifndef DTECTION_COLORED_H
#define DTECTION_COLORED_H

#include <opencv2/core/core.hpp>
#include "svm.h"
#include "Detection.h"
#include "roadScan.h"

namespace ns_detection
{
class Detector_colored : public Detector
{
private:
    // Private data
    cv::HOGDescriptor _d;    
    
    int _kernel_size;
    double _canny_low_Threshold;
    double _canny_high_Threshold;
    
    double _sw;
    
    double _saturation_Threshold;
    
    int _ratioT; 
    int _areaT; 

    const int _MAX_NUM_FEATURES;
    const int _MAX_RED_CANDIDATES;
	const int _INVALID_TYPE;

    static const int _WRITE_IMAGE = 0;
	

#if WRITE_IMAGE == 1
    int _triImageID = 0;
    int _recImageID = 0;
    int _cirImageID = 0;
#endif

    svm_model *_cir_model;
    svm_model *_rec_model;
    svm_model *_tri_model;
    
    std::vector<int> _feat_cir;
    std::vector<int> _feat_rec;
    std::vector<int> _feat_tri;

    std::vector<cv::Mat> _image;

    // Private methods
    int contoursSelect(std::vector<std::vector<cv::Point>> &contours,int *validIdx, double*validVal, double ratioT, int *maxIndex, double *maxValue); 
    Detector::Shape ShapeDetect(std::vector<cv::Point> &approxH, cv::InputArray &curve); 
    void CannyThreshold(cv::Mat src_gray, cv::Mat &dst);
    int targetClassify(cv::Mat image,cv::InputArray& curve, std::vector<int> &feat,struct svm_model *model);
    cv::Mat ID2Image(int target);
    bool Detector_colored::acceptTrackedPoint(int i,std::vector<uchar> status,std::vector<cv::Point2f> *points);
    void Detector_colored::handleTrackedPoints(cv:: Mat &output,std::vector<cv::Point2f> *points,std::vector<cv::Point2f>initial);
    int Detector_colored::TS_classify(Detector::Shape shape,cv::Mat image,cv::InputArray curve,std::string path = "");

public:
    Detector_colored(float highStep, double dist_per_piexl,int horizon_line,int featureNum = 1500);
    void trafficSignDetect(cv::Mat image, TS_Structure &target);
    //void positionMeasure(ns_roadScan::Parameters &inParam, cv::Point2d &GPS_current, cv::Point2d &GPS_next, cv::Mat &imageIn, TS_Structure &target);
};
}

#endif