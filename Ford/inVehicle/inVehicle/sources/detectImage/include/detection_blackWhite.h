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
#ifndef DTECTION_BLACK_WHITE_H
#define DTECTION_BLACK_WHITE_H

#include "Detection.h"

namespace ns_detection
{
class Detector_blackWhite : public Detector
{
private:
    // Private data
    const int _MAX_NUM_FEATURES;

    cv::HOGDescriptor _d;    
    int _kernel_size;
    double _sw;

    int   _MAX_NUM_SIGN_PER_IMG;
    double _MIN_DISTANCE;
    int   _START_ROW;
    int   _END_ROW;
    double _THESHOLD;

    svm_model *_cir_model;
    svm_model *_rec_model;
    
    std::vector<int> _feat_cir;
    std::vector<int> _feat_rec;

    std::vector<cv::Mat> _image;

    // Private methods
    int targetClassify(cv::Mat image, std::vector<int> &feat, svm_model *model);
    void fft2(cv::Mat &src, cv::Mat &dst, cv::Mat &complexI);
    void ifft2(cv::Mat &src, cv::Mat &dst) ;
    void processNoise(cv::Mat &src, cv::Mat &dst);
    double searchFixedRadusShape(cv::Mat &dxImg, cv::Mat &dyImg, cv::Mat &Sr,int radus,int nSide);
    int searchRegularPolygon(cv::Mat &src, cv::Mat &dxImg, cv::Mat &dyImg,std::vector<int> &radusVec,int nSide,TS_Structure &target, cv::Mat &dstTemp);
    cv::Mat ID2Image(int target);

public:
    Detector_blackWhite();

    void trafficSignDetect(cv::Mat image, TS_Structure &target);
};
}

#endif