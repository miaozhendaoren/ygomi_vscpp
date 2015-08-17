/*******************************************************************************
*                           Ygomi Confidential
*                  Copyright (c) Ygomi, LLC. 1995-2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  imageProc.h
* @brief Header file for image proc
*
* Change Log:
*      Date                Who             What
*      2014/11/24         Linkun Xu       Create
*******************************************************************************
*/
#ifndef IMAGEPROC_H
#define IMAGEPROC_H

#include <opencv2/opencv.hpp>

namespace imageProc
{
    // Data type
    class imgSize { public: int cols; int rows;}; // use CSize instead

    class trafficSignRec
    {
    private:
        // Debug flags
        int _dbgProfileTimeFlag;
        int _dbgDispImagsFlag;

        // Thresholds for color extract
        int _colorExtractThresh_S;
        int _colorExtractThresh_V;
        int _colorExtractThresh_H_red_high;
        int _colorExtractThresh_H_red_low;
        int _colorExtractThresh_H_blue_high;
        int _colorExtractThresh_H_blue_low;

        // Thresholds for noise reduction
        int _noiseReduceSize;

        // Thresholds for edge detection
        int _edgeDetectThreshLow;
        int _edgeDetectThreshHigh;

        // Thresholds for shape recognition
        double _shapeRecMatchThresh;

        // Thresholds for sign recognition
        double _signRecDistanceThresh;
        int _signRecMatchCountThresh;

        // Initialize detection and matching types
        std::string _signRecDetectType;
        std::string _signRecExtractType;
        std::string _signRecMatchType;
        cv::Mat _refDescriptor;

        // Reference shapes
        std::string _refStopFileLoc;
        std::vector<cv::Point> _shapeOctagonV;

        // Image size parameters
        imgSize _inputScaleSize;
        std::vector<imgSize> _outputScaleSizeV;

        // Detected images and indexes
        std::vector<cv::Mat> _detectedImgsV;
        std::vector<int> _detectedIdxsV;

        // Methods
        void featureDetectExtract(cv::Mat& inputImg, cv::Mat& descriptors);
        void featureMatch(cv::Mat& descriptor1, cv::Mat& descriptor2, std::vector<cv::DMatch>& matches);
        void colorExtract(cv::Mat& inputImgHsv, cv::Mat& imgRed, cv::Mat& imgBlue);
        void noiseReduce(cv::Mat& inputImg);
        void edgeDetect(cv::Mat& imgBgr, std::vector<std::vector<cv::Point> >& contours);
        void shapeRec(std::vector<std::vector<cv::Point> >& contours, std::vector<std::vector<cv::Point> >& detectedContours);
        void signRec(cv::Mat& inputImg, std::vector<std::vector<cv::Point> >& detectedContours, cv::Mat& recImg);

    public:
        trafficSignRec(int dbgProfileTimeFlagIn, int dbgDispImagsFlagIn);
        ~trafficSignRec();

        // Image utilities
        void imageScale(cv::Mat& inputImg, imgSize desSize, cv::Mat& outputImg);

        void setInputScale(imgSize& inputScaleIn);
        void setOutputScale(std::vector<imgSize>& outputScaleIn);

        // Threshold setting functions
        void resetDefaultThresholds();

        void setColorExtractThresh(int colorExtractThresh_SIn, 
            int colorExtractThresh_VIn, 
            int colorExtractThresh_H_red_highIn, 
            int colorExtractThresh_H_red_lowIn,
            int colorExtractThresh_H_blue_highIn,
            int colorExtractThresh_H_blue_lowIn);

        void setNoiseReduceThresh(int noiseReduceSizeIn);

        void setEdgeDetectThresh(int edgeDetectThreshLow, int edgeDetectThreshHigh);

        void setShapeRecThresh(double shapeRecMatchThreshIn);

        void setSignRecThresh(double signRecDistanceThreshIn, int signRecMatchCountThreshIn);

        // Main function
        void imageProc(cv::Mat& inputImg);

        std::vector<cv::Mat>* getOutputImg();

        std::vector<int>* getOutputIdx();
    };
}

#endif
