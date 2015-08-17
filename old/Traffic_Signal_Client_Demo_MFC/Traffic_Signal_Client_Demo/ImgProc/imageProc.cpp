/*******************************************************************************
*                           Ygomi Confidential
*                  Copyright (c) Ygomi, LLC. 1995-2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  imageProc.c
* @brief Source file for image proc
*
* Change Log:
*      Date                Who             What
*      2014/11/24         Linkun Xu       Create
*******************************************************************************
*/
#include "stdafx.h"
#include <iostream> // for standard I/O
#include <opencv2/opencv.hpp>
#include "imageProc.h"
#include <opencv2/nonfree/nonfree.hpp> // initModule_nonfree()

using namespace std;
using namespace cv;

namespace imageProc
{
#define ON 1
#define OFF 0

    void trafficSignRec::resetDefaultThresholds()
    {
        _colorExtractThresh_S = 35;
        _colorExtractThresh_V = 5;
        _colorExtractThresh_H_red_high = 15;
        _colorExtractThresh_H_red_low  = 165;
        _colorExtractThresh_H_blue_high = 123;
        _colorExtractThresh_H_blue_low  = 97;

        _noiseReduceSize = 1;

        _edgeDetectThreshLow = 500;
        _edgeDetectThreshHigh = 40000;

        _shapeRecMatchThresh = 0.1;

        _signRecDistanceThresh = 0.1;
        _signRecMatchCountThresh = 10;

        // Initialize detection and matching types
        _signRecDetectType = "SURF";
        _signRecExtractType = "SURF";
        _signRecMatchType = "BruteForce";

        // Initialize reference shapes
        _shapeOctagonV.clear();
        _shapeOctagonV.push_back(Point2f(1, 0));
        _shapeOctagonV.push_back(Point2f(2, 0));
        _shapeOctagonV.push_back(Point2f(3, 1));
        _shapeOctagonV.push_back(Point2f(3, 2));
        _shapeOctagonV.push_back(Point2f(2, 3));
        _shapeOctagonV.push_back(Point2f(1, 3));
        _shapeOctagonV.push_back(Point2f(0, 2));
        _shapeOctagonV.push_back(Point2f(0, 1));

    }

    trafficSignRec::trafficSignRec(int dbgProfileTimeFlagIn, int dbgDispImagsFlagIn)
    {
        // Initialize flags
        _dbgProfileTimeFlag = dbgProfileTimeFlagIn;
        _dbgDispImagsFlag = dbgDispImagsFlagIn;

        // Initialize thresholds
        resetDefaultThresholds();

        // Initialize image sizes
        // Input image scale size
        _inputScaleSize.cols = 640;
        _inputScaleSize.rows = 480;

        // Main output image scale size
        _outputScaleSizeV.clear();

        imgSize scaledTmp;
        scaledTmp.cols = 1024;
        scaledTmp.rows = 768;

        _outputScaleSizeV.push_back(scaledTmp);

        // Side output image scale sizes
        for(int k = 0; k < 3; k++)
        {
            scaledTmp.cols = 300;
            scaledTmp.rows = 250;

            _outputScaleSizeV.push_back(scaledTmp);
        }

        // Initialize reference features
        initModule_nonfree();
        _refStopFileLoc = "./RefImage/stop-sign-model.png";
        Mat refImage = imread(_refStopFileLoc);
        Mat refImageGray;
        cvtColor(refImage, refImageGray, CV_BGR2GRAY );
        featureDetectExtract(refImageGray, _refDescriptor);

    }

    trafficSignRec::~trafficSignRec()
    {
        _outputScaleSizeV.clear();
    }

    void trafficSignRec::setColorExtractThresh(int colorExtractThresh_SIn, 
        int colorExtractThresh_VIn, 
        int colorExtractThresh_H_red_highIn, 
        int colorExtractThresh_H_red_lowIn,
        int colorExtractThresh_H_blue_highIn,
        int colorExtractThresh_H_blue_lowIn)
    {
        _colorExtractThresh_S = colorExtractThresh_SIn;
        _colorExtractThresh_V = colorExtractThresh_VIn;
        _colorExtractThresh_H_red_high = colorExtractThresh_H_red_highIn;
        _colorExtractThresh_H_red_low  = colorExtractThresh_H_red_lowIn;
        _colorExtractThresh_H_blue_high = colorExtractThresh_H_blue_highIn;
        _colorExtractThresh_H_blue_low  = colorExtractThresh_H_blue_lowIn;
    }

    void trafficSignRec::setEdgeDetectThresh(int edgeDetectThreshLowIn, int edgeDetectThreshHighIn)
    {
        _edgeDetectThreshLow = edgeDetectThreshLowIn;
        _edgeDetectThreshHigh = edgeDetectThreshHighIn;
    }

    void trafficSignRec::setNoiseReduceThresh(int noiseReduceSizeIn)
    {
        _noiseReduceSize = noiseReduceSizeIn;
    }

    void trafficSignRec::setShapeRecThresh(double shapeRecMatchThreshIn)
    {
        _shapeRecMatchThresh = shapeRecMatchThreshIn;
    }

    void trafficSignRec::setSignRecThresh(double signRecDistanceThreshIn, int signRecMatchCountThreshIn)
    {
        _signRecDistanceThresh = signRecDistanceThreshIn;
        _signRecMatchCountThresh = signRecMatchCountThreshIn;
    }

    void trafficSignRec::featureDetectExtract(Mat& inputImg, Mat& outDescriptor)
    {
        vector<KeyPoint> keypoints;
        Ptr<FeatureDetector> mDetector = FeatureDetector::create(_signRecDetectType);
        mDetector->detect(inputImg, keypoints);

        Ptr<DescriptorExtractor> mExtractor = DescriptorExtractor::create(_signRecExtractType);
        mExtractor->compute(inputImg, keypoints, outDescriptor);
    }

    void trafficSignRec::featureMatch(Mat& descriptor1, Mat& descriptor2, vector<DMatch>& matches)
    {
        matches.clear();

        Ptr<DescriptorMatcher> mMatcher = DescriptorMatcher::create(_signRecMatchType);
        mMatcher->add(std::vector<Mat>(1, descriptor1));
        mMatcher->train();
        mMatcher->match(descriptor2, matches);
    }

    void trafficSignRec::colorExtract(Mat& inputImgHsv, Mat& imgRed, Mat& imgBlue)
    {
        int nRows = inputImgHsv.rows;
        int nCols = inputImgHsv.cols * inputImgHsv.channels();
        uchar *p, *pB, *pR;

        for( int i = 0; i < nRows; i++)
        {
            p = inputImgHsv.ptr<uchar>(i);
            pB = imgBlue.ptr<uchar>(i);
            pR = imgRed.ptr<uchar>(i);

            for( int j = 0; j < nCols; j+=3)
            {
                uchar h = p[j];
                uchar s = p[j+1];
                uchar v = p[j+2];

                if((s > _colorExtractThresh_S) && (v > _colorExtractThresh_V))
                {
                    if((h >= 0 && h <= _colorExtractThresh_H_red_high) || 
                        (h > _colorExtractThresh_H_red_low && h <= 180))
                        // Red
                    {
                        pR[j] = 255;
                        pR[j+1] = 255;
                        pR[j+2] = 255;
                    }else if(h > _colorExtractThresh_H_blue_low && h <= _colorExtractThresh_H_blue_high)
                        // Blue
                    {
                        pB[j] = 255;
                        pB[j+1] = 255;
                        pB[j+2] = 255;
                    }
                }
            }
        }
    }

    void trafficSignRec::noiseReduce(Mat& inputImg)
    {
        int element_shape = MORPH_ELLIPSE; //MORPH_ELLIPSE, MORPH_RECT or MORPH_CROSS

        Mat imgTmp;
        Mat element = getStructuringElement(element_shape, Size(_noiseReduceSize*2+1, _noiseReduceSize*2+1), Point(_noiseReduceSize, _noiseReduceSize) );

        erode(inputImg, imgTmp, element);
        dilate(imgTmp, inputImg, element);
    }

    void trafficSignRec::edgeDetect(Mat& imgBgr, vector<vector<Point> >& contours)
    {
        Mat imgGray, detected_edges;

        cvtColor( imgBgr, imgGray, CV_BGR2GRAY );

        //Canny(imgGray, imgGray, 100, 200);

        //Extract the contours
        findContours( imgGray, contours, RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        // Simplify contours
        //contours.resize(contours0.size());
        //for( size_t k = 0; k < contours0.size(); k++ )
        //    approxPolyDP(Mat(contours0[k]), contours[k], 1, true);

        // Select contour according to area
        vector<vector<Point>>::iterator itc = contours.begin();

        while(itc != contours.end())
        {
            int area = contourArea(*itc);

            if(area < _edgeDetectThreshLow || area > _edgeDetectThreshHigh)
            {
                itc = contours.erase(itc);
            }else
            {
                itc++;
            }
        }
    }

    void trafficSignRec::shapeRec(vector<vector<Point> >& contours, vector<vector<Point> >& detectedContours)
    {
        double matchRatioMax = 0;
        int detectedIdx = 0;

        for( size_t k = 0; k < contours.size(); k++ )
        {
            double matchRatio;
            matchRatio = matchShapes(_shapeOctagonV, contours[k], CV_CONTOURS_MATCH_I3, 0);

            if(matchRatio < _shapeRecMatchThresh)
            {
                detectedContours.push_back(contours[k]);
            }
        }
    }

    void trafficSignRec::signRec(Mat& inputImg, vector<vector<Point> >& detectedContours, Mat& recImg)
    {
        vector<vector<Point>>::iterator itc = detectedContours.begin();
        int k = 0;
        //recImg.clear();

        while(itc != detectedContours.end())
        {
            // Extract image containing contours
            // TODO: Could be optimized by only mask the contour area (using offset input in drawContours)
            Mat mask = Mat::zeros(inputImg.rows, inputImg.cols, CV_8UC1);
            Mat whiteBackGround(inputImg.rows, inputImg.cols, CV_8UC3);
            whiteBackGround.setTo(Scalar(255, 255, 255));

            drawContours( mask, detectedContours, k, Scalar(255), CV_FILLED, 8 );
            inputImg.copyTo(whiteBackGround, mask);

            Rect boundRect = boundingRect( Mat(*itc));
            Mat subImage = whiteBackGround(boundRect);

            // Feature match
            Mat subImageDescriptor;
            Mat subImageGray;
            vector<DMatch> matches;

            cvtColor(subImage, subImageGray, CV_BGR2GRAY );

            featureDetectExtract(subImageGray, subImageDescriptor);
            featureMatch(_refDescriptor, subImageDescriptor, matches);

            int goodMatchCount = 0;
            for(int i = 0; i < matches.size(); ++i)
            {
                if(matches[i].distance < _signRecDistanceThresh)
                    goodMatchCount++;
            }
            if(goodMatchCount < _signRecMatchCountThresh)
            {
                itc = detectedContours.erase(itc);
            }else
                // Detected
            {
                recImg = subImage.clone();

                itc++;
                k++;
            }
        }
    }

    void trafficSignRec::imageScale(Mat& inputImg, imgSize desSize, Mat& outputImg)
    {
        resize( inputImg, outputImg, Size(desSize.cols,desSize.rows) );
    }

    void trafficSignRec::setInputScale(imgSize& inputScaleIn)
    {
        _inputScaleSize.cols = inputScaleIn.cols;
        _inputScaleSize.rows = inputScaleIn.rows;
    }

    void trafficSignRec::setOutputScale(std::vector<imgSize>& outputScaleIn)
    {
        _outputScaleSizeV.clear();

        for(int k = 0; k < outputScaleIn.size(); k++)
        {
            imgSize scaledTmp;

            scaledTmp.cols = outputScaleIn[k].cols;
            scaledTmp.rows = outputScaleIn[k].rows;

            _outputScaleSizeV.push_back(scaledTmp);
        }
    }

    void trafficSignRec::imageProc(Mat& inputImg)
    {
        Mat inputImgHsv, imgGrayEdgeRed, imgGrayEdgeBlue;
        double t, tTol;

        // Initialize output vectors
        _detectedImgsV.clear();
        _detectedIdxsV.clear();

        // Initialize windows
        if(_dbgDispImagsFlag == ON)
        {
            imshow("Origin", inputImg);
        }

        if(_dbgProfileTimeFlag == ON)
        {
            t = (double)getTickCount();
            tTol = t;
        }

        // Scale input image
        Mat scaledImg;
        imageScale(inputImg, _inputScaleSize, scaledImg);

        // Convert color space to HSV
        cvtColor(scaledImg, inputImgHsv, CV_BGR2HSV );

        if(_dbgProfileTimeFlag == ON)
        {
            t = 1000*((double)getTickCount() - t)/getTickFrequency();
            cout << "Convert color space to HSV costs " << t << " milliseconds."<< endl;
            t = (double)getTickCount();
        }

        // Color detect
        Mat imgBlue = Mat::zeros( inputImg.size(), inputImg.type() );
        Mat imgRed  = Mat::zeros( inputImg.size(), inputImg.type() );

        colorExtract(inputImgHsv, imgRed, imgBlue);

        if(_dbgProfileTimeFlag == ON)
        {
            t = 1000*((double)getTickCount() - t)/getTickFrequency();
            cout << "Color detect costs " << t << " milliseconds."<< endl;
            t = (double)getTickCount();
        }

        if(_dbgDispImagsFlag == ON)
        {
            Mat imgBgrTmp;
            //cvtColor( imgRed, imgBgrTmp, CV_HSV2BGR );
            imshow("Color detect: Red", imgRed);
            //cvtColor( imgBlue, imgBgrTmp, CV_HSV2BGR );
            imshow("Color detect: Blue", imgBlue);
        }

        // Erode and dilate
        //cvtColor( imgRed, imgBgrRed, CV_HSV2BGR );
        //cvtColor( imgBlue, imgBgrBlue, CV_HSV2BGR );
        //noiseReduce(imgRed);
        //noiseReduce(imgBlue);

        if(_dbgProfileTimeFlag == ON)
        {
            t = 1000*((double)getTickCount() - t)/getTickFrequency();
            cout << "Erode and dilate costs " << t << " milliseconds."<< endl;
            t = (double)getTickCount();
        }

        if(_dbgDispImagsFlag == ON)
        {
            imshow("noiseReduced: Red", imgRed);
            imshow("noiseReduced: Blue", imgBlue);
        }

        // Edge detection and selection
        vector<vector<Point> > contoursRed;
        vector<vector<Point> > contoursBlue;

        edgeDetect(imgRed, contoursRed);
        edgeDetect(imgBlue, contoursBlue);

        if(_dbgProfileTimeFlag == ON)
        {
            t = 1000*((double)getTickCount() - t)/getTickFrequency();
            cout << "Edge detection costs " << t << " milliseconds."<< endl;
            t = (double)getTickCount();
        }

        if(_dbgDispImagsFlag == ON)
        {
            Mat imgGrayEdgeRed, imgGrayEdgeBlue;
            imgGrayEdgeRed.create( imgRed.size(), imgRed.type() );
            imgGrayEdgeBlue.create( imgBlue.size(), imgBlue.type() );
            imgGrayEdgeRed = Scalar::all(0);
            imgGrayEdgeBlue = Scalar::all(0);

            drawContours( imgGrayEdgeRed, contoursRed, -1, Scalar(255,255,255), 1 );
            drawContours( imgGrayEdgeBlue, contoursBlue, -1, Scalar(255,255,255), 1);

            imshow("Edge detection: Red", imgGrayEdgeRed);   
            imshow("Edge detection: Blue", imgGrayEdgeBlue);
        }

        // Shape recognize
        vector<vector<Point> > detectedContours;
        shapeRec(contoursRed, detectedContours);

        if(_dbgProfileTimeFlag == ON)
        {
            t = 1000*((double)getTickCount() - t)/getTickFrequency();
            cout << "Shape recognition costs " << t << " milliseconds."<< endl;
            t = (double)getTickCount();
        }

        if(_dbgDispImagsFlag == ON)
        {
            Mat imgDetected = inputImg.clone();

            drawContours( imgDetected, detectedContours, -1, Scalar(0,255,255), 4 );

            imshow("shapeRec: Red", imgDetected);
        }

        // Sign recognize
        Mat inputImgForSignRec = inputImg.clone();
        Mat recImg;
        signRec(inputImgForSignRec, detectedContours, recImg);

        // Output
        Mat imgDetected = inputImg.clone();
        Mat outImage;
        int idx = 0;
        drawContours( imgDetected, detectedContours, -1, Scalar(0,255,255), 4 );

        imageScale(imgDetected, _outputScaleSizeV[idx++], outImage);
        _detectedImgsV.push_back(outImage);

        if(!recImg.empty())
        {
            Mat outImage1, outImage2, outImage3;
            imageScale(recImg, _outputScaleSizeV[idx++], outImage1);
            _detectedImgsV.push_back(outImage1);
            _detectedIdxsV.push_back(1);

            Mat refImage = imread(_refStopFileLoc);
            imageScale(refImage, _outputScaleSizeV[idx++], outImage2);
            _detectedImgsV.push_back(outImage2);

            imageScale(imgRed, _outputScaleSizeV[idx++], outImage3);
            _detectedImgsV.push_back(outImage3);
        }

        if(_dbgProfileTimeFlag == ON)
        {
            t = 1000*((double)getTickCount() - t)/getTickFrequency();
            cout << "Sign recognition costs " << t << " milliseconds."<< endl;
            t = (double)getTickCount();
        }

        if(_dbgDispImagsFlag == ON)
        {
            Mat imgDetected = inputImg.clone();

            drawContours( imgDetected, detectedContours, -1, Scalar(0,255,255), 4 );

            imshow("signRec: Red", imgDetected);

            if(!recImg.empty())
            {
                imshow("recImg: Red", recImg);
            }else
            {
                Mat whiteBackGround(200, 200, CV_8UC3);
                whiteBackGround.setTo(Scalar(255, 255, 255));
                imshow("recImg: Red", whiteBackGround);
            }
        }

        if(_dbgProfileTimeFlag == ON)
        {
            tTol = 1000*((double)getTickCount() - tTol)/getTickFrequency();
            cout << "Total image proc costs " << tTol << " milliseconds."<< endl << endl;
        }
    }

    std::vector<Mat>* trafficSignRec::getOutputImg()
    {
        return &_detectedImgsV;
    }

    std::vector<int>* trafficSignRec::getOutputIdx()
    {
        return &_detectedIdxsV;
    }
}
