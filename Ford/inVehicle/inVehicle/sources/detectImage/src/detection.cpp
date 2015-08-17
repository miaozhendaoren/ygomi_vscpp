/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  detection.cpp
* @brief Road furniture detection source file
*
* Change Log:
*      Date                Who             What
*      2015/1/13         Bingtao Gao      Create
*******************************************************************************
*/
#include "Detection.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

namespace ns_detection
{
double Detector::angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void Detector::calculateGradient(Mat &src,Mat &dxImg,Mat &dyImg)
{
    Mat srcImg(src);
    if (src.channels() > 1)
    {
        cvtColor(srcImg,srcImg,COLOR_BGR2GRAY);
        //return false;
    }

    //double startT = static_cast<double>(cv::getTickCount());

    // step 1: caculate the digradient

    // set mask
    float mask[3][3]={{1,2,1},{0,0,0},{-1,-2,-1}};
    Mat y_mask=Mat(3,3,CV_32F,mask);
    Mat x_mask=y_mask.t();

    Mat sobelX,sobelY;
    filter2D(srcImg,sobelX,CV_32F,x_mask);
    filter2D(srcImg,sobelY,CV_32F,y_mask);

    Mat gradientNorm(src.rows,src.cols, CV_32F) ;
    Mat gradientSQ = sobelX.mul(sobelX) + sobelY.mul(sobelY);
    sqrt(gradientSQ,gradientNorm);

    //namedWindow("gradientN",1);
    //imshow("gradientN",gradientNorm/100);

    Scalar meanG = mean(gradientNorm);
    
    sobelX.copyTo(dxImg);
    sobelY.copyTo(dyImg);
    
    // set the gradient to zero if below the threshold
    for(int ii = 0; ii < gradientNorm.rows;ii++)
    {
        uchar* inDataRow = gradientNorm.data + ii*gradientNorm.step; 
        uchar* dxInRow = dxImg.data + ii*dxImg.step;
        uchar* dyInRow = dyImg.data + ii*dyImg.step;

        for(int jj = 0; jj < gradientNorm.cols;jj++)
        {
            float *inData = (float*)inDataRow + jj; 
            float *dxIn = (float*)dxInRow + jj;
            float *dyIn = (float*)dyInRow + jj;

            if ( *inData  < 4*meanG[0])
            {
                *dyIn = 0;
                *dxIn = 0;
            }
        }
    }
}

/**
* Helper function to display text in the center of a contour
*/
void Detector::setTitle(cv::Mat& im, const std::string label)
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);

    cv::Point pt(0, text.height);
    cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
    cv::putText(im, label, pt, fontface, scale, CV_RGB(255,0,0), thickness, 8);
}

/**
* Helper function to display Label in the bottom of a contour
*/
void Detector::setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Rect r = cv::boundingRect(contour);

    cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + (2*(r.height + text.height) / 2));
    cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
    cv::putText(im, label, pt, fontface, scale, CV_RGB(255,0,0), thickness, 8);
}

void Detector::drawLine(cv::Mat image,std::vector<cv::Point> &approx,Scalar color)
{
    std::vector<cv::Point>::const_iterator itp= approx.begin();
    while (itp!=(approx.end()-1)) 
    {
        line(image,*itp,*(itp+1),color,2);
        ++itp;
    }
    // last point linked to first point
    cv::line(image,*(approx.begin()),*(approx.end()-1),color,2);
}

Detector::Detector():_PI_DIV_180(0.01745329251994329576923690768489), _PI(3.1415926535897932384626433832795)
{}

// Empty definition
void Detector::trafficSignDetect(Mat image, TS_Structure &target)
{
    return;
}

void Detector::loadFeat(vector<int> &feat, char *fileName)
{
    feat.clear();
    std::ifstream fin(fileName);
    char line[256]={0};
    int number;
    while(fin.getline(line, sizeof(line)))
    {
        std::stringstream numberStr(line);
        numberStr >> number;
        feat.push_back(number);
    }
    fin.clear();
    fin.close();
}
}
