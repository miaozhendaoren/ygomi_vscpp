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

//#include <iostream>
//#include <sstream>
//#include <fstream>
//#include <string>
#include "markLocate.h"
#include "AppInitCommon.h" //invertH

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

Detector::Detector(float highStep,double dist_per_piexl,int horizon_line):_DIST_PER_PIEXL(dist_per_piexl),_HORIZON_LINE_PIEXL(horizon_line),
													_PI_DIV_180(0.01745329251994329576923690768489), _PI(3.1415926535897932384626433832795), _OFFSET_NUMBER(100)
{
    //float stepSize[] = {0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12, 12.5, 13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17, 17.5, 18, 18.5, 19, 19.5, 20, 20.5, 21, 21.5, 22, 22.5, 23, 23.5, 24, 24.5, 25, 25.5, 26, 26.5, 27, 27.5, 28, 28.5, 29, 29.5, 30};
    
    for(int ii = 0; ii < _OFFSET_NUMBER; ++ii)
    {
        offset.push_back(highStep*(ii+1));
    }
}

// Empty definition
//void Detector::trafficSignDetect(Mat image, TS_Structure &target)
//{
//    return;
//}

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

void Detector::positionMeasure(Parameters &inParam, Point2d &GPS_current, Point2d &GPS_next, Mat &imageIn, TS_Structure &target, Mat &invertH)
{
    int numSigns = target.trafficSign.size();
    Point2d &GPS_reference = inParam.GPSref;

    Mat imageOut;
#if (RD_LOCATION == RD_GERMAN_LEHRE)
    int imageWidthTemp = imageIn.cols;
    int imageHeightTemp = imageIn.rows;
    inParam.imageScaleWidth = 1.0;
    inParam.imageScaleHeight = 1.0;
#else
    int imageWidthTemp = imageIn.cols * inParam.imageScaleWidth;
    int imageHeightTemp = imageIn.rows * inParam.imageScaleHeight;

    resize(imageIn,imageOut,Size(imageWidthTemp,imageHeightTemp));
#endif
    

    for(int idx = 0;  idx < numSigns; idx++)
    {
        // try all the stepSize 
        vector<Point> birdViewPoints;
        for(int index = 0; index < _OFFSET_NUMBER ;index++)
        {
            // calculate the pixel 
            float stepSize = offset[index];
            float signHeight = stepSize*target.trafficSign[idx].rect.height;
            int xPixel = (int)(target.trafficSign[idx].center.x * inParam.imageScaleWidth + 0.5);// add 0.5 pixel for round to 1 pixel
            int yPixel = (int)((target.trafficSign[idx].center.y + target.trafficSign[idx].rect.height * 0.5 + signHeight) * inParam.imageScaleHeight + 0.5); // the first 0.5 means the half height of traffic sign.
            Point buttomPixel = Point(xPixel,yPixel);

            //circle(imageOut,buttomPixel,2,Scalar(0,0,255),1);

            // if the pixel is out of the image, stop process
            if(yPixel > imageHeightTemp)
            {
                break;
            }
            // if the pixel is above the horizontal line ,skip it.
            //if(yPixel < _HORIZON_LINE_PIEXL) // FIXME:230 
            //{
            //    continue;
            //}

            Point pixelLocationBirdView;
            Point2d refGPSOriginalImage;

            // change the image to bird view 
            determineBirdViewLocation(invertH, buttomPixel, pixelLocationBirdView);

            // change the bird view pixel to relative location.
            getRefGPSLocationOfEveryPixelInRoadScanImage(imageOut, inParam.stretchRate, GPS_current, GPS_next, GPS_reference, pixelLocationBirdView, _DIST_PER_PIEXL, inParam.offsetDist, refGPSOriginalImage);

            //cal absolute GPS of original image pixel from refGPS and GPS_reference, if need
            ns_database::point3D_t refGPSOriginalImage3d;

            refGPSOriginalImage3d.lat = refGPSOriginalImage.x;
            refGPSOriginalImage3d.lon = refGPSOriginalImage.y;

            target.trafficSign[idx].position.push_back(refGPSOriginalImage3d);
            target.trafficSign[idx].offset.push_back(stepSize);
        }


#ifdef TRAFFIC_SIGN_TEST
		namedWindow("imagePositon");
        imshow("iamgePositoin",imageOut);
        static int ID = 0;
        char fileName[100];
        sprintf_s( fileName,100, "D:/Newco/airport_Code/Demo/Ford/inVehicle/Release/%05d.jpg",ID++);

        imwrite(fileName,imageOut);
        waitKey(30);
#endif
    }
}

}
