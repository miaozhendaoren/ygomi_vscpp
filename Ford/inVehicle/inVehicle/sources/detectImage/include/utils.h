#ifndef UTILS_H
#define UTILS_H
#pragma once
using namespace std;
using namespace cv;
#include "roadScan.h"

#include <list>

namespace ns_roadScan
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

class ExpMovingAverage {
private:
        double alpha; // [0;1] less = more stable, more = less stable
    double oldValue;
        bool unset;
public:
    ExpMovingAverage() {
        this->alpha = 0.2;
                unset = true;
    }

        void clear() {
                unset = true;
        }

    void add(double value) {
        if (unset) {
            oldValue = value;
                        unset = false;
        }
        double newValue = oldValue + alpha * (value - oldValue);
        oldValue = newValue;
    }

        double get() {
                return oldValue;
        }
};

void getGPSLocationOfEveryPixelInRoadScanImage(Point2d GPS1,Point2d GPS2,Point2d pixel,int scopeOfScanImage,Point2d &GPSFinal, double distancePerPixel);


#define COEFF_DD2METER (111320.0)
void coordinateChange(Point2d in,Point2d ref,Point2d &out);


void fft2(Mat &src, Mat &dst,Mat &complexI);
void ifft2(Mat &src, Mat &dst);
void processNoise(Mat &src, Mat &dst);

void getInformationOfEveryLine(gpsInformationAndInterval &GPSAndInterval,Mat &roadDraw,Mat &longLane,int lineNum,int startPoint,dataEveryRow &rowData,Parameters inParam);
void calThread(Mat &Inimg, int &hisTH,Mat &dstImage);
int calRidgePar(Mat &Inimg);
void ridgeDetect(Mat &image_f,Mat &Kapa, double sigma1, double sigma2);

void calThread(Mat &Inimg, int &hisTH, Mat&dstImage);
int calRidgePar(Mat &Inimg);

void circleDection(Mat &Inimage,vector<landMark> &landmark,int histThres);
int SurfFeatureMatch(Mat &src1,Mat &src2);
void calculateGradient(Mat &src,Mat &dxImg,Mat &dyImg);
double searchFixedRadusShape(Mat &dxImg, Mat &dyImg,Mat &Sr,int radus,int nSide);
int searchRegularPolygon(Mat &src,Mat &dxImg, Mat &dyImg,vector<int> &radusVec,int nSide,TS_Structure &target,Mat &dstTemp);


void landMarkDetection(Mat &Inimage,vector<landMark> &landmark,int histThres);
void arrowDetection(Mat &longLane,Mat &longLaneBW,vector<landMark> &arrows,Mat &laneBW);


int imageAdjust(Mat &src, Mat &dst,double low, double high, double bottom, double top, 	double gamma );
void shadowProcess(Mat &src, Mat &dst);
bool judgeShadow(Mat &src);
void findGPSInterval(vector<gpsInformationAndInterval> &gpsAndInterval,Point2d &point, Mat &inImage,Parameters inParam, Point2d &pointRel);

void linkInterval(Mat &src, Mat &dst);
int arrowClassify(Mat &inImge);
void calHAndInvertH(Parameters &inParam, Mat &H, Mat &invertH);
void blockCalRidge(Mat &matlongLane_cut, Parameters& inParam,Mat &allUnitKapa,Mat &allUnitContous,Mat &allKappBin);
void linkPaintLine(Mat allUnitContous,vector<landMark> &landmark,Mat &Tline_link_out);

void gray2BW(Mat src, Mat &dst);
void delMiddlePart(Mat &src,Mat &dst);
bool changeLaneDection(Mat &src);
void changeLaneProcess(Mat &roi,vector<gpsInformationAndInterval> &GPSAndInterval,int currentFrame,
    vector<Point2d> &gps_points,Parameters inParam,int &deleterows);
void polyFitting(Mat &longLane, Mat &fittingImg);
void linkFittingLine(Mat fittingImage,Mat &Tline_link_out);

void cvtRGB2Gray(Mat &longLaneRGB, Mat &longLaneGray);

void resetGPSOffset(double offsetDistance, Point2d &GPS_1, Point2d &GPS_2);
}

#endif