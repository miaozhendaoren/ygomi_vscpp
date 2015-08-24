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

CvPoint2D32f point_on_segment(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt);

float dist2line(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt);

void showHistogram(Mat& img);
void IPM_Conversion(Mat in, Mat &out); 
void rotate(cv::Mat& src, double angle, cv::Mat& dst);
double rad(double d);
double GPStoDst(Point2d A,Point2d B);
//Point2d getGPSLocationOfEveryPixelInRoadScanImage(Point2d,Point2d,Point,int);
void getGPSLocationOfEveryPixelInRoadScanImage(Point2d GPS1,Point2d GPS2,Point2d pixel,int scopeOfScanImage,Point2d &GPSFinal, double distancePerPixel);


#define COEFF_DD2METER (111320.0)
void coordinateChange(Point2d in,Point2d ref,Point2d &out);


void fft2(Mat &src, Mat &dst,Mat &complexI);
void ifft2(Mat &src, Mat &dst);
void processNoise(Mat &src, Mat &dst);

double rad(double d);
double GPStoDst(Point2d A,Point2d B);

void calPaintEdgePos(Mat &img, int lineNum, int curr_x, Point &left_edge, Point &right_edge);

void getPaintEdgeLocation(Mat &inputImage, Point paintPoint, Point &leftLocation, Point &rightLocation);

void getInformationOfEveryLine(gpsInformationAndInterval &GPSAndInterval,Mat &roadDraw,Mat &longLane,int lineNum,int startPoint,dataEveryRow &rowData,Parameters inParam);
void calThread(Mat &Inimg, int &hisTH,Mat &dstImage);
int calRidgePar(Mat &Inimg);
void ridgeDetect(Mat &image_f,Mat &Kapa, double sigma1, double sigma2);

void calThread(Mat &Inimg, int &hisTH, Mat&dstImage);
int calRidgePar(Mat &Inimg);

void circleDection(Mat &Inimage,vector<laneMarker> &lanemarker,int histThres);
void calculateGradient(Mat &src,Mat &dxImg,Mat &dyImg);
double searchFixedRadusShape(Mat &dxImg, Mat &dyImg,Mat &Sr,int radus,int nSide);
int searchRegularPolygon(Mat &src,Mat &dxImg, Mat &dyImg,vector<int> &radusVec,int nSide,TS_Structure &target,Mat &dstTemp);

void stopLineDetection(Mat &Inimage,vector<laneMarker> &lanemarker,int histThres);

void laneMarkerDetection(Mat &Inimage,vector<laneMarker> &lanemarker,int histThres);
double getPSNR ( const Mat& I1, const Mat& I2);

void arrowDetection(Mat &Inimage,vector<laneMarker> &lanemarker); 
int SurfFeatureMatch(Mat &src1,Mat &src2);
int imageAdjust(Mat &src, Mat &dst,   
	double low, double high,   // X方向：low and high are the intensities of src  
	double bottom, double top, // Y方向：mapped to bottom and top of dst  
	double gamma );
void shadowProcess(Mat &src, Mat &dst);
bool judgeShadow(Mat &src);
void findGPSInterval(vector<gpsInformationAndInterval> &gpsAndInterval,Point2d &point, Mat &inImage,Parameters inParam, Point2d &pointRel);

void linkInterval(Mat &src, Mat &dst);
void ridgeDetect2(Mat image_f,Mat &Kapa, double sigma1, double sigma2);
int arrowClassify(Mat &inImge);
void calHAndInvertH(Parameters &inParam, Mat &H, Mat &invertH);
void blockCalRidge(Mat &matlongLane_cut, Parameters& inParam,Mat &allUnitKapa,Mat &allUnitContous,Mat &allKappBin);
void linkPaintLine(Mat allUnitContous,vector<laneMarker> &lanemarker,Mat &Tline_link_out);
}

#endif