/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  roadScan.h
* @brief Road detection header file
*
* Change Log:
*      Date                Who             What
*      2015/05/20         Bingtao Gao      Create
*******************************************************************************
*/

#ifndef _ROAD_SCAN_H
#define _ROAD_SCAN_H

#include <opencv2\opencv.hpp>
#include <stdio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace std;

namespace ns_roadScan
{

#ifdef ROAD_SCAN_UT
#define DEBUG_ROAD_SCAN
#else
//#define DEBUG_ROAD_SCAN
#endif

#define SCALE 50
#define HH  28
#define BIRDVIEWROW    (10)
#define CUT 32765.0

#define GREEN	CV_RGB(0,255,0)
#define RED		CV_RGB(255,0,0)
#define BLUE	CV_RGB(0,0,255)
#define PURPLE	CV_RGB(255,0,255)
#define YELLOW	CV_RGB(255,255,0)

#define GRAYIMAGE   1
#define BINARYIMAGE 2

#define CUT 32765.0

#define PI_DIV_180		0.01745329251994329576923690768489
const int MAX_NUM_SIGN_PER_IMG = 6;
const float MIN_DISTANCE = 400.0;
//const int START_ROW = 0;
//const int END_ROW = 1800;
const float THESHOLD = 0.1;//0.35

struct landMarkInfo
{
	int    lineType;			// solid : 0 , dash : 1
	double orientation;			// Direction: angle
	Point2d  location;
	double endpoint;
	double startpoint;
	double width;
	Scalar color;
};

struct pavementInfo
{
	double width;	 
	Scalar color;
};

struct laneInfo
{
	Point  XYBLOCK;
	struct landMarkInfo leftlandMark;
	struct landMarkInfo rightlandMark;
	struct pavementInfo pavement;
};

struct laneGpsInfo
{
	int    gpsIndex;
	Point  xyBirdView;
};

struct mapBoudary
{
	int    min_x;			
	int    max_x;
	int    min_y;
	int    max_y;		
};

//struct dataEveryRowout
//{
//	Point2d Left_Middle_RelGPS;// default - (0.0,0.0) left paint relative GPS
//	bool isPaint_Left;//1 - paint / 0 - no paint
//	Point2d Left_Paint_Edge[2];//[0] left left edge,[1] left right edge; default - (0.0,0.0)
//	double Left_Area_Pixel_Mean;//default 0
//
//
//	Point2d Middle_RelGPS;// relative GPS in middle position of road
//	double Middle_Area_Pixel_Mean;// average pixel value in middle area of road  
//
//	Point2d Right_Middle_RelGPS;
//	bool isPaint_Right;
//	Point2d Right_Paint_Edge[2];
//	double Right_Area_Pixel_Mean;
//
//};

struct dataEveryRow
{
	Point leftPoint;//(-1,-1)
	Point2d Left_Middle_RelGPS;// default - (0.0,0.0) left paint relative GPS
	int isPaint_Left;//1 - paint / 0 - no paint
	Point left_Edge_XY[2];
	Point2d Left_Paint_Edge[2];//[0] left left edge,[1] left right edge; default - (0.0,0.0)
	double Left_Area_Pixel_Mean;//default 0


	Point2d Middle_RelGPS;// relative GPS in middle position of road
	double Middle_Area_Pixel_Mean;// average pixel value in middle area of road  

	Point rightPoint;
	Point2d Right_Middle_RelGPS;
	int isPaint_Right;
	Point right_Edge_XY[2];
	Point2d Right_Paint_Edge[2];
	double Right_Area_Pixel_Mean;

};

struct gpsInformationAndInterval
{
	Point2d GPS_now,GPS_next;
	int intervalOfInterception;
};

struct Parameters
{
	Point centerPoint;
	double lengthRate;

	int distanceOfSlantLeft;
	int distanceOfSlantRight;
	int distanceOfUpMove;
	int distanceOfLeft;
	int distanceOfRight;
    int ridgeThreshold;

	int stretchRate;

	int downSampling;

	double distancePerPixel;

	Point2d GPSref;

	double imageScaleHeight;
    double imageScaleWidth;

	int imageRows;
	int imageCols;

    bool discardRoadDataAfterLaneChange;

    float offsetDist;
};

struct SLineInfo
{
	double vx;
	double vy;
	Point2d WeiPoint;
	Point upoint;
	Point dpoint;
};


struct PairPoints
{
	Point up;
	Point down;
	int objindex;
	int srcindex;
};

struct landMark
{
    Point2d center;
    Point2d centerRel;
    double width;
    double hight;
    Point2d angleVec;
    int flag;
    int type;//1000.stop line; 1002.man hole; 2001-2005, arrows;
};

struct boundaryPoint
{
    Point leftPt;
    Point rightPt;
    Point topPt;
    Point downPt;
};

int roadImageGen(Mat imageIn, Mat &history, int *rowIndex, Point2d *GPS_abs, Point2d *GPS_next, 
    gpsInformationAndInterval *gpsAndInterval, int *intrtmp, Parameters inParam,Point2d &GPS_stop,bool &stopFlg);
void roadImageProc2(Mat longLane, Parameters &inParam, vector<gpsInformationAndInterval> &GPSAndInterval, vector<dataEveryRow> &roadPaintData, 
     vector<landMark> &vecLandMark);

bool readParamRoadScan(char* paramFileName, Parameters& inParam);

}
#endif