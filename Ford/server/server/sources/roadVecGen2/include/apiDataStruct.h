/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  apiDataStrut.h
* @brief data structures definition for merging section lanes with new data.
*
* Change Log:
*      Date                Who                 What
*      2015/08/17       Zhong Ning             Create
*      2015/09/10       Qian Xu,Shili Wang     Modify
*******************************************************************************
*/

#pragma once

#ifndef __API_DATA_STRUCT__
#define __API_DATA_STRUCT__

#include <stdio.h>
#include <windows.h>
#include <math.h>
#include <opencv2\opencv.hpp>
#include "database.h"
#include "databaseDef.h"
//#include "databaseServer.h"

using std::list;
using std::vector;

using namespace ns_database;
using namespace cv;
using namespace std;

// use this macro to control image show or saving for debug purpose
#define VISUALIZATION_ON     0
#define SAVE_DATA_ON         0

const double MaxLength = 1000.0;
const double MinLength = 0.0;

enum RESAMPLE_METHOD
{
    USE_INTERPOLATION = 0,
    USE_POLYNOMIALFIT = 1,
};

struct Scale
{
    float minXVal;
    float maxXVal;
    float minYVal;
    float maxYVal;
};

struct Offset
{
    float X;
    float Y;
};

typedef struct _rptSecData_t
{
    bool                                revDirFlag;   // 0: not reverse direction,
                                                      // 1: reverse direction
    list<vector<point3D_t>>             rptLaneData;  // multiple lanes with 2
                                                      // lines each lane
} rptSecData_t;

typedef struct _reportSectionData
{
    uint32                              sectionId;    // segment ID
    list<rptSecData_t>                  rptSecData;   // reported new lane data
} reportSectionData;

typedef struct _backgroundSectionData
{
    uint32                              sectionId;    // segment ID
    list<list<vector<point3D_t>>>       bgSectionData;// background database
                                                      // data, lanes -> lines
} backgroundSectionData;

enum seg_handle_e
{
	NOT_HANDLED = 1, 
	HANDLED = 2, 
	HANDLE_EXP
};

enum seg_report_e
{
	NOT_CURRENT_RPT = 1, 
	CURRENT_RPT = 2, 
	RPT_EXP
};

typedef struct _foregroundSectionData
{
    uint32                              sectionId;    // segmentID
	seg_report_e                        isNewRpt;
	seg_handle_e                        dealStart;
	seg_handle_e                        dealEnd;
    list<vector<point3D_t>>             fgSectionData;// foreground database
                                                      // multiple lines
} foregroundSectionData;

typedef struct _sectionConfigure
{
    double dbWidth;                                   // section Width
    double dbOverlap;                                 // section Overlap
    double dbMinLength;                               // section Minimum Length
    double dbMaxLength;                               // section Maximum Length
    double dbPaintV;                                  // value for weigh paint
    uint32 uiStepSize;                                // windows size to calculate lane length
    uint32 uiSecNum;                                  // number of sections
}sectionCon;

struct samplePoint_t
{
	uint32    segId;
	uint32    startLoc;
	point3D_t leftLane;
};

struct sampleSectionBody_t
{
	uint32 segId;
	uint32 startLoc;
	uint32 endLoc;
	bool   reverseFlag;
};

struct sampleSectionOverlap_t
{
	uint32 segId;
	vector<point3D_t> sampleOverlapData;
	uint32 startLoc;
	uint32 endLoc;
};

enum segment_type_e
{
    NORMAL_E=0,
	CROSSING_E,
	T_ROAD_E,
	CROSSING_RA_E,
	CROSSING_AR_E,
	TROAD_CROSSING_RA_E,
	TROAD_CROSSING_AR_E,
	T_ROAD_CROSS,
	DEFAULT_E
};

struct secCfgInfo_t
{
	uint32 secId;                // section identification
	segment_type_e secType; 
	vector<uint32> prevSegId;       // prev sections id
	vector<uint32> nextSegId;       // next sections id
	uint32 prePointId;
	uint32 nextPointId;
	point3D_t prePoint_ext;  
	point3D_t nextPoint_ext;
};

struct secPointInfo_t
{
	uint32 pointId;                // point identification       
	vector<uint32> connPtsId;      // connected points Id,the connPts num:{1 or 2:begin and end point of the road,2:normal,>2:crossroad & T }
	vector<uint32> connSecs;       // the corresponding sections. The order must be the same with connPts2D
};


#endif
