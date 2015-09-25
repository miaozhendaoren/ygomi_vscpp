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

typedef struct _reportSectionData
{
    uint32                              sectionId;    // segment ID
    bool                                revDirFlag;   // 0: not reverse direction, 1: reverse direction
    list<list<list<vector<point3D_t>>>> rptSecData;   // reported new lane data
                                                      // multiple reported data
                                                      // of lanes with lines
} reportSectionData;

typedef struct _backgroundSectionData
{
    uint32                              sectionId;    // segment ID
    list<list<vector<point3D_t>>>       bgSectionData;// background database
                                                      // data, lanes -> lines
} backgroundSectionData;

typedef struct _foregroundSectionData
{
    uint32                              sectionId;    // segmentID
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

#endif
