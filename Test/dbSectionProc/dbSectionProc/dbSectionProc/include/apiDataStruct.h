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
*      Date                Who             What
*      2015/08/17       Zhong Ning        Create
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
#include "databaseServer.h"

using std::list;
using std::vector;

using namespace ns_database;
using namespace cv;

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


#endif
