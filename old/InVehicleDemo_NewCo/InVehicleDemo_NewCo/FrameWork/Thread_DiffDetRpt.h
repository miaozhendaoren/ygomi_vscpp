/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_DiffDetRpt.h
* @brief get the image data from Image sensors and get current GPS information 
*             process image data and detect traffic sign and other needed inforamtion
*             call Difference Detection Unit to compare the detect information and Digital Horizon Database information
*             send differnce data to Digital Horizon Data Difference Collector(some buffer can cache?)
*             then send the difference data to Server side.
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/
#pragma once

#include <stdio.h>
#include <process.h>
#include <float.h>
#include <string>
#include <list>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include "database.h"
#include "messageProcessClass.h"
#include "AppInitCommon.h"
#include "svm.h"
#include "Detection.h"
#include "statisticsFurInfo.h"

#define USE_CAMERA 0
#define USE_VIDEO 1
#define USE_IMAGE 2
#define IMAGE_SOURCE USE_CAMERA

#define IMAGE_DISPLAY 1

#define SEND_MAX_BYTE_NUM 10000
#define IMAGE_WIDTH	800.00
#define IMAGE_HEIGHT 600.00	
#define FRAME_STATISTICS_NUM 50
#define FRAME_NUM_IN_RANG	5
#define MAX_RALIABILITY 5
#define SCREEN_WIDTH_LEFT_THRESHOLD			(2/5)
#define SCREEN_WIDTH_RIGHT_THRESHOLD	    (4/5)

using namespace std;
using namespace ns_database;

unsigned int __stdcall Thread_DiffDetRpt(void *data);
short compareFurnitureList(list<furAttributes_t>* list1,list<furAttributes_t>* list2, list<furAttributes_t>* list3);
short detectImage(int* number_of_frames,VideoCapture* capturePtr,Detector* detectorPtr,TS_Structure* targetPtr,int* detectNumberPtr);
void convertImageToFurniture(TS_Structure* targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,list<furAttributes_t>* furnListPtr);
void filterFurToReport(list<statisticsFurInfo_t>* furnListInBuffPtr, list<furAttributes_t>* furnListInDetPtr,list<furAttributes_t>* outListPtr);
short findDeleteFurnitures(TS_Structure* detImageptr,list<furAttributes_t>* list1,list<deleteFurInfo_t>* list2, list<furAttributes_t>* list3);
