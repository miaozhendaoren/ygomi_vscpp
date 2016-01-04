/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  AppInitCommon.h
* @brief 
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Xin Shao  	  create
*******************************************************************************
*/
#pragma once

#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include <WinSock.h>
#include "typeDefine.h"
#include "saveLinePointInSafe.h"
#include "databaseInVehicle.h" // databaseInVehicle
#include "ImageBuffer.h"
#include "roadScan.h" // readParamRoadScan
#include "detection_colored.h"    // Detector_colored
#include "detection_blackWhite.h" // Detector_blackWhite
#include "configure.h"
#include "RoadSeg.h"

#define LANE_END_LINE_FLAG (0XDEADBEEF)

extern	SOCKET sockServer;
extern	SOCKET sockClient;
extern	SOCKADDR_IN serverAddr;

extern	int g_SocketLen;
extern  uint32 g_VehicleID;
extern unsigned short g_GpsPort;
extern unsigned long g_EmulatorIP;
extern unsigned short g_EmulatorPort;

extern ns_database::databaseInVehicle* database_gp;
extern ns_historyLine::saveLinePointInSafe	historyInfoP;
extern ImageBufferAll imageBuffer;
extern std::vector<ns_roadScan::Parameters> inParamVec;
extern std::vector<cv::Mat> HVec;
extern std::vector<cv::Mat> invertHVec;
extern std::vector<cv::Mat> laneHVec;

//extern bool rightLaneFlag;
extern int g_CameraPort;
extern list<segAttributes_t> g_segCfgList;
extern ns_roadsegment::All_RoadSegment *roadSegConfig_gp;

#if(RD_SIGN_DETECT == RD_SIGN_DETECT_COLOR)
    extern std::vector<ns_detection::Detector_colored *> trafficSignDetectorVec;
#elif(RD_SIGN_DETECT == RD_SIGN_DETECT_WHITE_BLACK)
    extern std::vector<ns_detection::Detector_blackWhite *> trafficSignDetectorVec;
#endif

void trySetConnectSocket(bool flag);
void appInitEvents(void);
bool startSocket();
void getVehicleID(uint64* vehicleID);
void databaseInit();
int detectorInit();
void sendDatabaseVersion();
unsigned int __stdcall Thread_ReconnectSocket(void *data);
uint8 getLoopIdxFromFurInLoopIdx(IN int inParamIndex);
