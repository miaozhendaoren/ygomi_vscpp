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
#include "typeDefine.h"
#include "saveLinePointInSafe.h"
#include "databaseInVehicle.h" // databaseInVehicle
#include "ImageBuffer.h"
#include "roadScan.h" // readParamRoadScan
#include "detection_colored.h"    // Detector_colored
#include "detection_blackWhite.h" // Detector_blackWhite
#include "configure.h"


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
extern ns_roadScan::Parameters inParam;
extern cv::Mat H;
extern cv::Mat invertH;
extern int g_CameraPort;
extern list<segAttributes_t> g_segCfgList;

#if(RD_USE_CAMERA == OFF) 
//extern char aviNames[50][100];
//extern char gpsNames[50][100];

#define  MAX_AVIGPS_FIlE_NUMBER 50
#define  MAX_FIlENAME_lENGTH	100

typedef struct
{
	char aviNames[MAX_AVIGPS_FIlE_NUMBER][MAX_FIlENAME_lENGTH];
	char gpsNames[MAX_AVIGPS_FIlE_NUMBER][MAX_FIlENAME_lENGTH];
	int numFiles;
}AviAndGpsFiles;

extern AviAndGpsFiles  g_aviAndgpsfilelist;
#endif

typedef struct
{
	SOCKADDR_IN serverAddr;
	unsigned short clientPort;
	uint32 VehicleID;
	unsigned long EmulatorIP;
	unsigned short EmulatorPort;
	unsigned short GpsPort;
}netWorkConfig;

extern netWorkConfig g_NetworkConfig;


//#if(RD_SIGN_DETECT == RD_SIGN_DETECT_COLOR)
//    extern ns_detection::Detector_colored *trafficSignDetector;
//#elif(RD_SIGN_DETECT == RD_SIGN_DETECT_WHITE_BLACK)
//	extern ns_detection::Detector_blackWhite *trafficSignDetector;
//#endif
extern ns_detection::Detector *trafficSignDetector;

void trySetConnectSocket(bool flag);
void appInitEvents(void);
bool startSocket();
void getVehicleID(uint64* vehicleID);
void databaseInit();
bool configParameterInit(void);
int detectorInit();
void sendDatabaseVersion();
unsigned int __stdcall Thread_ReconnectSocket(void *data);
