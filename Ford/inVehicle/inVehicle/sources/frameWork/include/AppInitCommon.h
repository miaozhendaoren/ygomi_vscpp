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

extern	SOCKET sockServer;
extern	SOCKET sockClient;
extern	SOCKADDR_IN serverAddr;
extern volatile SOCKET g_ServerSockUDP;

extern	int g_SocketLen;
extern  uint32 g_VehicleID;
extern unsigned short g_GpsPort;
extern unsigned long g_EmulatorIP;
extern unsigned short g_EmulatorPort;

extern ns_database::databaseInVehicle* database_gp;
extern ns_historyLine::saveLinePointInSafe	historyInfoP;
extern ImageBufferAll imageBuffer;
extern ns_roadScan::Parameters inParam;

void trySetConnectSocket(bool flag);
void appInitEvents(void);
bool startSocket();
void getVehicleID(uint64* vehicleID);
void databaseInit();
int detectorInit();
void sendDatabaseVersion();
unsigned int __stdcall Thread_ReconnectSocket(void *data);
