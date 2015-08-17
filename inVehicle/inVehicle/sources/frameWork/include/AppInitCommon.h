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
extern	SOCKET sockServer;
extern	SOCKET sockClient;
extern	SOCKADDR_IN serverAddr;

extern	int g_SocketLen;
extern  uint64 g_VehicleID;

void appInitEvents(void);
bool startSocket(void);
void getVehicleID(uint64* vehicleID);
void databaseInit();
void sendDatabaseVersion();
