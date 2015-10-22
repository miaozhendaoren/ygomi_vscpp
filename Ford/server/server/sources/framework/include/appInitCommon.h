/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  appInitCommon.h
* @brief initialize the function and signals.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#pragma once

#include <winsock.h>
#include "typeDefine.h"
#include "database.h"
#include "databaseServer.h"
#include "RoadVecGen.h"
#include "RoadVecGen2.h"
#include "messageProcessClass.h"
#include "messageQueueClass.h"

void appInitEvents(void);
void databaseInit();
unsigned int __stdcall startSocket(void *data);
void msgQueueInit();
void viewPointInit();

#define MAX_CLIENT_NUM		10

#define SERVER_PLAY_BACK_MODE 1
#define SERVER_LOG_DIFF_MSG   0

struct portToVehi_t
{
	//u_short portId;
	//short	vehiNum;
	uint32 vehicleId;
	SOCKADDR_IN client;
};

struct sockInfo_t
{
	SOCKET sockClient;
	SOCKADDR_IN client;
};

extern list<portToVehi_t> portToVehiList;
extern messageQueueClass* messageQueue_gp;
extern messageQueueClass* databaseQueue_gp;
extern ns_database::databaseServer* database_gp;
extern ns_database::CRoadVecGen2 *roadVecGen2_gp;

extern SOCKET sockServer;			// socket
extern SOCKET sockClient;
extern SOCKADDR_IN server;     // server address
extern list<sockInfo_t> clientList;	//client address

extern HANDLE g_readySema_readDb;
extern HANDLE g_readySema_msgQueue;
extern HANDLE g_readySema_Redraw;
extern HANDLE g_clientlistMutex;