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
#include "messageProcessClass.h"
#include "messageQueueClass.h"

void appInitEvents(void);
void databaseInit();
int16 startSocket(void);
void msgQueueInit();
void resetVehicleDatabase();
#define MAX_CLIENT_NUM		10



struct portToVehi_t
{
	//u_short portId;
	//short	vehiNum;
	uint64 vehicleId;
	SOCKADDR_IN client;
};

extern list<portToVehi_t> portToVehiList;
extern messageQueueClass* messageQueue_gp;
extern messageQueueClass* databaseQueue_gp;
extern ns_database::databaseServer* database_gp;

extern SOCKET sockServer;			// socket
extern SOCKET sockClient;
extern SOCKADDR_IN server;     // server address
extern list<SOCKADDR_IN> clientList;	//client address

extern HANDLE g_readySema_readDb;
extern HANDLE g_readySema_msgQueue;
extern HANDLE g_readySema_Redraw;