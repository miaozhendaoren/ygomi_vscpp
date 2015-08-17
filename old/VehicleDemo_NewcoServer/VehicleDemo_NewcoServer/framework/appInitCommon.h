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
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include <winsock.h>
#include <stdio.h>
#include <list>
#include <queue>
#include "typeDefine.h"
#include "database.h"
#include "messageProcessClass.h"
#include "messageQueueClass.h"

#define MAX_SIZE_OF_UPDATE_DATA_BUFF			10000
#define MSG_QUEUE_MAX_SIZE						8

#define RECV_MAX_BYTE_NUM			10000
#define SEND_MAX_BYTE_NUM			10000

using namespace std;
using namespace ns_database;

void appInitEvents(void);
void databaseInit();
int16 startSocket(void);
void msgQueueInit();


extern messageQueueClass* messageQueue_gp;
extern messageQueueClass* databaseQueue_gp;
extern ns_database::database* database_gp;

extern SOCKET sockServer;			// socket
extern SOCKET sockClient;
extern SOCKADDR_IN server;     // server address
extern SOCKADDR_IN client;	//client address

extern HANDLE g_readySema_readDb;
extern HANDLE g_readySema_msgQueue;
extern HANDLE g_readySema_Redraw;