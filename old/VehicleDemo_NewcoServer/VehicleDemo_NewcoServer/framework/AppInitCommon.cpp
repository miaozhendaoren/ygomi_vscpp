/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  appInitCommon.cpp
* @brief initialize the function and signals.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#include "database.h"   // database
#include <list>
#include "appInitCommon.h"
#include "LogInfo.h"


SOCKET sockServer;			// socket
SOCKET sockClient;
SOCKADDR_IN server;     // server address
SOCKADDR_IN client;	//client address

//uint8 updateDataFrmDbBuff[MAX_SIZE_OF_UPDATE_DATA_BUFF];

HANDLE g_readySema_readDb;
HANDLE g_readySema_Redraw;
HANDLE g_readySema_msgQueue;

using namespace ns_database;

ns_database::database* database_gp;
messageQueueClass* messageQueue_gp;
messageQueueClass* databaseQueue_gp;

void appInitEvents(void)
{
	g_readySema_readDb = CreateSemaphore(NULL,0,10,NULL);
	g_readySema_Redraw = CreateSemaphore(NULL,0,10,NULL);
	g_readySema_msgQueue = CreateSemaphore(NULL,0,10,NULL);

}
void databaseInit()
{
	database_gp = new ns_database::database("DHD/road_data_germ1.dhd");

    void* input = 0;

    database_gp->readDb(&input, ns_database::file_e);
}
void msgQueueInit()
{
	messageQueue_gp = new messageQueueClass(8);
	databaseQueue_gp = new messageQueueClass(8);

}
int16 startSocket(void)
{
	// Initialize WinSock and check version  
	WORD wVersionRequested = MAKEWORD(1,1);  
    WSADATA wsaData; 
 
	FILE* f = fopen(".//Config//NewcoServerIP.txt", "r");
	char buff[100];
	int nPort;

    
    int nRet = WSAStartup(wVersionRequested, &wsaData); 

    if (wsaData.wVersion != wVersionRequested)  
    {     
        logPrintf(logLevelError_e,"COMM","The window socket version does not supoorted!"); 
		return -1;
    } 
	// start a socket to process the client message
	sockServer = socket(AF_INET,SOCK_DGRAM,0); 
    if (sockServer == INVALID_SOCKET)  
    {  
        logPrintf(logLevelError_e,"COMM","create a socket for receiving message failed!");  
        return -1;  
    } 
	//  server IP and port number. 
	fscanf(f,"%s",buff);//server IP
	fscanf(f,"%d",&nPort);// server port
	fclose(f);
	server.sin_family           = AF_INET; 
	server.sin_addr.S_un.S_addr = inet_addr(buff);
	server.sin_port             = htons((short)nPort);
	// client IP and port
	f = fopen(".//Config//NewcoClientIP.txt", "r");
	fscanf(f,"%s",buff);//server IP
	fscanf(f,"%d",&nPort);// server port
	fclose(f);
	//client port
	client.sin_family           = AF_INET; 
	client.sin_addr.S_un.S_addr = inet_addr(buff);//INADDR_ANY
	client.sin_port             = htons((short)nPort);

	sockClient = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockClient == INVALID_SOCKET)  
    {  
        logPrintf(logLevelError_e,"COMM","Create a socket for sending mesaage failed!");   
        return 255;  
    }  

	nRet = bind(sockServer,   // Socket   
        (SOCKADDR*)&server,  // Our address  
        sizeof(struct sockaddr));// Size of address structure  
    if (nRet == SOCKET_ERROR)  
    {  
        logPrintf(logLevelError_e,"COMM","Bind socket failed!");  
        closesocket(sockServer);  
        return -1;  
    }  
	return 0;
}