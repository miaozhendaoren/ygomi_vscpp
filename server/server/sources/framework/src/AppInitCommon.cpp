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
#include <string.h>
#include "database.h"   // database
#include "databaseServer.h" // databaseServer
//#include "appInitCommon.h"
#include "LogInfo.h"
#include "messageQueueClass.h"



SOCKET sockServer;			// socket
SOCKET sockClient;
SOCKADDR_IN server;     // server address
list<SOCKADDR_IN> clientList;	//client address

//uint8 updateDataFrmDbBuff[MAX_SIZE_OF_UPDATE_DATA_BUFF];

HANDLE g_readySema_readDb;
HANDLE g_readySema_Redraw;
HANDLE g_readySema_msgQueue;

ns_database::databaseServer* database_gp;
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
	database_gp = new ns_database::databaseServer("./resource/DHD/road_data_germ1.dhd");

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

	// get host ip and port
    char hostname[256];  
    nRet=gethostname(hostname,sizeof(hostname));  
    if (nRet==SOCKET_ERROR)  
    {  
        return false;  
    }  
    HOSTENT* host=gethostbyname(hostname);  
	char clientBuff[100];
    if (host==NULL)  
    {  
        return false;  
    }
	strcpy(clientBuff,inet_ntoa(*(in_addr*)*host->h_addr_list));
	server.sin_family           = AF_INET; 
	server.sin_addr.S_un.S_addr = inet_addr(clientBuff);

	// client IP and port
	FILE *fp = fopen("./Config/NewcoServerConfig.txt", "r");
	int fIdx = 0;
	SOCKADDR_IN client;
	while(!feof(fp))
	{
		char tempBuff[100];
		fscanf(fp,"%s",tempBuff);
		if(std::strstr(tempBuff,"serverPort:") != NULL)
		{
			int nPort = atoi(&(tempBuff[strlen("serverPort:")]));
			server.sin_port             = htons((short)nPort);
			continue;
		}
		short	cPort;
		if(std::strstr(tempBuff,"clientIPAndPort:") != NULL)
		{	
			client.sin_family           = AF_INET; 
			client.sin_addr.S_un.S_addr = inet_addr(&tempBuff[strlen("clientIPAndPort:")]);
			fgetc(fp);
			fscanf(fp,"%s",tempBuff);
			cPort = atoi(&tempBuff[0]);
			client.sin_port = htons(cPort);
			clientList.push_back(client);
		}
	}

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
void resetVehicleDatabase()
// reset vehicle's database 
{
	messageProcessClass statusMessage;
	diffRptMsg_t* statusRptMsgPtr = statusMessage.getDiffRptMsg();
	statusMessage.setMsgHeader((uint32*)statusRptMsgPtr,STATUS_UPDATE_RPT_MSG,0,highLevel_e,4,1);
	statusRptMsgPtr->payloadHeader.tlvArray[0].tag = 1;
	statusRptMsgPtr->payloadHeader.tlvArray[0].len = 2;
	statusRptMsgPtr->payloadHeader.tlvArray[0].value = 3;
	messageQueue_gp->push(&statusMessage);
	ReleaseSemaphore(g_readySema_msgQueue,1,NULL);
}