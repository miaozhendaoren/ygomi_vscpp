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
#include "appInitCommon.h"
#include "VisualizeControl.h"


SOCKET sockServer;			// socket
SOCKET sockClient = 0;
SOCKADDR_IN server;     // server address
list<sockInfo_t> clientList;	//client address

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
	database_gp = new ns_database::databaseServer();

    void* input = 0;

    database_gp->readDb(&input, ns_database::file_e);
}
void msgQueueInit()
{
	messageQueue_gp = new messageQueueClass(300);
	databaseQueue_gp = new messageQueueClass(300);

}
bool readOverViewPoint(char* fileName,eyeLookAt_t &eye)
{
	FILE* fp = fopen(fileName,"r");
	if(fp == NULL)
	{
		return false;
	}
	char tempBuff[100];
	fscanf(fp,"%s",tempBuff);
	if(std::strstr(tempBuff,"overViewPoint:") != NULL)
	{
		fscanf(fp,"%f,%f,%f",&(eye.eyePosition.x),&(eye.eyePosition.y),&(eye.eyePosition.z));
		eye.lookatPosition.x = eye.eyePosition.x;
		eye.lookatPosition.y = 0;
		eye.lookatPosition.z = eye.eyePosition.z;
	}
	fclose(fp);
	return true;
}
void viewPointInit()
{
	bool readStatus = readOverViewPoint("./config/DE_Airport_overViewPoint.txt",serverEyeInfo[0]);
	if(!readStatus)
    {
        logPrintf(logLevelError_e,"COMM","can't read the overView point!"); 
    }

}
unsigned int __stdcall startSocket(void *data)
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
	sockServer = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP); 
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
	server.sin_addr.S_un.S_addr = inet_addr(clientBuff);//inet_addr("10.69.2.49");//inet_addr(clientBuff);

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
		if(std::strstr(tempBuff,"serverIPAndPort:") != NULL)
		{	
			server.sin_addr.S_un.S_addr = inet_addr(&tempBuff[strlen("serverIPAndPort:")]);
			fgetc(fp);
			fscanf(fp,"%s",tempBuff);
			cPort = atoi(&tempBuff[0]);
			server.sin_port = htons(cPort);
		}
		if(std::strstr(tempBuff,"clientIPAndPort:") != NULL)
		{	
			client.sin_family           = AF_INET; 
			client.sin_addr.S_un.S_addr = inet_addr(&tempBuff[strlen("clientIPAndPort:")]);
			fgetc(fp);
			fscanf(fp,"%s",tempBuff);
			cPort = atoi(&tempBuff[0]);
			client.sin_port = htons(cPort);
			//clientList.push_back(client);
		}
	}
	fclose(fp);
#if 0
	sockClient = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockClient == INVALID_SOCKET)  
    {  
        logPrintf(logLevelError_e,"COMM","Create a socket for sending mesaage failed!");   
        return 255;  
    }  
#endif
	nRet = bind(sockServer,   // Socket   
        (SOCKADDR*)&server,  // Our address  
        sizeof(struct sockaddr));// Size of address structure  
    if (nRet == SOCKET_ERROR)  
    {  
        logPrintf(logLevelError_e,"COMM","Bind socket failed!");  
        closesocket(sockServer);  
        return -1;  
    }  

	nRet = listen(sockServer,5);
    if (nRet == SOCKET_ERROR)  
    {  
        logPrintf(logLevelError_e,"COMM","listen socket failed!");  
        closesocket(sockServer);  
        return -1;  
    }  

	while(1)
	{
		int length = sizeof(server);
		sockClient = accept(sockServer,(SOCKADDR*)&server,&length);
		if (sockClient == INVALID_SOCKET)  
		{  
			logPrintf(logLevelError_e,"COMM","accpet failed!");  
			return -1;  
		} 

		struct sockInfo_t clientInfo;
		clientInfo.client = server;
		clientInfo.sockClient = sockClient;
		clientList.push_back(clientInfo);

	}
	return 0;
}