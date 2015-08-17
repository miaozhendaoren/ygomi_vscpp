/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  AppInitCommon.cpp
* @brief 
*
* Change Log:
*      Date                Who             What
*       2015/01/09          Xin Shao        create
*******************************************************************************
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include "AppInitCommon.h"
#include "database.h"   // database
#include "databaseInVehicle.h" // databaseInVehicle
#include "typeDefine.h"
#include "LogInfo.h"
#include "messageProcessClass.h"
#include "saveLinePointInSafe.h"
#include "ImageBuffer.h"
#include "roadScan.h" // readParamRoadScan

HANDLE g_readySema_GPS;
HANDLE g_readySema_DiffDet;
HANDLE g_readyEvent_ConnectSocket;

ns_database::databaseInVehicle* database_gp;
ns_historyLine::saveLinePointInSafe	historyInfoP(5,500);
ImageBufferAll imageBuffer;

SOCKET sockServer;
SOCKET sockClient;
SOCKADDR_IN serverAddr;
volatile SOCKET g_ServerSockUDP;

int g_SocketLen;

uint32 g_VehicleID;
unsigned short g_GpsPort;
unsigned long g_EmulatorIP;
unsigned short g_EmulatorPort;
HANDLE socketMutex;

ns_roadScan::Parameters inParam;

void trySetConnectSocket(bool flag)
{
	DWORD dwWaitResult = WaitForSingleObject(socketMutex,0);
	if (dwWaitResult != WAIT_OBJECT_0 && dwWaitResult != WAIT_TIMEOUT)
	{

	}else
	{
		//ReleaseSemaphore(g_readySema_ConnectSocket, 1 ,NULL);
		SetEvent(g_readyEvent_ConnectSocket);
		ReleaseMutex(socketMutex);
	}
}

void appInitEvents(void)
{
    g_readySema_GPS = CreateSemaphore(NULL,0,10,"semaphore_GPS");
    g_readySema_DiffDet = CreateSemaphore(NULL,0,10,"semaphore_DiffDet");
	
	g_readyEvent_ConnectSocket = CreateEvent(NULL,TRUE,FALSE,NULL);

	socketMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(socketMutex);
}

void databaseInit()
{
    database_gp = new ns_database::databaseInVehicle();

    void* input = 0;

    database_gp->readDb(&input, ns_database::file_e);
    
}

int detectorInit()
{
    bool readStatus = ns_roadScan::readParamRoadScan("./config//US.txt", inParam);
    if(!readStatus)
    {
        return -1;
    }else
    {
        return 0;
    }
}

//void getVehicleID(uint64* vehicleID)
//{
//    char buff[100];
//    FILE* f = fopen("./config//VehicleID.txt", "r");
//    fscanf(f,"%s",buff);//server IP
//    *vehicleID = atoi(buff);
//}

bool startSocket()
{
    SOCKADDR_IN clientAddr;
	int sockLen = sizeof(clientAddr);
    FILE* fp = fopen("./config//NewcoVehicleConfig.txt", "r");   
    int nRet;

    // Initialize WinSock and check version  
    WORD wVersionRequested = MAKEWORD(2,0);  
    WSADATA wsaData;   
    //logPrintf(logLevelError_e,"COMM","shaoxin");
    nRet = WSAStartup(wVersionRequested, &wsaData);  
    if (wsaData.wVersion != wVersionRequested)  
    {     
        logPrintf(logLevelError_e,"COMM","The window socket version does not supoorted!");  
        return false;  
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
	clientAddr.sin_family           = AF_INET;
	clientAddr.sin_addr.S_un.S_addr = inet_addr(clientBuff);//inet_addr("127.0.0.1");

	g_SocketLen = sizeof(clientAddr);
	// get the serverIP and port, GPS port and vehicle ID
	while(!feof(fp))
	{
		char tempBuff[100];
		serverAddr.sin_family           = AF_INET;
		fscanf(fp,"%s",tempBuff);
		if(std::strstr(tempBuff,"serverIP:") != NULL)
		{
			serverAddr.sin_addr.S_un.S_addr = inet_addr(&tempBuff[strlen("serverIP:")]);
			continue;
		}
		int sPort;
		if(std::strstr(tempBuff,"serverPort:") != NULL)
		{	
			sPort = atoi(&tempBuff[strlen("serverPort:")]);
			serverAddr.sin_port = htons(sPort);
			continue;
		}
		int	cPort;
		if(std::strstr(tempBuff,"clientPort:") != NULL)
		{	
			cPort = atoi(&tempBuff[strlen("clientPort:")]);
			clientAddr.sin_port = htons(cPort);
			continue;
		}
		if(std::strstr(tempBuff,"gpsPort:") != NULL)
		{	
			cPort = atoi(&tempBuff[strlen("gpsPort:")]);
			g_GpsPort = htons(cPort);
			continue;
		}
		if(std::strstr(tempBuff,"vehicleId:") != NULL)
		{	
			g_VehicleID = atoi(&tempBuff[strlen("vehicleId:")]);
			continue;
		}
		if(std::strstr(tempBuff,"emulatorIP:") != NULL)
		{
			g_EmulatorIP = inet_addr(&tempBuff[strlen("emulatorIP:")]);
		}
		if(std::strstr(tempBuff,"emulatorPort:") != NULL)
		{
			 cPort = atoi(&tempBuff[strlen("emulatorPort:")]);
			 g_EmulatorPort = htons(cPort);
		}
	}

    //open the UDP socket client, connect to server side
    sockClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockClient == INVALID_SOCKET)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
        return false;  
    }  

	//nRet = bind(sockClient,(SOCKADDR*)&clientAddr,sizeof(struct sockaddr));
	//if (nRet == SOCKET_ERROR)  
    //{  
	//	int errorCode = WSAGetLastError();
    //    logPrintf(logLevelError_e,"COMM","Bind socket failed!");   
    //    return -1;  
    //} 


	nRet = connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr));
	if(nRet == SOCKET_ERROR)
	{
		int errorCode = WSAGetLastError();
		logPrintf(logLevelFatal_e, "COMM", "Connect socket failed!");
		//closesocket(sockClient);
		//WSACleanup();
		return false;
	}
    return true;

}

unsigned int __stdcall Thread_ReconnectSocket(void *data)
{
	while(1)
	{
		WaitForSingleObject(g_readyEvent_ConnectSocket, INFINITE);
		WaitForSingleObject(socketMutex,INFINITE);
		while(SOCKET_ERROR == connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr)))
		{
			int errorCode = WSAGetLastError();
			if(errorCode == 10056)
			{
				closesocket(sockClient);

				while(INVALID_SOCKET == socket(AF_INET, SOCK_STREAM, IPPROTO_TCP))
				{
					logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
				}
			}
			logPrintf(logLevelFatal_e, "COMM", "Connect socket failed!");
			Sleep(50);
		}

		ResetEvent(g_readyEvent_ConnectSocket);
		ReleaseMutex(socketMutex);
	}

}

void sendDatabaseVersion()
{
	
	//int sendLen;
 
	#define SEND_MAX_BYTE_NUM 10000
	char sendBuff[SEND_MAX_BYTE_NUM];
	messageProcessClass statusMessage;
	diffRptMsg_t* statusRptMsgPtr = statusMessage.getDiffRptMsg();
	uint16 headerLen = sizeof(statusRptMsgPtr->msgHeader) + sizeof(statusRptMsgPtr->payloadHeader.pduHeader[0]);
	statusMessage.setMsgHeader((uint32*)statusRptMsgPtr,STATUS_UPDATE_RPT_MSG,g_VehicleID,highLevel_e,0,1,headerLen);
	statusRptMsgPtr->payloadHeader.tlvArray[0].tag = 3;
	statusRptMsgPtr->payloadHeader.tlvArray[0].len = 2;
	statusRptMsgPtr->payloadHeader.tlvArray[0].value = 0xfffe;
	
	int nRet = send(sockClient,(char*)statusRptMsgPtr,headerLen,0);//,(SOCKADDR*)&serverAddr,g_SocketLen);//send response message data
	if ((nRet == SOCKET_ERROR) || (nRet == 0))
	{
		logPrintf(logLevelInfo_e, "COMM", "Send message to server failed!");
	}
	else
	{
		logPrintf(logLevelInfo_e, "COMM", "<<<< Send message to server OK");
	}
}