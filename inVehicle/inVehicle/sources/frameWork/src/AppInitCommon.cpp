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
#include "database.h"   // database
#include "databaseInVehicle.h" // databaseInVehicle
#include "typeDefine.h"
#include "LogInfo.h"
#include "messageProcessClass.h"

HANDLE g_readySema_GPS;
HANDLE g_readySema_DiffDet;

ns_database::databaseInVehicle* database_gp;
ns_database::databaseInVehicle* database_gpFinal;

SOCKET sockServer;
SOCKET sockClient;
SOCKADDR_IN serverAddr;
volatile SOCKET g_ServerSockUDP;

int g_SocketLen;

uint64 g_VehicleID;

void appInitEvents(void)
{
    g_readySema_GPS = CreateSemaphore(NULL,0,10,"semaphore_GPS");
    g_readySema_DiffDet = CreateSemaphore(NULL,0,10,"semaphore_DiffDet");

}

void databaseInit()
{
    database_gp = new ns_database::databaseInVehicle("./resource/DHD/road_data_germ1.dhd");

    void* input = 0;

    database_gp->readDb(&input, ns_database::file_e);
    database_gpFinal = new ns_database::databaseInVehicle();
    
}

//void getVehicleID(uint64* vehicleID)
//{
//    char buff[100];
//    FILE* f = fopen("./config//VehicleID.txt", "r");
//    fscanf(f,"%s",buff);//server IP
//    *vehicleID = atoi(buff);
//}

bool startSocket(void)
{
    SOCKADDR_IN clientAddr;
	int sockLen = sizeof(clientAddr);
    FILE* fp = fopen("./config//NewcoVehicleConfig.txt", "r");   
    int nRet;
	int gpsPort;

    // Initialize WinSock and check version  
    WORD wVersionRequested = MAKEWORD(2,0);  
    WSADATA wsaData;   
    
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
	clientAddr.sin_addr.S_un.S_addr = inet_addr(clientBuff);

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
			gpsPort = atoi(&tempBuff[strlen("gpsPort:")]);
			continue;
		}
		if(std::strstr(tempBuff,"vehicleId:") != NULL)
		{	
			g_VehicleID = atoi(&tempBuff[strlen("vehicleId:")]);
			continue;
		}
	}
    //open the UDP socket client, connect to server side
    sockClient = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockClient == INVALID_SOCKET)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
        return false;  
    }  
    //printf("client IP is : %s, port is:%d \n",inet_ntoa(serverAddr.sin_addr),serverAddr.sin_port);
    //open the UDP socket server
    sockServer = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockServer == INVALID_SOCKET)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
        return false;
    }

    nRet = bind(sockClient,   // Socket   
    (SOCKADDR*)&clientAddr,  // Our address  
    sizeof(struct sockaddr));// Size of address structure  
    if (nRet == SOCKET_ERROR)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Bind socket failed!");
        closesocket(sockClient);  
        return false;  
    }

	// zero the sockaddr_in structure
	sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(gpsPort);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
	//open the UDP socket server, and listening
    if((g_ServerSockUDP = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
    {
        return false;
    }

    if( SOCKET_ERROR == bind(g_ServerSockUDP, (struct sockaddr*)&local, sizeof(local)) )
    {
        return false;
    }
    return true;
}
void sendDatabaseVersion()
{
	
	//int sendLen;
 
	#define SEND_MAX_BYTE_NUM 10000
	char sendBuff[SEND_MAX_BYTE_NUM];
	messageProcessClass statusMessage;
	diffRptMsg_t* statusRptMsgPtr = statusMessage.getDiffRptMsg();
	statusMessage.setMsgHeader((uint32*)statusRptMsgPtr,STATUS_UPDATE_RPT_MSG,g_VehicleID,highLevel_e,4,1);
	statusRptMsgPtr->payloadHeader.tlvArray[0].tag = 3;
	statusRptMsgPtr->payloadHeader.tlvArray[0].len = 2;
	statusRptMsgPtr->payloadHeader.tlvArray[0].value = 0xFFFE; // default version
	statusMessage.packedStatusRptMsg((uint32*)sendBuff);
	int sendLen =  sizeof(diffMsgHeader_t) + statusRptMsgPtr->msgHeader.numPDUs*4;
	int nRet = sendto(sockServer,(char*)sendBuff,sendLen,0,(SOCKADDR*)&serverAddr,g_SocketLen);//send response message data
	if ((nRet == SOCKET_ERROR) || (nRet == 0))
	{
		logPrintf(logLevelInfo_e, "COMM", "Send message to server failed!");
	}
	else
	{
		logPrintf(logLevelInfo_e, "COMM", "<<<< Send message to server OK");
	}
}