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
#include <stdio.h>
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include "database.h"   // database
#include "typeDefine.h"
#include "LogInfo.h"

HANDLE g_readySema_GPS;
HANDLE g_readySema_DiffDet;

ns_database::database* database_gp;
ns_database::database* database_gpFinal;

SOCKET sockServer;
SOCKET sockClient;
SOCKADDR_IN serverAddr;
SOCKADDR_IN clientAddr;
int g_SocketLen = sizeof(clientAddr);

uint64 g_VehicleID;

void appInitEvents(void)
{
    g_readySema_GPS = CreateSemaphore(NULL,0,10,"semaphore_GPS");
    g_readySema_DiffDet = CreateSemaphore(NULL,0,10,"semaphore_DiffDet");

}

void databaseInit()
{
    database_gp = new ns_database::database("DHD/road_data_germ1.dhd");

    void* input = 0;

    database_gp->readDb(&input, ns_database::file_e);
    database_gpFinal = new ns_database::database();
    
}

void getVehicleID(uint64* vehicleID)
{
    char buff[100];
    FILE* f = fopen(".//Config//VehicleID.txt", "r");
    fscanf(f,"%s",buff);//server IP
    *vehicleID = atoi(buff);
}

int startSocket(void)
{
    int sockLen = sizeof(clientAddr);
    FILE* f = fopen(".//Config//NewcoClientIP.txt", "r");
    char buff[100];
    int nRet;
    int nPort;

    // Initialize WinSock and check version  
    WORD wVersionRequested = MAKEWORD(2,0);  
    WSADATA wsaData;   
    
    nRet = WSAStartup(wVersionRequested, &wsaData);  
    if (wsaData.wVersion != wVersionRequested)  
    {     
        fprintf(stderr,"\n Wrong version\n");  
        return 255;  
    }  
  
    // get the server IP and port number. 
    fscanf(f,"%s",buff);//server IP
    fscanf(f,"%d",&nPort);// server port
    fclose(f);
    //open the UDP socket client, connect to server side
    sockClient = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockClient == INVALID_SOCKET)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
        return 255;  
    }  

    clientAddr.sin_family           = AF_INET; 
    clientAddr.sin_addr.S_un.S_addr = inet_addr(buff);//(ULONG)&buff[0];
    clientAddr.sin_port             = htons((short)nPort);

    // server IP and port
    f = fopen(".//Config//NewcoServerIP.txt", "r");
    fscanf(f,"%s",buff);//server IP
    fscanf(f,"%d",&nPort);// server port
    fclose(f);
    serverAddr.sin_family           = AF_INET; 
    serverAddr.sin_addr.S_un.S_addr = inet_addr(buff);//INADDR_ANY
    serverAddr.sin_port             = htons((short)nPort);
    //printf("client IP is : %s, port is:%d \n",inet_ntoa(serverAddr.sin_addr),serverAddr.sin_port);
    //open the UDP socket server
    sockServer = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockServer == INVALID_SOCKET)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
        return 255;
    }

    nRet = bind(sockClient,   // Socket   
    (SOCKADDR*)&clientAddr,  // Our address  
    sizeof(struct sockaddr));// Size of address structure  
    if (nRet == SOCKET_ERROR)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Bind socket failed!");
        closesocket(sockClient);  
        return SOCKET_ERROR;  
    }
    return 0;
}
