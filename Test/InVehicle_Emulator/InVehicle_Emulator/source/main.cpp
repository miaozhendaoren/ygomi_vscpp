/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  main.cpp
* @brief main function
*
* Change Log:
*      Date                Who             What
*      2015/09/02         Xin Shao        Create
*******************************************************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include <string.h>

#include "typeDefine.h"
#include "messageProcessClass.h"

SOCKET sockClient;
SOCKADDR_IN serverAddr;
char dataFileName[200];
HANDLE socketMutex;
HANDLE g_readyEvent_ConnectSocket;
#define RECV_BUF_LEN  10000
uint8 recvBuf[RECV_BUF_LEN];
int timeDelay = 5000;

void appInitEvents(void)
{
    //g_readySema_GPS = CreateSemaphore(NULL,0,10,"semaphore_GPS");
    //g_readySema_DiffDet = CreateSemaphore(NULL,0,10,"semaphore_DiffDet");
	
	g_readyEvent_ConnectSocket = CreateEvent(NULL,TRUE,FALSE,NULL);

	socketMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(socketMutex);
}

bool startSocket()
{
	FILE* fp = fopen("./config/NewcoVehicleConfig.txt", "r");

	// Initialize WinSock and check version  
    WORD wVersionRequested = MAKEWORD(2,0);  
    WSADATA wsaData;   
    
    int nRet = WSAStartup(wVersionRequested, &wsaData);  
    if (wsaData.wVersion != wVersionRequested)  
    {     
        printf("COMM: The window socket version does not supoorted!");  
        return false;  
    } 

	while(!feof(fp))
	{
		char tempBuff[100];
		serverAddr.sin_family           = AF_INET;
		fscanf(fp,"%s",tempBuff);
		if(strstr(tempBuff,"serverIP:") != NULL)
		{
			serverAddr.sin_addr.S_un.S_addr = inet_addr(&tempBuff[strlen("serverIP:")]);
			continue;
		}
		int sPort;
		if(strstr(tempBuff,"serverPort:") != NULL)
		{	
			sPort = atoi(&tempBuff[strlen("serverPort:")]);
			serverAddr.sin_port = htons(sPort);
			continue;
		}
		if(strstr(tempBuff,"vehicleId:") != NULL)
		{	
			//g_VehicleID = atoi(&tempBuff[strlen("vehicleId:")]);
			continue;
		}
		if(strstr(tempBuff,"dataFile:") != NULL)
		{	
			//g_VehicleID = atoi(&tempBuff[strlen("vehicleId:")]);
			memset(dataFileName,0,200);
			strcpy(dataFileName,&tempBuff[strlen("dataFile:")]);
			continue;
		}
		if(strstr(tempBuff,"timeDelay:") != NULL)
		{
			timeDelay = 1000*atoi(&tempBuff[strlen("timeDelay:")]);
			continue;
		}
	}

	fclose(fp);

	//open the UDP socket client, connect to server side
    sockClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockClient == INVALID_SOCKET)  
    {  
        printf("COMM: Creating socket failed!\n");
        return false;  
    }

	nRet = connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr));
	if(nRet == SOCKET_ERROR)
	{
		int errorCode = WSAGetLastError();
		printf("COMM: Connect socket failed!\n");
		return false;
	}

	return true;
}

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

unsigned int __stdcall Thread_ReconnectSocket(void *data)
{
	while(1)
	{
		WaitForSingleObject(g_readyEvent_ConnectSocket, INFINITE);
		WaitForSingleObject(socketMutex,INFINITE);
		while(SOCKET_ERROR == connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr)))
		{
			int errorCode = WSAGetLastError();
			if(errorCode == 10056 || errorCode == 10038)
			{
				closesocket(sockClient);

				while(INVALID_SOCKET == socket(AF_INET, SOCK_STREAM, IPPROTO_TCP))
				{
					printf("COMM: Creating socket failed!\n");
				}
			}
			printf("COMM: Connect socket failed!\n");
			Sleep(2000);
		}

		ResetEvent(g_readyEvent_ConnectSocket);
		ReleaseMutex(socketMutex);
	}

}



diffRptMsg_t headerBuf;
uint8 payloadBuf[MAX_PAYLOAD_BYTE_NUM];

unsigned int __stdcall Thread_SendMsg(void *data)
{

	FILE *fpMsg = fopen(dataFileName,"rb");
	int readNum;
	int counter = 0;
	int nRet;


	while(1)
	{
		uint8* recvBuffP = (uint8*)&headerBuf;
		int readLen;

		//step 1: read header length
		readLen = sizeof(headerBuf.msgHeader.headerLen);
		readNum = fread(&(headerBuf.msgHeader.headerLen), readLen, 1, fpMsg);
		if(readNum != 1)
		{
			fseek(fpMsg, 0, SEEK_SET);
			counter = 0;
			continue;
		}
		
		//read header
		recvBuffP += sizeof(headerBuf.msgHeader.headerLen);
		readLen = headerBuf.msgHeader.headerLen - sizeof(headerBuf.msgHeader.headerLen);
		readNum = fread(recvBuffP, readLen, 1, fpMsg);
		if(readNum != 1)
		{
			fseek(fpMsg, 0, SEEK_SET);
			counter = 0;
			continue;
		}

		//read payload
		readLen = headerBuf.msgHeader.payloadLen;
		if(readLen != 0)
		{
			readNum = fread(payloadBuf, readLen,1,fpMsg);
			if(readNum != 1)
			{
				fseek(fpMsg, 0, SEEK_SET);
				counter = 0;
				continue;
			}
		}

		printf("message Count = %d\n",counter);
		counter++;

		//send header
		nRet = send(sockClient,(char*)&headerBuf,headerBuf.msgHeader.headerLen,0);
		if ((nRet == SOCKET_ERROR) || (nRet == 0))
		{
			trySetConnectSocket(true);
		}
		else
		{
			if((headerBuf.msgHeader.payloadLen > 0))
			{
				nRet = send(sockClient,(char*)payloadBuf,headerBuf.msgHeader.payloadLen,0);
				if ((nRet == SOCKET_ERROR) || (nRet == 0))
				{
					trySetConnectSocket(true);
				}
			}
		}

		Sleep(timeDelay);

	}

	fclose(fpMsg);

}


unsigned int __stdcall Thread_RecvMsg(void *data)
{
	while(1)
	{
		int nRet = recv(sockClient,(char*)recvBuf,RECV_BUF_LEN,0);
		if((nRet == SOCKET_ERROR)|| (nRet == 0))
		{
			trySetConnectSocket(true);
			Sleep(2000);
		}
	}
}


#pragma comment(lib, "ws2_32.lib") 

int main(int argc, char* argv[])
{
	appInitEvents();
	//start Socket
	startSocket();

	HANDLE threadHandle[3];
	threadHandle[0] = (HANDLE) _beginthreadex(0,0,&Thread_RecvMsg,0,0,0);
	threadHandle[1] = (HANDLE) _beginthreadex(0,0,&Thread_SendMsg,0,0,0);
	threadHandle[2] = (HANDLE) _beginthreadex(0,0,&Thread_ReconnectSocket,0,0,0);

	SetThreadPriority(threadHandle[0],THREAD_PRIORITY_NORMAL);
	SetThreadPriority(threadHandle[1],THREAD_PRIORITY_LOWEST);
	SetThreadPriority(threadHandle[2],THREAD_PRIORITY_NORMAL);

	WaitForMultipleObjects(3, threadHandle, true, INFINITE);

	CloseHandle(threadHandle[0]);
	CloseHandle(threadHandle[1]);
	CloseHandle(threadHandle[2]);

	return 0;

}

