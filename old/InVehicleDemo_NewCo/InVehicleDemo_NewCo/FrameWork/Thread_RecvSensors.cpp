/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_RecvGPS.cpp
* @brief use UDP socket server to receive the GPS data from other MAC application.
*             parser the GPS data.
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/
#include <stdio.h>
#include <process.h>
#include <Windows.h>

#include "Signal_Thread_Sync.h"
#include "NEMA_GPGGA_Proc.h"

#pragma comment(lib,"ws2_32.lib")

#define NC_UDP_GPS_DATA_BUF_LEN        (1500)

volatile SOCKET g_ServerSockUDP;
CNEMA_GPGGA_PROC gNemaGpggaProc;

char nc_udpGpsBuffer[NC_UDP_GPS_DATA_BUF_LEN];

unsigned int systemTime2Ms(SYSTEMTIME* st)
{
	unsigned int time_ms = st->wMilliseconds + st->wSecond*1000 
		+ st->wMinute*60*1000 + st->wHour*60*60*1000;
	return time_ms;
}

unsigned int __stdcall Thread_RecvSensors(void *data)
{
    int error = ~0;
    int portNum = 0;
    int numrcv;

    FILE *fp;
    error = fopen_s(&fp, "./Config/NEMA_GPS_UDP_port.txt", "rt");
	
    if(NULL != error)
    {
        return error;
    }
    else
    {
        fseek(fp, 0L, SEEK_SET);
        fscanf_s(fp, "%d", &portNum);
    }

    sockaddr_in local;
    sockaddr_in from;
    int fromlen = sizeof(from);
    
    // zero the sockaddr_in structure
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(portNum);
    local.sin_addr.s_addr = htonl(INADDR_ANY);

    //open the UDP socket server, and listening
    if((g_ServerSockUDP = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
    {
        return INVALID_SOCKET;
    }

    if( SOCKET_ERROR == bind(g_ServerSockUDP, (struct sockaddr*)&local, sizeof(local)) )
    {
        return SOCKET_ERROR;
    }
    
    while(1)
    {
        //receive GPS data in NEMA 0183 GPGGA
        numrcv = recvfrom(g_ServerSockUDP, nc_udpGpsBuffer, NC_UDP_GPS_DATA_BUF_LEN, 0, (struct sockaddr*)&from, &fromlen);
        if (numrcv != SOCKET_ERROR)
        {
			//update the receive system time.
			SYSTEMTIME sysTime;
			GetLocalTime(&sysTime);

            if(gNemaGpggaProc.NEMA_GPGGA_checksum(nc_udpGpsBuffer))
            {
				gGpsInfo.stPre = gGpsInfo.st;
				gGpsInfo.st = systemTime2Ms(&sysTime);
                gNemaGpggaProc.NEMA_GPGGA_parser(nc_udpGpsBuffer);
            }

            //printf("receive UDP data: %s\n", nc_udpGpsBuffer);
			//printf("Lat: %f\n",gGpsInfo.dLatitude);
			//printf("longi: %f\n",gGpsInfo.dLongitude);
			//printf("alti: %f\n",gGpsInfo.altitude);
			//printf("durantion is %d\n",(gGpsInfo.st - gGpsInfo.stPre));
        }
        
        //parse the GPS data.
        
        //maybe need to give other task a semaphore to notice GPS data is ready.
        ReleaseSemaphore(g_readySema_GPS, 1 ,NULL);
        
    }
    return 0;
}
