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
#define NC_UDP_GPS_DATA_BUF_LEN        (1500)

extern volatile SOCKET g_ServerSockUDP;
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
    int numrcv;

    
    sockaddr_in from;
    int fromlen = sizeof(from);
    
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
