#include "stdafx.h"
#include "afxmt.h"
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include "ThreadSocketServerUDP.h"
#include "GPS_NEMA_Data.h"

volatile SOCKET g_ServerSockUDP;
GPS_NEMA_GPGGA_Data g_GPSInfo;

UINT ThreadUCPSocketListen(LPVOID lpParam)
{
    char recvData[UDP_DATA_BUF_LEN];
    sockaddr_in m_clientAddr;
    int size = sizeof(sockaddr_in);
    
    for(;;)
    {
		memset(recvData, 0, UDP_DATA_BUF_LEN);
        int iRead = recvfrom(g_ServerSockUDP, recvData, UDP_DATA_BUF_LEN, 0, (struct sockaddr*)&m_clientAddr,&size);
        if(iRead >0)
		{
			//GPS signal parser
            //client IP         //char* gcClientIP = inet_ntoa(m_clientAddr.sin_addr);
            //client port     //m_clientAddr.sin_port;
            
			//CString strData = recvData;
            //int checksum = 0;
			//for(int idx =1; idx <= 66; idx++)
			//{
			//	checksum ^= recvData[idx];
			//}
			CString strData;
			strData.Format("%s", recvData);

			BOOL flag = g_GPSInfo.dataProc(strData);

		}
		else
		{
			if(NULL != g_ServerSockUDP)
			{
				//closesocket(g_ClientSockTCP);
			}
			return -1;
		}
    }
}