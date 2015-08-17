#include "stdafx.h"
#include "afxmt.h"
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include "ThreadSocketClientTCP.h"

volatile SOCKET g_ClientSockTCP;

UINT ThreadTCPSocketListen(LPVOID lpParam)
{
	char recvData[TCP_DATA_BUF_LEN] = {0};
	for(;;)
	{
		int iRead = recv(g_ClientSockTCP, recvData, TCP_DATA_BUF_LEN, 0);
		if(iRead >0)
		{
			//do data base write process


		}
		else
		{
			if(NULL != g_ClientSockTCP)
			{
				//closesocket(g_ClientSockTCP);
			}
			return -1;
		}
	}

	return 0;
}