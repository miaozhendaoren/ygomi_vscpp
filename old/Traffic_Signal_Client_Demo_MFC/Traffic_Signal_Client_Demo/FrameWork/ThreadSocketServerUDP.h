#pragma once

#include "WinSock2.h"
#include "GPS_NEMA_Data.h"

#define UDP_DATA_BUF_LEN   1000

extern volatile SOCKET g_ServerSockUDP;
extern GPS_NEMA_GPGGA_Data g_GPSInfo;

extern UINT ThreadUCPSocketListen(LPVOID lpParam);