#pragma once

#include "WinSock2.h"

#define TCP_DATA_BUF_LEN   1000

extern volatile SOCKET g_ClientSockTCP;

extern UINT ThreadTCPSocketListen(LPVOID lpParam);