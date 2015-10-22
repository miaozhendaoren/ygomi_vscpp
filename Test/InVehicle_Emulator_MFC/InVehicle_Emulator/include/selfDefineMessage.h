/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  selfDefineMessage.h
* @brief Header file for some message for the worker process inform UI process
*
* Change Log:
*      Date                Who             What
*	   2015/10/09		  qiu heng		  create
*******************************************************************************
*/
#pragma once

#define WM_DISPLAYCOUNT  WM_USER+2
#define WM_DISPLAYSTATUS WM_USER+3

//status  message define
#define	STATUS_CONNECT_SOCKET_SUCCESS  0
#define	STATUS_CREATE_SOCKET_FAIL  1
#define	STATUS_CONNECT_SOCKET_FAIL  2


struct ThreadProcInfo
{
     HWND hWnd;  
};