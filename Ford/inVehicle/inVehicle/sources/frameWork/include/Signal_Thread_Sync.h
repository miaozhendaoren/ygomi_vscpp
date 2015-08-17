/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Signal_Thread_Sync.h
* @brief 
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Xin Shao  	  create
*******************************************************************************
*/
#pragma once

#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file

extern HANDLE g_readySema_GPS;
extern HANDLE g_readySema_DiffDet;  //semaphore to control Difference detect and report thread.