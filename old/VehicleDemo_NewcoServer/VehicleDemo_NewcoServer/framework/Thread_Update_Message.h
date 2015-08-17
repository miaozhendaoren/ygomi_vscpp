/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_Update_Message.h
* @brief packed the update message and send to client.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#pragma once

#include <stdio.h>
#include <process.h>

#define TIMER_NUMBER 100
#define TIMER_START_VALUE 10
#define TIMER_INTERVAL 5000		//5s timer
unsigned int __stdcall Thread_Update_Message(void *data);