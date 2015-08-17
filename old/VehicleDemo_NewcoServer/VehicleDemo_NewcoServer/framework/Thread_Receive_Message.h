/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_Receive_Message.h
* @brief this thread to receive the messages from socket.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#pragma once

#include <stdio.h>
#include <process.h>

unsigned int __stdcall Thread_Receive_Message(void *data);