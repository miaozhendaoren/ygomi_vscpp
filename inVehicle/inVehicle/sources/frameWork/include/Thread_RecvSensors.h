/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_RecvGPS.h
* @brief use UDP socket server to receive the GPS data from other MAC application.
*             parser the GPS data.
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/
#pragma once

#include <stdio.h>
#include <process.h>

unsigned int __stdcall Thread_RecvSensors(void *data);