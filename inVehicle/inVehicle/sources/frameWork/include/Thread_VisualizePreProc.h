/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_VisualizePreProc,h
* @brief call Digital Horizon Data Access Manager to get the look ahead data.
*             in this demo, to match to GPS module, use GPS thread semphore to trigger one time access data operation.
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/
#pragma once

#include <stdio.h>
#include <process.h>
#include "Visualization.h"
unsigned int __stdcall Thread_VisualizePreProc(void *data);
