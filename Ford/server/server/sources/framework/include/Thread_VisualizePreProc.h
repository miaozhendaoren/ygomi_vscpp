/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_VisualizePreProc.cpp
* @brief call Digital Horizon Data Access Manager to get the look ahead data.
*             in this demo, when the Horizon data base changed, it will trigger this thread to redraw.
*
* Change Log:
*      Date                Who             What
*      2014/01/25         Xin Shao        Create
*******************************************************************************
*/
#pragma once

#include "Visualization.h"

unsigned int __stdcall Thread_VisualizePreProc(void *data);

#define MASK_BIT_LOW_24  0x0FFFFFF