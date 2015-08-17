/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_ImageSensorCollector.cpp
* @brief get the image data from Image sensors and get current GPS information 
*        add circle buffer to save continuous image frame, it is depend on ImageSensor's configure
*
* Change Log:
*      Date                Who             What
*      2015/06/01         Xin Shao        Create
*******************************************************************************
*/
#pragma once
#include "ImageBuffer.h"

unsigned int __stdcall Thread_ImageSensorCollector(void *data);