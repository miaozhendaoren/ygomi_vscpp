/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  TimeStamp.h
* @brief record the CPU time
*
* Change Log:
*      Date                Who             What
*      2015/10/15         wanglei          Create
*******************************************************************************
*/
#pragma once
#define BUFFER_MODE 1
#define BUFFER_LEN  100
#define TIMESTAMP_FILENAME  "./TimeStampLog.bin"

void TimeStampInit();
void TimeStampExit();
void TimeStamp(int func_id , int sub_id);