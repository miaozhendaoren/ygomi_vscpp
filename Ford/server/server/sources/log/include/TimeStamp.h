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

#include "configure.h"

#define BUFFER_MODE 0
#define BUFFER_LEN  100
#define TIMESTAMP_FILENAME  "./log/TimeStampLog.bin"

#if (ON == RD_TIMESTAMP)
#define RD_ADD_TS(funId,subId) TimeStamp((funId),(subId))
#define RD_TS_INIT()           TimeStampInit()
#define RD_TS_EXIT()           TimeStampExit()
#define RD_FLUSH_TS_BUF()      FlushTimeStampBuf()
#else
#define RD_ADD_TS(funId,subId)
#define RD_TS_INIT() 
#define RD_TS_EXIT()
#define RD_FLUSH_TS_BUF()
#endif

#if 0    //inVehicle side
typedef enum
{
    tsFunId_eIdle = 0,
    tsFunId_eThread_Update,
	tsFunId_eThread_DifRpt,
	tsFunId_eThread_Visual_Pre,
	tsFunId_eThread_ReConSocket,
	tsFunId_eThread_ImageCollect,
	tsFunId_eDataBase,
	tsFunId_eImageTimerIsr,
	tsFunId_eVisualPreTimerIsr,
    tsFunId_eMaxNum
} rd_TsFunId_eum;
#endif

#if 1    //server side 
typedef enum
{
    tsFunId_eIdle = 0,
    tsFunId_eThread_Recevie,
	tsFunId_eThread_DBUpdate,
	tsFunId_eThread_Send,
	tsFunId_eThread_Visual_Pre,
	tsFunId_eThread_AcceptSocket,
	tsFunId_eDataBase,
    tsFunId_eMaxNum
} rd_TsFunId_eum;
#endif

void TimeStampInit();
void TimeStampExit();
void TimeStamp(int func_id , int sub_id);
void FlushTimeStampBuf();