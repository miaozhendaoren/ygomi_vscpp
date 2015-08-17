/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  LogInfo.h
* @brief 
*
* Change Log:
*      Date                Who             What
*	   2015/02/09		  yuanzhang 	  create
*******************************************************************************
*/
#pragma once

#include <iostream>   // clog
#include <Windows.h>  // FOREGROUND_RED

enum logInfoLevel_e
{
    logLevelDebug_e,
    logLevelInfo_e,
    logLevelNotice_e,
    logLevelWarning_e,
    logLevelError_e,
    logLevelCrit_e,
    logLevelAlert_e,
    logLevelFatal_e, 
    logLevelEmerg_e,
};

void logPrintf(
    logInfoLevel_e logType, 
    std::string moduleName, 
    std::string logInfo, 
    int colorValue = (FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE));
