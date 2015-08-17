/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  LogInfo.c
* @brief 
*
* Change Log:
*      Date                Who             What
*       2015/02/09          yuanzhang       create
*******************************************************************************
*/

#include "LogInfo.h"

#include <stdio.h>    // sprintf
#include <iostream>   // clog
#include <Windows.h>  // FOREGROUND_INTENSITY

using std::cerr;
using std::clog;
using std::endl;
using std::string;

#define LOGTOTERMINAL               0
#define LOGTOFILE                   1
#define LOGTOBOTH                   2
#define LOG_PLACE_TO                LOGTOTERMINAL     

void logPrintf(logInfoLevel_e logType, string moduleName, string logInfo, int colorValue)
{
    SYSTEMTIME sys;   
    GetLocalTime( &sys );

    char logInfoBuffer[30];
    sprintf(logInfoBuffer, "%4d/%02d/%02d %02d:%02d:%02d, ", sys.wYear,sys.wMonth,sys.wDay,sys.wHour,sys.wMinute,sys.wSecond);

    string logString = logInfoBuffer;

    switch(logType)
    {
    case logLevelDebug_e:
        logString += "DEBUG, ";
        break;
    case logLevelInfo_e:
        logString += "INFO, ";
        break;
    case logLevelNotice_e:
        logString += "NOTICE, ";
        break;
    case logLevelWarning_e:
        logString += "WARNING, ";
        break;
    case logLevelError_e:
        logString += "ERROR, ";
        break;
    case logLevelCrit_e:
        logString += "CRIT, ";
        break;
    case logLevelAlert_e:
        logString += "ALERT, ";
        break;
    case logLevelFatal_e:
        logString += "FATAL, ";
        break;
    case logLevelEmerg_e:
        logString += "EMERG, ";
        break;
    
    default:
        logString += "DEBUG, ";
        break;
    }

    if(0 == moduleName.size())
    {
        logString += "Module_unknown";
    }
    else
    {
        logString += moduleName;
    }
    logString += ", ";

    if(0 == logInfo.size())
    {
        logString += "Empty_msg";
    }
    else
    {
        logString += logInfo;
    }
    
#if LOG_PLACE_TO == LOGTOTERMINAL
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY | colorValue);
    clog << logString.c_str() << endl;
#endif
}
