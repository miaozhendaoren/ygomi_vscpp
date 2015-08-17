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
*	   2015/02/09		  yuanzhang 	  create
*******************************************************************************
*/

#include <stdio.h>
#include <string.h>
#include "LogInfo.h"
#include "Windows.h"

char *logInfoBufP;

char logInfoBuffer[LOG_BUFFER_LENGTH];

#define LOGTOTERMINAL               0
#define LOGTOFILE                   1
#define LOGTOBOTH                   2
#define LOG_PLACE_TO                LOGTOTERMINAL     

void logPrintf(int logType, char * moduleName, char *logInfo)
{
	int strLength;
    SYSTEMTIME sys;   
    GetLocalTime( &sys );
    sprintf(logInfoBuffer, "%4d/%02d/%02d %02d:%02d:%02d, ", sys.wYear,sys.wMonth,sys.wDay,sys.wHour,sys.wMinute,sys.wSecond);
    switch(logType)
    {
    case logLevelDebug_e:
        strcat(logInfoBuffer, "DEBUG, ");
        break;
    case logLevelInfo_e:
        strcat(logInfoBuffer, "INFO, ");
        break;
    case logLevelNotice_e:
        strcat(logInfoBuffer, "NOTICE, ");
        break;
    case logLevelWarning_e:
        strcat(logInfoBuffer, "WARNING, ");
        break;
    case logLevelError_e:
        strcat(logInfoBuffer, "ERROR, ");
        break;
    case logLevelCrit_e:
        strcat(logInfoBuffer, "CRIT, ");
        break;
    case logLevelAlert_e:
        strcat(logInfoBuffer, "ALERT, ");
        break;
    case logLevelFatal_e:
        strcat(logInfoBuffer, "FATAL, ");
        break;
    case logLevelEmerg_e:
        strcat(logInfoBuffer, "EMERG, ");
        break;
    
    default:
        strcat(logInfoBuffer, "DEBUG, ");
        break;
    }
	if(0 == strlen(moduleName))
	{
		strcat(logInfoBuffer, "Module_unknown");
	}
	else
	{
		strcat(logInfoBuffer, moduleName);
	}
	strcat(logInfoBuffer, ", ");

	strLength = min(strlen(logInfo), LOG_BUFFER_LENGTH - strlen(logInfoBuffer) - 2);
	if(0 == strlen(logInfo))
	{
		strcat(logInfoBuffer, "Empty_msg\n");
	}
	else
	{
		int logInfoBufLen = strlen(logInfoBuffer);
		strncpy(logInfoBuffer + logInfoBufLen, logInfo, strLength);
		logInfoBuffer[strLength + logInfoBufLen] = '\0';
		strcat(logInfoBuffer, "\n");
	}
	
#if LOG_PLACE_TO == LOGTOTERMINAL
    printf(logInfoBuffer);
#endif
}
