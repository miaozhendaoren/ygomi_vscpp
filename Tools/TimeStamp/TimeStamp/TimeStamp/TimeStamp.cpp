/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  TimeStamp.cpp
* @brief record the CPU time
*
* Change Log:
*      Date                Who             What
*      2015/10/15         wanglei          Create
*******************************************************************************
*/
#include <iostream>   
#include <Windows.h> 
#include "TimeStamp.h"
#if (BUFFER_MODE)
    unsigned int TimeStampBuf[BUFFER_LEN];
    int static bufindex=0;
#endif
int static DayOfMonth[12]={0,31,59,90,120,151,181,212,243,273,304,334};//year 2015:31,28,31,30,31,30,31,31,30,31,30,31
__inout CRITICAL_SECTION cs;

void change(unsigned int *data)
{
    unsigned int tmp=0;
    tmp = ((*data)&0xff)<<24;
    tmp |= ((*data)&0xff00)<<8;
    tmp |= ((*data)&0xff0000)>>8;
    tmp |= ((*data)&0xff000000)>>24;
    *data = tmp;
}

void TimeStampInit()
{
    FILE* fp;
    char s[]= TIMESTAMP_FILENAME;
    fp = fopen(s,"wb+");
    fclose(fp);

    InitializeCriticalSection(&cs);

#if (BUFFER_MODE)
    bufindex = 0;
#endif

    return;
}

void TimeStampExit()
{
    DeleteCriticalSection(&cs);
    return;
}

#if (!BUFFER_MODE)
void TimeStamp(int func_id , int sub_id)
{
    EnterCriticalSection(&cs);

    FILE* fp;
    unsigned long long systime_tmp;
    unsigned int output[2];
    char s[]= TIMESTAMP_FILENAME;
    fp = fopen(s,"ab+");
    if(NULL == fp)
    {//logPrintf(logLevelInfo_e, "DIFF_DETECT", "e e e\n!");
    }

    //add time stamp
    SYSTEMTIME sysTime;
    GetLocalTime(&sysTime);
    systime_tmp = sysTime.wMilliseconds +
                  sysTime.wSecond*1000 +
                  sysTime.wMinute*1000*60 + 
                  sysTime.wHour*1000*60*60 +
                  sysTime.wDay*1000*60*60*24+
                  DayOfMonth[sysTime.wMonth]*1000*60*60*24;
                  //sysTime.wYear*1000*60*60*24*30*12 +
    output[0] = systime_tmp & 0xffffffff;
    output[1] = (func_id << 16) | sub_id;
    change(output);
    change((output+1));

    fwrite(output,8,1,fp);
 
    fclose(fp);

    LeaveCriticalSection(&cs);
}
#else

void TimeStamp(int func_id , int sub_id)
{
    EnterCriticalSection(&cs);

    if(BUFFER_LEN == bufindex)//full
    {
        bufindex = 0;
        FILE* fp;
        char s[]= TIMESTAMP_FILENAME;
        fp = fopen(s,"ab+");
        if(NULL == fp)
        {//logPrintf(logLevelInfo_e, "DIFF_DETECT", "e e e\n!");
        }

        fwrite(TimeStampBuf,4*BUFFER_LEN,1,fp);
        fclose(fp);
    }
    else
    {
        SYSTEMTIME sysTime;
        unsigned long long systime_tmp;
        GetLocalTime(&sysTime);
        systime_tmp = sysTime.wMilliseconds +
                  sysTime.wSecond*1000 +
                  sysTime.wMinute*1000*60 + 
                  sysTime.wHour*1000*60*60 +
                  sysTime.wDay*1000*60*60*24+
                  DayOfMonth[sysTime.wMonth]*1000*60*60*24;
        TimeStampBuf[bufindex] = systime_tmp & 0xffffffff;
        TimeStampBuf[bufindex+1] = (func_id << 16) | sub_id;
        change(&TimeStampBuf[bufindex]);
        change(&TimeStampBuf[bufindex+1]);
        bufindex = bufindex + 2;
    }

    LeaveCriticalSection(&cs);
}
#endif