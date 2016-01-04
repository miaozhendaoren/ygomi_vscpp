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
#include "LogInfo.h"
#include "typeDefine.h"

static LARGE_INTEGER nFreq;
static LARGE_INTEGER nStartTime = {0};

static uint32 TimeStampBuf[BUFFER_LEN];
typedef struct
{
	uint32 tsBufFullFlag;
	uint32 bufindex;
} TimeStampCtrl_t;
static TimeStampCtrl_t TsCtrlBlock;

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

	//get the timer frequence
	QueryPerformanceFrequency(&nFreq);
	//get the start the timer counter
	QueryPerformanceCounter(&nStartTime);

//#if (BUFFER_MODE == BUFFER_LOOP_ACCESS)
	TsCtrlBlock.tsBufFullFlag = 0;
	TsCtrlBlock.bufindex = 0;
	//output the address of TimeStampBuf
	char infobuffer[200];
	sprintf_s(infobuffer, sizeof(infobuffer),"TimeStamp buffer startaddress is 0x%x,buffer size is %d.", (uint32)TimeStampBuf, BUFFER_LEN);
	logPrintf(logLevelNotice_e, "TIMESTAMP", infobuffer);
//#endif

    return;
}

void TimeStampExit()
{
    DeleteCriticalSection(&cs);
    return;
}

#if (BUFFER_MODE == DIRECT_IO_ACCESS)
void FlushTimeStampBuf()
{
	
}
void TimeStamp(int func_id , int sub_id)
{
    EnterCriticalSection(&cs);

    FILE* fp;
    uint64 systime_tmp = 0; 
    unsigned int output[2];
    char s[]= TIMESTAMP_FILENAME;
    fp = fopen(s,"ab+");
    if(NULL == fp)
    {
    }

	LARGE_INTEGER nCurTime;
	QueryPerformanceCounter(&nCurTime);
	double dfMinus, dfFreq, dfTim;
	dfFreq = (double)nFreq.QuadPart;
	dfMinus =(double)(nCurTime.QuadPart - nStartTime.QuadPart);
	dfTim = dfMinus / dfFreq; //the time unit is second 
	systime_tmp = (uint64)(dfTim*1000*1000);   //get current microsecond value	
	
    //add time stamp
    output[0] = systime_tmp & 0xffffffff;
    output[1] = (func_id << 16) | sub_id;
    change(output);
    change((output+1));

    fwrite(output,8,1,fp);
 
    fclose(fp);

    LeaveCriticalSection(&cs);
}
#elif (BUFFER_MODE == BUFFER_CONSTANT_ACCESS)
void FlushTimeStampBuf()
{
	EnterCriticalSection(&cs);
	
	FILE* fp;
	char s[]= TIMESTAMP_FILENAME;
	fp = fopen(s,"ab+");
	if(NULL != fp)
	{
        fwrite(TimeStampBuf,4*(TsCtrlBlock.bufindex),1,fp);
        fclose(fp);
        TsCtrlBlock.bufindex = 0;	
    }
		
	LeaveCriticalSection(&cs);
}
	
void TimeStamp(int func_id , int sub_id)
{
    EnterCriticalSection(&cs);

    if(BUFFER_LEN == TsCtrlBlock.bufindex)//full
    {
        TsCtrlBlock.bufindex = 0;
        FILE* fp = NULL;
        char s[]= TIMESTAMP_FILENAME;
        fp = fopen(s,"ab+");
        if(NULL != fp)
        {
            fwrite(TimeStampBuf,4*BUFFER_LEN,1,fp);
            fclose(fp);
        }
    }
    else
    {
        LARGE_INTEGER nCurTime; 
		uint64 systime_tmp = 0; 
		QueryPerformanceCounter(&nCurTime);
		double dfMinus, dfFreq, dfTim;
		dfFreq = (double)nFreq.QuadPart;
		dfMinus =(double)(nCurTime.QuadPart - nStartTime.QuadPart);
		dfTim = dfMinus / dfFreq; //the time unit is second 
		systime_tmp = (uint64)(dfTim*1000*1000);   //get current microsecond value		
		
        TimeStampBuf[TsCtrlBlock.bufindex] = systime_tmp & 0xffffffff;
        TimeStampBuf[TsCtrlBlock.bufindex+1] = (func_id << 16) | sub_id;
        change(&TimeStampBuf[TsCtrlBlock.bufindex]);
        change(&TimeStampBuf[TsCtrlBlock.bufindex+1]);
        TsCtrlBlock.bufindex += 2; 
    }

    LeaveCriticalSection(&cs);
}
#elif (BUFFER_MODE == BUFFER_LOOP_ACCESS)
void FlushTimeStampBuf()
{
	
}

void TimeStamp(int func_id , int sub_id)
{
	EnterCriticalSection(&cs);
	
	if(BUFFER_LEN == TsCtrlBlock.bufindex)//full
	{
		TsCtrlBlock.bufindex = 0;
		TsCtrlBlock.tsBufFullFlag = 1;
	}
	else
	{
		LARGE_INTEGER nCurTime; 
		uint64 systime_tmp = 0; 
		QueryPerformanceCounter(&nCurTime);
		double dfMinus, dfFreq, dfTim;
		dfFreq = (double)nFreq.QuadPart;
		dfMinus =(double)(nCurTime.QuadPart - nStartTime.QuadPart);
		dfTim = dfMinus / dfFreq; //the time unit is second 
		systime_tmp = (uint64)(dfTim*1000*1000);   //get current microsecond value		

		TimeStampBuf[TsCtrlBlock.bufindex] = systime_tmp & 0xffffffff;
        TimeStampBuf[TsCtrlBlock.bufindex+1] = (func_id << 16) | sub_id;
        change(&TimeStampBuf[TsCtrlBlock.bufindex]);
        change(&TimeStampBuf[TsCtrlBlock.bufindex+1]);
		TsCtrlBlock.bufindex += 2; 
	}

	LeaveCriticalSection(&cs);
}

#endif