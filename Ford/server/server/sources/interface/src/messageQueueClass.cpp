/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  messageQueueClass.cpp
* @brief Message queue source file
*
* Change Log:
*      Date                Who             What
*      2015/1/13          Qin Shi          Create
*******************************************************************************
*/

#include "messageQueueClass.h"

messageQueueClass::messageQueueClass(void)
{
	queueNum = 4;
/*	_hMutex = CreateMutex(NULL,FALSE,NULL);
	ReleaseMutex(_hMutex); */ 
}
messageQueueClass::messageQueueClass(int32 queueSize)
{
	queueNum = queueSize;
	//_hMutex = CreateMutex(NULL,FALSE,NULL);
	//ReleaseMutex(_hMutex);  
}
bool messageQueueClass::push(messageProcessClass* msgInPtr)
{
	//WaitForSingleObject(_hMutex,INFINITE);
    int msgSize = _msgQueue.size();
	if( msgSize < queueNum - 10) // 10 for highLevel_e
	{
		_msgQueue.push(*msgInPtr);
		//ReleaseMutex(_hMutex);
		return true;
	}
	else if( (highLevel_e == msgInPtr->messagePriority) && (msgSize < queueNum) )
	{
		_msgQueue.push(*msgInPtr);
		return true;
	}
    else
    {
        return false;
    }
}
bool messageQueueClass::pop()
{
	//WaitForSingleObject(_hMutex,INFINITE);
	if(_msgQueue.empty())
	{	
		//ReleaseMutex(_hMutex);
		return false;
	}
	else
	{	
		_msgQueue.pop();
		//ReleaseMutex(_hMutex);
		return true;
	}
}
bool messageQueueClass::empty()
{
	bool flag;
	//WaitForSingleObject(_hMutex,INFINITE);
	flag = _msgQueue.empty();
	//ReleaseMutex(_hMutex);
	return flag;
}
int32 messageQueueClass::size()
{
	int32 number;
	//WaitForSingleObject(_hMutex,INFINITE);
	number = _msgQueue.size();
	//ReleaseMutex(_hMutex);
	return number;
}
/*bool messageQueueClass::front(messageProcessClass* msgInPtr)
{
	//WaitForSingleObject(_hMutex,INFINITE);
	if(_msgQueue.empty())
	{ 
		//ReleaseMutex(_hMutex);
		return false;
	}
	else
	{ 
		*msgInPtr = _msgQueue.front(); 
		//ReleaseMutex(_hMutex);
		return true;
	}
}
bool messageQueueClass::back(messageProcessClass* msgInPtr)
{
	//WaitForSingleObject(_hMutex,INFINITE);
	if(_msgQueue.empty())
	{ 
		//ReleaseMutex(_hMutex);
		return false;
	}
	else
	{ 
		*msgInPtr = _msgQueue.back(); 
		//ReleaseMutex(_hMutex);
		return true;
	}
}*/
bool messageQueueClass::top(messageProcessClass* msgInPtr)
{
	//WaitForSingleObject(_hMutex,INFINITE);
	if(_msgQueue.empty())
	{ 
		//ReleaseMutex(_hMutex);
		return false;
	}
	else
	{ 
		*msgInPtr = _msgQueue.top(); 
		//ReleaseMutex(_hMutex);
		return true;
	}
}
messageQueueClass::~messageQueueClass(void)
{
	int idx;
	int32 msgNum = _msgQueue.size();
	for(idx = 0; idx < msgNum;idx++)
	{
		_msgQueue.pop();
	}
	//CloseHandle(_hMutex);  
}

