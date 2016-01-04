/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  messageQueueClass.h
* @brief Message queue header file
*
* Change Log:
*      Date                Who             What
*      2015/1/13          Qin Shi          Create
*******************************************************************************
*/
#pragma once
//#include <Windows.h>    //HANDLE header file
#include <queue>
#include "typeDefine.h"
#include "messageProcessClass.h"
using namespace std;

class messageQueueClass
{
private:
	int32  queueNum;
	priority_queue<messageProcessClass> _msgQueue;
	//HANDLE _hMutex;
public:
	messageQueueClass(void);
	messageQueueClass(int32 queueSize);
	bool push(messageProcessClass* msgInPtr);
	bool pop();
	bool empty();
	int32 size();
	//bool front(messageProcessClass* msgInPtr);
	//bool back(messageProcessClass* msgInPtr);
    bool top(messageProcessClass* msgInPtr);
	~messageQueueClass(void);
};