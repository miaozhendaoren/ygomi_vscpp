/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  messageProcessClass.h
* @brief 
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Qin Shi   	  create
*******************************************************************************
*/
#pragma once
#include <stdio.h>
#include <string.h>
#include "typeDefine.h"
#define MAX_PDU_NUM							300
#define MAX_PAYLOAD_BYTE_NUM				(1024*5000)  //5000K


struct tlv_t
{
	int8 tag;
	int8 len;
	int16 value;
};
struct pduHeader_t
{
	uint16 pduType;
	uint16 operate;
	uint32 pduOffset;
};
struct diffMsgHeader_t
{
	uint16 headerLen;
	uint16 msgTypeID;
	uint16 priority;
	uint16 numPDUs;
	uint32 vehicleID;
	uint32 payloadLen;
};
struct diffRptMsg_t
{
	diffMsgHeader_t msgHeader;
	union
	{
		tlv_t tlvArray[MAX_PDU_NUM];
		pduHeader_t pduHeader[MAX_PDU_NUM];
	}payloadHeader;
	uint8 *payload;
};

