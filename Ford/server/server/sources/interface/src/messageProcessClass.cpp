/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  messageProcessClass.cpp
* @brief 
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Qin Shi		  create
*******************************************************************************
*/
#include <stdio.h>
#include<iostream>
#include "typeDefine.h"
#include "messageProcessClass.h"

using namespace std;

messageProcessClass::messageProcessClass(void)
{
	memset((void*)&diffRptMsg,0,sizeof(diffRptMsg));
	memset((void*)&updateRptMsg,0,sizeof(updateRptMsg));
}
void messageProcessClass::packedUpdateRptMsg(uint32* sendoutMsgPtr)
{
	int     pduIdx;
	uint32  numPdus    = updateRptMsg.msgHeader.numPDUs;
	uint32* sendMsg32Ptr = (uint32*)sendoutMsgPtr;
	uint32	allPduHeadSize = 0;
	updateRptMsg.msgHeader.headerLen = sizeof(diffMsgHeader_t) + numPdus*sizeof(pduHeader_t);
	memcpy((void*)sendMsg32Ptr,(void*)&updateRptMsg,sizeof(diffMsgHeader_t));// message hearder
	sendMsg32Ptr += (sizeof(diffMsgHeader_t)/4);

	for(pduIdx = 0; pduIdx < numPdus; pduIdx++)
	{	
		pduHeader_t* pduPtr = (pduHeader_t*)sendMsg32Ptr;

		pduPtr->pduType = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduType;
		pduPtr->operate = updateRptMsg.payloadHeader.pduHeader[pduIdx].operate;
		pduPtr->pduOffset = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduOffset;
		sendMsg32Ptr += (sizeof(pduHeader_t)/4);
	}

}
void messageProcessClass::packedStatusRptMsg(uint32* sendoutMsgPtr)
{
	int     pduIdx;
	uint32  numPdus    = updateRptMsg.msgHeader.numPDUs;
	uint32* sendMsg32Ptr = (uint32*)sendoutMsgPtr;
	updateRptMsg.msgHeader.headerLen = sizeof(diffMsgHeader_t) + numPdus*sizeof(tlv_t);
	memcpy((void*)sendMsg32Ptr,(void*)&updateRptMsg,sizeof(diffMsgHeader_t));// message hearder
	sendMsg32Ptr += (sizeof(diffMsgHeader_t)/4);
	for(pduIdx = 0; pduIdx < numPdus; pduIdx++)
	{	
		tlv_t* tlvPtr = (tlv_t*)sendMsg32Ptr;

		tlvPtr->tag = updateRptMsg.payloadHeader.tlvArray[pduIdx].tag;
		tlvPtr->len = updateRptMsg.payloadHeader.tlvArray[pduIdx].len;
		tlvPtr->value = updateRptMsg.payloadHeader.tlvArray[pduIdx].value;
		sendMsg32Ptr += sizeof(tlv_t)/4;
	}
}
void messageProcessClass::parseDiffRptMsg(uint32 * inputMsgPtr,uint32* localRptMsgPtr)
{
	int     pduIdx;
	diffRptMsg_t*	msgTempPtr = (diffRptMsg_t*)inputMsgPtr;
	uint32	msgTypeId = msgTempPtr->msgHeader.msgTypeID;
	diffRptMsg_t*	diffMsgPtr = (diffRptMsg_t*)localRptMsgPtr;

	uint32  numPdus    = msgTempPtr->msgHeader.numPDUs;
	uint32	allPduHeadSize = 0;
	uint32* sendMsg32Ptr = (uint32*)inputMsgPtr;
	memcpy((void*)diffMsgPtr,(void*)sendMsg32Ptr,sizeof(diffMsgHeader_t));// message hearder
	sendMsg32Ptr += (sizeof(diffMsgHeader_t)/4);

	for(pduIdx = 0; pduIdx < numPdus; pduIdx++)
	{
		pduHeader_t* pduPtr = (pduHeader_t*)sendMsg32Ptr;
		diffMsgPtr->payloadHeader.pduHeader[pduIdx] = *pduPtr;
		sendMsg32Ptr += (sizeof(pduHeader_t)/4);
		//allPduHeadSize += 8;
	}
	diffRptMsg.payload = (uint8*)sendMsg32Ptr;
	//memcpy((void*)diffRptMsg.payload,(void*)sendMsg32Ptr,diffRptMsg.msgHeader.length - allPduHeadSize);
}
void messageProcessClass::setMsgHeader(uint32* msgIn,uint16 headerLen,uint16 msgTypeID,uint32 vehicleID,uint16 priority,uint32 pduLen,uint16 numPDUs)
{
	diffMsgHeader_t* msgHeaderPtr = (diffMsgHeader_t*)msgIn;
	msgHeaderPtr->headerLen = headerLen;
	msgHeaderPtr->msgTypeID = msgTypeID;
	msgHeaderPtr->vehicleID = vehicleID;
	msgHeaderPtr->numPDUs = numPDUs;
	msgHeaderPtr->priority = priority;
	msgHeaderPtr->payloadLen = pduLen;
}
void messageProcessClass::setUpdateRptPduMsgHeader(uint16 pduIdx,uint16 pduType,uint16 operate,uint32 pduOffset)
{
	updateRptMsg.payloadHeader.pduHeader[pduIdx].operate = operate;
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduType = pduType;
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduOffset = pduOffset;
}
void messageProcessClass::setUpdateRptMsgPayload(uint8* byteInPtr,uint32 size)
{
	updateRptMsg.payload = new uint8[size];
	memcpy((void*)updateRptMsg.payload,(void*)byteInPtr,size);
}
uint8* messageProcessClass::getDiffRptMsgPayload(void)
{
	return diffRptMsg.payload;
}
diffMsgHeader_t* messageProcessClass::getDiffRptMsgHeader(void)
{
	return &(diffRptMsg.msgHeader);
}
diffRspMsg_t* messageProcessClass::getDiffRspMsg(void)
{
	return &diffRspMsg;
}
diffRptMsg_t* messageProcessClass::getDiffRptMsg(void)
{
	return &diffRptMsg;
}
pduHeader_t* messageProcessClass::getDiffRptPduHeader(uint32 pduIdx)
{
	return &(diffRptMsg.payloadHeader.pduHeader[pduIdx]);
}
diffRspMsg_t* messageProcessClass::getUpdateRspMsg(void)
{
	return &updateRspMsg;
}  
diffRptMsg_t* messageProcessClass::getUpdateRptMsg(void)
{
	return &updateRptMsg;
}  

void messageProcessClass::packedDiffRspMsg(uint32* sendoutMsgPtr)
{
	memcpy((void*)sendoutMsgPtr,(void*)&diffRspMsg,sizeof(diffRspMsg_t));
}
void messageProcessClass::parseRspMsg(uint32* msgBuffPtr,uint32* inputMsgPtr)
{
	diffRspMsg_t*   tempPtr = (diffRspMsg_t*)msgBuffPtr;
	memcpy((void*)tempPtr,(void*)inputMsgPtr,sizeof(diffRspMsg_t));
}
void messageProcessClass::setDiffRspErrCode(uint16 errCode)
{
	diffRspMsg.errorCode = errCode;
}
void messageProcessClass::parseStatusRptMsg(uint32 * inputMsgPtr,uint32* localRptMsgPtr)
{
	int     pduIdx;
	diffRptMsg_t*	msgTempPtr = (diffRptMsg_t*)inputMsgPtr;
	uint32	msgTypeId = msgTempPtr->msgHeader.msgTypeID;
	diffRptMsg_t*	diffMsgPtr = (diffRptMsg_t*)localRptMsgPtr;

	uint32  numPdus    = msgTempPtr->msgHeader.numPDUs;
	uint32	allPduHeadSize = 0;
	uint32* sendMsg32Ptr = (uint32*)inputMsgPtr;
	memcpy((void*)diffMsgPtr,(void*)sendMsg32Ptr,sizeof(diffMsgHeader_t));// message hearder
	sendMsg32Ptr += (sizeof(diffMsgHeader_t)/4);
	
	if(numPdus > MAX_PDU_NUM)
	{
		cout<< "the pdu number exceeds the maximum supported number." << endl;
	}

	for(pduIdx = 0; pduIdx < (int32)numPdus; pduIdx++)
	{
		tlv_t* tlvPtr = (tlv_t*)sendMsg32Ptr;

		diffMsgPtr->payloadHeader.tlvArray[pduIdx].tag = tlvPtr->tag;
		diffMsgPtr->payloadHeader.tlvArray[pduIdx].len = tlvPtr->len;
		diffMsgPtr->payloadHeader.tlvArray[pduIdx].value = tlvPtr->value;
		sendMsg32Ptr += (sizeof(tlv_t)/4);
	}
}
void messageProcessClass::prcocessRecvMsg(uint32* msgIn)
{
	uint32 msgID = msgIn[0];
	switch(msgID)
	{
		case STATUS_UPDATE_RPT_MSG:// status report message
			parseStatusRptMsg(msgIn,(uint32*)&diffRptMsg);
			break;
		case DIFF_RPT_MSG:// update message
		{
			parseDiffRptMsg(msgIn,(uint32*)&diffRptMsg);
			break;
		}
		case UPDATE_REQ_MSG:
			break;
		case STATUS_UPDATE_RSP_MSG:
			break;
		case DIFF_RSP_MSG:
			parseRspMsg((uint32*)&diffRspMsg,msgIn);
			break;
		case UPDATE_RSP_MSG:
			parseRspMsg((uint32*)&updateRspMsg,msgIn);
			break;
		case ERR_INDICATION_RPT_MSG:
			break;
	}
}
void messageProcessClass::getVehicleIDInMsg(void* msgInPtr,uint32* vehicleID)
{
	diffMsgHeader_t* msgHeadPtr = (diffMsgHeader_t*)msgInPtr;
	*vehicleID = msgHeadPtr->vehicleID;
}
void messageProcessClass::setMessageLength(void* msgInPtr,uint32 len)
{
    diffMsgHeader_t* msgHeadPtr = (diffMsgHeader_t*)msgInPtr;
    msgHeadPtr->payloadLen = len;
}
messageProcessClass::~messageProcessClass(void)
{
}