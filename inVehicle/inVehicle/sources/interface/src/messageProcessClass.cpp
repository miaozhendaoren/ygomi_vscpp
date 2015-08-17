/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  messageProcessClass.cpp
* @brief a class to process the messages.
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Qin Shi		  create
*******************************************************************************
*/
#include <stdio.h>
#include <string.h>
#include "typeDefine.h"
#include "messageProcessClass.h"


messageProcessClass::messageProcessClass(void)
{
	memset((void*)&diffRptMsg,0,sizeof(diffRptMsg));
	memset((void*)&updateRptMsg,0,sizeof(updateRptMsg));
}
void messageProcessClass::packedDiffRptMsg(uint32* sendoutMsgPtr)
{
	int     pduIdx;
	uint32  numPdus    = diffRptMsg.msgHeader.numPDUs;
	uint32* sendMsg32Ptr = (uint32*)sendoutMsgPtr;
	memcpy((void*)sendMsg32Ptr,(void*)&diffRptMsg,sizeof(diffMsgHeader_t));// message hearder
	sendMsg32Ptr += (sizeof(diffMsgHeader_t)/4);
	for(pduIdx = 0; pduIdx < (int32)numPdus; pduIdx++)
	{	
		diffRptPduHeader_t* pduPtr = (diffRptPduHeader_t*)sendMsg32Ptr;

		pduPtr->pduType = diffRptMsg.payloadHeader.pduHeader[pduIdx].pduType;
		pduPtr->operate = diffRptMsg.payloadHeader.pduHeader[pduIdx].operate;
		pduPtr->pduOffset = diffRptMsg.payloadHeader.pduHeader[pduIdx].pduOffset;
		sendMsg32Ptr += 2;
		switch (pduPtr->pduType)
		{
			case 1:
			case 2:
					pduPtr->pduDiffTypeHeader.fntHeader.segMentID = diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.segMentID;
					pduPtr->pduDiffTypeHeader.fntHeader.furnitureID = diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.furnitureID;
					sendMsg32Ptr += 2;
				break;
			case 3:
					pduPtr->pduDiffTypeHeader.vectorHeader.segMentID = diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.segMentID;
					pduPtr->pduDiffTypeHeader.vectorHeader.vectorID = diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.vectorID;
					sendMsg32Ptr += 2;
				break;
		}
	}
	memcpy((void*)sendMsg32Ptr,(void*)diffRptMsg.payload,diffRptMsg.msgHeader.length);
}
void messageProcessClass::packedStatusRptMsg(uint32* sendoutMsgPtr)
{
	int     pduIdx;
	uint32  numPdus    = diffRptMsg.msgHeader.numPDUs;
	uint32* sendMsg32Ptr = (uint32*)sendoutMsgPtr;
	memcpy((void*)sendMsg32Ptr,(void*)&diffRptMsg,sizeof(diffMsgHeader_t));// message hearder
	sendMsg32Ptr += (sizeof(diffMsgHeader_t)/4);
	for(pduIdx = 0; pduIdx < numPdus; pduIdx++)
	{	
		tlv_t* tlvPtr = (tlv_t*)sendMsg32Ptr;

		tlvPtr->tag = diffRptMsg.payloadHeader.tlvArray[pduIdx].tag;
		tlvPtr->len = diffRptMsg.payloadHeader.tlvArray[pduIdx].len;
		tlvPtr->value = diffRptMsg.payloadHeader.tlvArray[pduIdx].value;
		sendMsg32Ptr += sizeof(tlv_t)/4;
	}
}
void messageProcessClass::parseUpdateRptMsg(uint32 * inputMsgPtr,uint32* localRptMsgPtr)
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

	for(pduIdx = 0; pduIdx < (int32)numPdus; pduIdx++)
	{
		diffRptPduHeader_t* pduPtr = (diffRptPduHeader_t*)sendMsg32Ptr;

		diffMsgPtr->payloadHeader.pduHeader[pduIdx].pduType = pduPtr->pduType;
		diffMsgPtr->payloadHeader.pduHeader[pduIdx].operate = pduPtr->operate;
		diffMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset = pduPtr->pduOffset;
		sendMsg32Ptr += 2;
		switch (pduPtr->pduType)
		{
			case furnitureSign_e:
			case furnitureElement_e:
					diffMsgPtr->payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.segMentID = pduPtr->pduDiffTypeHeader.fntHeader.segMentID;
					diffMsgPtr->payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.furnitureID = pduPtr->pduDiffTypeHeader.fntHeader.furnitureID;
					sendMsg32Ptr += 2;
				break;
			case vectorElement_e:
					diffMsgPtr->payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.segMentID = pduPtr->pduDiffTypeHeader.vectorHeader.segMentID;
					diffMsgPtr->payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.vectorID = pduPtr->pduDiffTypeHeader.vectorHeader.vectorID;
					sendMsg32Ptr += 2;
				break;
		}
	}
	memcpy((void*)diffMsgPtr->payload,(void*)sendMsg32Ptr,diffMsgPtr->msgHeader.length);
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

	for(pduIdx = 0; pduIdx < (int32)numPdus; pduIdx++)
	{
		tlv_t* tlvPtr = (tlv_t*)sendMsg32Ptr;

		diffMsgPtr->payloadHeader.tlvArray[pduIdx].tag = tlvPtr->tag;
		diffMsgPtr->payloadHeader.tlvArray[pduIdx].len = tlvPtr->len;
		diffMsgPtr->payloadHeader.tlvArray[pduIdx].value = tlvPtr->value;
		sendMsg32Ptr += (sizeof(tlv_t)/4);
	}
}
void messageProcessClass::setMsgHeader(uint32* msgIn,uint32 msgTypeID,uint64 vehicleID,uint16 priority,uint32 length,uint16 numPDUs)
{
	diffMsgHeader_t* msgHeaderPtr = (diffMsgHeader_t*)msgIn;
	msgHeaderPtr->msgTypeID = msgTypeID;
	msgHeaderPtr->vehicleID = vehicleID;
	msgHeaderPtr->numPDUs = numPDUs;
	msgHeaderPtr->priority = priority;
	msgHeaderPtr->length = length;
}
void messageProcessClass::setDiffRptPduMsgHeader(uint16 pduIdx,uint16 pduType,uint16 operate,uint32 pduOffset)
{
	diffRptMsg.payloadHeader.pduHeader[pduIdx].operate = operate;
	diffRptMsg.payloadHeader.pduHeader[pduIdx].pduType = pduType;
	diffRptMsg.payloadHeader.pduHeader[pduIdx].pduOffset = pduOffset;
}
void messageProcessClass::setDiffRptPduFntHeader(uint16 pduIdx,	uint32 segMentID,uint32 furnitureID)
{
	diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.segMentID = segMentID;
	diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.furnitureID = furnitureID;
}
void messageProcessClass::setDiffRptPduVectorHeader(uint16 pduIdx, uint32 segMentID,uint32 vectorID)
{
	diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.segMentID = segMentID;
	diffRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.vectorID = vectorID;
}
void messageProcessClass::setDiffRptMsgPayload(uint8* byteInPtr,uint32 size)
{
	memcpy((void*)diffRptMsg.payload,(void*)byteInPtr,size);
}
uint8* messageProcessClass::getDiffRptMsgPayload(void)
{
	return diffRptMsg.payload;
}
diffRptMsg_t* messageProcessClass::getUpdateRptMsg(void)
{
	return &updateRptMsg;
}
diffRptMsg_t* messageProcessClass::getDiffRptMsg(void)
{
	return &diffRptMsg;
}
diffRspMsg_t* messageProcessClass::getUpdateRspMsg(void)
{
	return &updateRspMsg;
}
diffMsgHeader_t* messageProcessClass::getDiffRspMsgHeader(void)
{
	return &(diffRspMsg.msgHeader);
}
diffRptPduHeader_t* messageProcessClass::getDiffRptPduHeader(uint32 pduIdx)
{
	return &(diffRptMsg.payloadHeader.pduHeader[pduIdx]);
}
void messageProcessClass::packedRspMsg(uint32* msgBuffInPtr,uint32* sendoutMsgPtr)
{
	memcpy((void*)sendoutMsgPtr,(void*)msgBuffInPtr,sizeof(diffRspMsg_t));
}
void messageProcessClass::parseRspMsg(uint32* msgBuffPtr,uint32* msgInPtr)
{
	memcpy((void*)msgBuffPtr,(void*)msgInPtr,sizeof(diffRspMsg_t));
}
void messageProcessClass::setRspMsgErrCode(uint16* msgIn,uint16 errCode)
{
	*msgIn = errCode;
}
//void VehicleDataProcessClass::setDiffRspSubErrCode(uint32 pduIdx, uint16 pduType,uint16 subErrCode)
//{
//	diffRspMsg.erroPdu[pduIdx].pduType = pduType;
//	diffRspMsg.erroPdu[pduIdx].subErrCode = subErrCode;
//}
void messageProcessClass::CompareDiff(char* msgFrmDb)
{
	// parse the msgbody and compare with it.
}
void messageProcessClass::prcocessRecvMsg(uint32* msgIn)
{
	uint32 msgID = msgIn[0];
	switch(msgID)
	{
		case STATUS_UPDATE_RPT_MSG:// status report message
			parseStatusRptMsg(msgIn,(uint32*)&updateRptMsg);
			break;
		case DIFF_RPT_MSG:
			break;
		case UPDATE_REQ_MSG:// update message
			parseUpdateRptMsg(msgIn,(uint32*)&updateRptMsg);
			break;
		case STATUS_UPDATE_RSP_MSG:
		case DIFF_RSP_MSG:
			parseRspMsg((uint32*)&diffRspMsg,msgIn);
			break;
		case UPDATE_RSP_MSG:
			parseRspMsg((uint32*)&updateRspMsg,msgIn);
			break;
	}
}
messageProcessClass::~messageProcessClass(void)
{
}