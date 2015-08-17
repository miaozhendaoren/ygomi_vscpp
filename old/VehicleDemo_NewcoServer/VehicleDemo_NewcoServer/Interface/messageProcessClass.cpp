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
#include <string.h>
#include "typeDefine.h"
#include "messageProcessClass.h"


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
	memcpy((void*)sendMsg32Ptr,(void*)&updateRptMsg,sizeof(diffMsgHeader_t));// message hearder
	sendMsg32Ptr += (sizeof(diffMsgHeader_t)/4);
	for(pduIdx = 0; pduIdx < numPdus; pduIdx++)
	{	
		diffRptPduHeader_t* pduPtr = (diffRptPduHeader_t*)sendMsg32Ptr;

		pduPtr->pduType = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduType;
		pduPtr->operate = updateRptMsg.payloadHeader.pduHeader[pduIdx].operate;
		pduPtr->pduOffset = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduOffset;
		sendMsg32Ptr += 2;

		switch (pduPtr->pduType)
		{
			case 1:
			case 2:
					pduPtr->pduDiffTypeHeader.fntHeader.segMentID = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.segMentID;
					pduPtr->pduDiffTypeHeader.fntHeader.furnitureID = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.furnitureID;
					sendMsg32Ptr += 2;

				break;
			case 3:
					pduPtr->pduDiffTypeHeader.vectorHeader.segMentID = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.segMentID;
					pduPtr->pduDiffTypeHeader.vectorHeader.vectorID = updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.vectorID;
					sendMsg32Ptr += 2;

				break;
		}
	}
	memcpy((void*)sendMsg32Ptr,(void*)updateRptMsg.payload,updateRptMsg.msgHeader.length);
}
void messageProcessClass::packedStatusRptMsg(uint32* sendoutMsgPtr)
{
	int     pduIdx;
	uint32  numPdus    = updateRptMsg.msgHeader.numPDUs;
	uint32* sendMsg32Ptr = (uint32*)sendoutMsgPtr;
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
	memcpy((void*)diffRptMsg.payload,(void*)sendMsg32Ptr,diffRptMsg.msgHeader.length );
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
void messageProcessClass::setDiffMsgRspHeader(	uint32 msgTypeID,uint64 vehicleID,uint16 priority,uint32 length,uint16 numPDUs)
{
	diffRspMsg.msgHeader.msgTypeID = msgTypeID;
	diffRspMsg.msgHeader.vehicleID = vehicleID;
	diffRspMsg.msgHeader.numPDUs = numPDUs;
	diffRspMsg.msgHeader.priority = priority;
	diffRspMsg.msgHeader.length = length;
}
void messageProcessClass::setUpdateRptPduMsgHeader(uint16 pduIdx,uint16 pduType,uint16 operate,uint32 pduOffset)
{
	updateRptMsg.payloadHeader.pduHeader[pduIdx].operate = operate;
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduType = pduType;
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduOffset = pduOffset;
}
void messageProcessClass::setUpdateRptPduFntHeader(uint16 pduIdx,	uint32 segMentID,uint32 furnitureID)
{
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.segMentID = segMentID;
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.fntHeader.furnitureID = furnitureID;
}
void messageProcessClass::setUpdateRptPduVectorHeader(uint16 pduIdx, uint32 segMentID,uint32 vectorID)
{
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.segMentID = segMentID;
	updateRptMsg.payloadHeader.pduHeader[pduIdx].pduDiffTypeHeader.vectorHeader.vectorID = vectorID;
}
void messageProcessClass::setUpdateRptMsgPayload(uint8* byteInPtr,uint32 size)
{
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
diffRptPduHeader_t* messageProcessClass::getDiffRptPduHeader(uint32 pduIdx)
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
		case 0x0000:// status report message

			break;
		case 0x0001:// update message
		{
			parseDiffRptMsg(msgIn,(uint32*)&diffRptMsg);
			diffRptMsg_t* msgInPtr = (diffRptMsg_t*)msgIn;
			diffRspMsg.msgHeader.msgTypeID = msgID + 0x1000;
			diffRspMsg.msgHeader.vehicleID = msgInPtr->msgHeader.vehicleID;
			diffRspMsg.msgHeader.priority = msgInPtr->msgHeader.priority;
			diffRspMsg.msgHeader.numPDUs = 0;
			diffRspMsg.msgHeader.length = 0;
			diffRspMsg.errorCode = MSG_OK;
			break;
		}
		case 0x0002:
			break;
		case 0x1000:
			break;
		case 0x1001:
			parseRspMsg((uint32*)&diffRspMsg,msgIn);
			break;
		case 0x1002:
			parseRspMsg((uint32*)&updateRspMsg,msgIn);
			break;
		case 0x0003:
			break;
	}
}
void messageProcessClass::getVehicleIDInMsg(void* msgInPtr,uint64* vehicleID)
{
	diffMsgHeader_t* msgHeadPtr = (diffMsgHeader_t*)msgInPtr;
	*vehicleID = msgHeadPtr->vehicleID;
}
messageProcessClass::~messageProcessClass(void)
{
}