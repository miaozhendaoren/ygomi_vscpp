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
#define MAX_ROAD_POINT_BYTES				(1024*10000) //10000K
//message ID
#define STATUS_UPDATE_RPT_MSG				0x0000
#define STATUS_UPDATE_RSP_MSG				0x1000
#define DIFF_RPT_MSG						0x0001
#define DIFF_RSP_MSG						0x1001

#define UPDATE_REQ_MSG						0x0002
#define UPDATE_RSP_MSG						0x1002

#define ERR_INDICATION_RPT_MSG				0x0003

// error code 
//#define MSG_OK								0x0000
//#define MSG_ERR								0x0001

//operation
#define ADD_OPERATION                       0x0000
#define UPDATE_OPERATION                    0x0001
#define REDUCE_OPERATION                    0x0002


// attributes
#define SEGMENT                             0x0000
#define FURNITURE                           0x0001
#define VECTORLIST                          0x0006
#define LANE_POINT                          0x0002
#define SIDE_LANE                           0x0008
#define FURNITURELIST                       0x0005
//operation+attributes

#define ADD_NEW_SEGMENT						(((uint32)ADD_OPERATION << 16) + SEGMENT)
#define ADD_NEW_FURNITURE					(((uint32)ADD_OPERATION << 16) + FURNITURE) //0x00000001
#define UPDATE_FURNITURE					(((uint32)UPDATE_OPERATION << 16) + FURNITURE) //0x00010001
#define ADD_ALL_VECTORLIST					(((uint32)ADD_OPERATION << 16) + VECTORLIST) // 0x00000006
#define REDUCE_ONE_FURNITURE				(((uint32)REDUCE_OPERATION << 16) + FURNITURE) //0x00020001
#define ADD_LANE_POINT						(((uint32)ADD_OPERATION << 16) + LANE_POINT) //0x00000002
#define SIDE_LANE_INFO                      (((uint32)ADD_OPERATION << 16) + SIDE_LANE) //0x00000003
#define ADD_ALL_FURNITURE					(((uint32)ADD_OPERATION << 16) + FURNITURELIST)//0x00000005

//structure
enum statusMsgTag_e:uint8
{
	communitationStatus_e = 0x0,
	resetDatabaseFurniture_e,
	locationAreaId_e,
	databaseVersion_e,
};
enum pduType_e:uint16
{
	segmentInfo_e = SEGMENT,
	furnitureElement_e,
	curLaneInfo_e,
	vectorElement_e,
	dynamicData_e,
	furnitureList_e,
	vectorList_e,
	dynamicDataList_e,
    sideLaneInfo_e,
};
enum operation_e:uint16
{
	addDatabase_e = 0x00,
	updateDatabase_e,
	deleteDatabase_e,
};
enum msgLevel_e:uint16
{
	highLevel_e = 0x00,
	middLevel_e,
	lowLevel_e,
};
struct furnitureHeader_t
{
	uint32 segMentID;
	uint32 furnitureID;
};
struct vectorHeader_t
{
	uint32 segMentID;
	uint32 vectorID;
};
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
struct errorPdu_t
{
	uint16 pduType;
	uint16 subErrCode;
};
struct diffRspMsg_t
{
	diffMsgHeader_t msgHeader;
	uint16			errorCode;
	//errorPdu_t      erroPdu[MAX_PDU_NUM];
};

class messageProcessClass
{
public:
	messageProcessClass(void);
	~messageProcessClass(void);
	void setMsgHeader(uint32* msgIn,uint16 msgTypeID,uint32 vehicleID,uint16 priority,uint32 payloadLen,uint16 numPDUs,uint16 headerLen);
	void setDiffRptPduMsgHeader(uint16 pduIdx,uint16 pduType,uint16 operate,uint32 pduOffset);
	void setDiffRptMsgPayload(uint8* byteInPtr,uint32 size);
	void packedRspMsg(uint32* msgBuffInPtr,uint32* sendoutMsgPtr);
	void setRspMsgErrCode(uint16*msgIn,uint16 errCode);
	void setDiffRspSubErrCode(uint32 pduIdx, uint16 pduType,uint16 subErrCode);
	void setDiffRptMsgPayloadAddr(uint8* bufAddr);
	void prcocessRecvMsg(uint32* msgIn);
	void parseRspMsg(uint32* msgBuffPtr,uint32* msgInPtr);
	void parseStatusRptMsg(uint32 * inputMsgPtr,uint32* localRptMsgPtr);
	uint8* getDiffRptMsgPayload(void);
	diffRptMsg_t* getUpdateRptMsg(void);
	diffRptMsg_t* getDiffRptMsg(void);
	diffRspMsg_t* getUpdateRspMsg(void);
	pduHeader_t* getDiffRptPduHeader(uint32 pduIdx);
	diffMsgHeader_t* getDiffRspMsgHeader(void);
	void packedStatusRptMsg(uint32* sendoutMsgPtr);

private:
	diffRptMsg_t diffRptMsg;
	diffRspMsg_t diffRspMsg;
	diffRptMsg_t updateRptMsg;
	diffRspMsg_t updateRspMsg;
};
