/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_Master_DBAccesss.cpp
* @brief process all the operations with server'sdatabase.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#include <Windows.h>
#include <mmsystem.h>
#include "appInitCommon.h"
#include "Thread_Master_DBAccess.h"
#pragma comment(lib,"winmm.lib")

unsigned int __stdcall Thread_Master_DBAccess(void *data)
{
	while(1)
	{
		WaitForSingleObject(g_readySema_msgQueue,INFINITE);
		// check message queue is empty
		if( !messageQueue_gp->empty())
		{
			//uint64 vehicleId;
			int32 msgType;
			diffRptMsg_t* diffRptMsgPtr;
			messageProcessClass currentMsg;
			messageQueue_gp->front(&currentMsg);
			
			diffRptMsgPtr = currentMsg.getDiffRptMsg();
			msgType = diffRptMsgPtr->msgHeader.msgTypeID;
			if( (msgType & 0x1000) == 0x0000)
			{
				int pduIdx;
				switch(msgType)
				{
				case 0x0000:
					
					for(pduIdx = 0;pduIdx < diffRptMsgPtr->msgHeader.numPDUs;pduIdx++)
					{
						int8 tag = diffRptMsgPtr->payloadHeader.tlvArray[pduIdx].tag;
						int16 value = diffRptMsgPtr->payloadHeader.tlvArray[pduIdx].value;
						if((1 == tag) && (3 == value))
						{
							// delete furniture
							database_gp->resetFurniture();
						}
					}
					break;
				case 0x0001:
					for(pduIdx = 0;pduIdx < diffRptMsgPtr->msgHeader.numPDUs;pduIdx++)
					{
						int pduLen;
						uint32 type = (diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].operate << 16) + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduType;
						if(pduIdx == (diffRptMsgPtr->msgHeader.numPDUs - 1))
						{
							pduLen = diffRptMsgPtr->msgHeader.length - diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
						}
						else
						{
							pduLen = diffRptMsgPtr->payloadHeader.pduHeader[pduIdx+1].pduOffset - diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset;
						}
						// add to  database
						switch(type)
						{
						
							case 0x00000000:// add a new segment data
								database_gp->addSegmentTlv(diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
								break;
							case 0x00000001:
								database_gp->addFurnitureTlv(diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
								PlaySound("./Resource/detect.wav",NULL, SND_FILENAME|SND_ASYNC);
								break;
							case 0x00000005:
								break;
							case 0x00000006:
								// add a vector list for specified segment.
								database_gp->addAllVectorsInSegTlv(diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
								break;
							case 0x00020001:
								database_gp->reduceFurnitureTlv(diffRptMsgPtr->payload + diffRptMsgPtr->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
								PlaySound("./Resource/reduce.wav",NULL, SND_FILENAME|SND_ASYNC);
								break;
						}
					}
					break;
				}
			}
			// for debug and test client
			{
				diffRptMsg_t* diffMsgPtr = currentMsg.getDiffRptMsg();
				diffRptMsg_t* updateMsgPtr = currentMsg.getUpdateRptMsg();
				memcpy((void*)updateMsgPtr,(void*)diffMsgPtr,sizeof(diffRptMsg_t));
				databaseQueue_gp->push(&currentMsg);
			}
			messageQueue_gp->pop();
			ReleaseSemaphore(g_readySema_readDb,1,NULL);
			ReleaseSemaphore(g_readySema_Redraw,1,NULL);
		}
	}
	return 0;
}