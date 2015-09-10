/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_DBUpdate.cpp
* @brief Call Digital Horizon Data Update Manager to update data to Digital Horizon Database
*
* Change Log:
*      Date                Who             What
*      2015/05/08         Xin Shao        Create
*      2015/05/11         Qin Shi         Add the update DB function
*******************************************************************************
*/
#include <winsock.h>
#include <vector>
#include "typeDefine.h"
#include "database.h"
#include "databaseInVehicle.h"
#include "messageProcessClass.h"
#include "LogInfo.h"
#include "AppInitCommon.h"
#include "saveLinePointInSafe.h"

#pragma comment(lib, "ws2_32.lib") 


using ns_historyLine::saveLinePointInSafe;

unsigned int __stdcall Thread_DBUpdate(void *data)
{
    while(1)
    {
		
        //memset(recvBuf,0,sizeof(recvBuf));
RESTART_LABEL:
		sockaddr_in from;
      
        messageProcessClass recvMsg;
		diffRptMsg_t* recvHeader =  recvMsg.getUpdateRptMsg();
		// receive message header length
		int headerLen = 0;
		uint8* recvBuffP = (uint8*)recvHeader; 
		while( headerLen < sizeof(recvHeader->msgHeader.headerLen))
		{
			int nRet = recv(sockClient,(char*)recvBuffP,sizeof(recvHeader->msgHeader.headerLen) - headerLen,0);
			if ((nRet == SOCKET_ERROR)|| (nRet == 0))
			{  
				int errorCode = WSAGetLastError();
				trySetConnectSocket(true);
				logPrintf(logLevelError_e, "DB_UPDATE", "Receive message header length from socket failed 1!");
				Sleep(1000);
				goto RESTART_LABEL;
			} 
			else
			{
				recvBuffP += nRet;
				headerLen += nRet;
			}
			//logPrintf(logLevelError_e, "DB_UPDATE", "Receive message header length from socket OK!");
		}

		//receive message header
		int headerSize = 0;
		headerLen = recvHeader->msgHeader.headerLen;
		while( headerSize < (headerLen - sizeof(recvHeader->msgHeader.headerLen)) )
		{
			int nRet = recv(sockClient,(char*)recvBuffP,headerLen - headerSize - sizeof(recvHeader->msgHeader.headerLen),0);
			if ((nRet == SOCKET_ERROR)|| (nRet == 0))
			{  
				logPrintf(logLevelError_e, "DB_UPDATE", "Receive message header from socket failed 2!");
				goto RESTART_LABEL;
			} 
			else
			{
				recvBuffP += nRet;
				headerSize += nRet;
			}
		} 
		//memcpy((void*)recvHeader,(void*)recvBuff,headerLen);
		//receive message payload
		
		if(recvHeader->msgHeader.payloadLen != 0)
		{
			int paylaodSize = 0;
			recvHeader->payload = new uint8[recvHeader->msgHeader.payloadLen];// malloc memory to store the payload
			uint8* paylaodPtr = recvHeader->payload;

			while(paylaodSize < recvHeader->msgHeader.payloadLen)
			{
				int nRet = recv(sockClient,(char*)paylaodPtr,recvHeader->msgHeader.payloadLen - paylaodSize,0);
				if ((nRet == SOCKET_ERROR)|| (nRet == 0))
				{  
					logPrintf(logLevelError_e, "DB_UPDATE", "Receive message payload from socket failed 3!");
					delete recvHeader->payload;
					recvHeader->payload = NULL;
					goto RESTART_LABEL;
				} 
				else
				{
					paylaodPtr += nRet;
					paylaodSize += nRet;
				}
			}
		}
		// process message
        int16 msgType = recvHeader->msgHeader.msgTypeID;
        if( (msgType & 0x1000) == 0x0000)
        {     
            int pduIdx;

            switch(msgType)
            {
				case STATUS_UPDATE_RPT_MSG:
				{
					for(pduIdx = 0;pduIdx < recvHeader->msgHeader.numPDUs;pduIdx++)
					{
						int8 tag = recvHeader->payloadHeader.tlvArray[pduIdx].tag;
						int16 value = recvHeader->payloadHeader.tlvArray[pduIdx].value;
						if((1 == tag))
						{
							if(3 == value)
							{
							// delete furniture
							database_gp->resetFurniture();
							// delete all vectors
							database_gp->resetAllVectors();
							}else if(4 == value)
							{
								// delete furniture
								database_gp->resetFurniture();
							}else if(5 == value)
							{
								// delete all vectors
								database_gp->resetAllVectors();
							}
						}
						
					}
					break;
				}
				case UPDATE_REQ_MSG:
                {
                    for(pduIdx = 0;pduIdx < recvHeader->msgHeader.numPDUs;pduIdx++)
                    {
                        int pduLen;
                        uint32 type = (recvHeader->payloadHeader.pduHeader[pduIdx].operate << 16) + recvHeader->payloadHeader.pduHeader[pduIdx].pduType;
                        if(pduIdx == (recvHeader->msgHeader.numPDUs - 1))
                        {
                            pduLen = recvHeader->msgHeader.payloadLen - recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset;
                        }
                        else
                        {
                            pduLen = recvHeader->payloadHeader.pduHeader[pduIdx+1].pduOffset - recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset;
                        }
                        switch(type)
                        {
                            case ADD_NEW_SEGMENT:// add a new segment data
                                database_gp->addSegmentTlv(recvHeader->payload + recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                break;
                            case ADD_NEW_FURNITURE:
                                // fall through
                            case UPDATE_FURNITURE:
                                database_gp->addFurnitureTlv(recvHeader->payload + recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                break;
                            case ADD_ALL_FURNITURE:
                                database_gp->addFurnitureListTlv(recvHeader->payload + recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                break;
                            case ADD_ALL_VECTORLIST:
                                // add a vector list for specified segment.
								historyInfoP.saveHistoryLine(database_gp);
								database_gp->resetAllVectors();
                                database_gp->addAllVectorsInSegTlv(recvHeader->payload + recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                break;
                            case REDUCE_ONE_FURNITURE:
                                database_gp->reduceFurnitureTlv(recvHeader->payload + recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                break;
                        }
                    }
                }
                break;
            }
			delete recvHeader->payload;
			recvHeader->payload = NULL;
        }
    }
    return 0;
}