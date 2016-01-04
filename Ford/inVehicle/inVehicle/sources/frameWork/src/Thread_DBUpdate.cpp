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
#include "TimeStamp.h"
#include "Signal_Thread_Sync.h"

#pragma comment(lib, "ws2_32.lib") 


using ns_historyLine::saveLinePointInSafe;

unsigned int __stdcall Thread_DBUpdate(void *data)
{
	startSocket();
	sendDatabaseVersion();
	ReleaseSemaphore(g_readySema_SocketReady, 1 ,NULL);
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
		RD_ADD_TS(tsFunId_eThread_Update,1);
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

		RD_ADD_TS(tsFunId_eThread_Update,2);
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
		RD_ADD_TS(tsFunId_eThread_Update,3);
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
		RD_ADD_TS(tsFunId_eThread_Update,4);
        // This is the flag to indicate if the PDU is the first PDU of ADD_ALL_FURNITURE
        bool firstAddAllFurPduFlag = true;

		// process message
        int16 msgType = recvHeader->msgHeader.msgTypeID;
        if( (msgType & 0x1000) == 0x0000)
        {     
            int pduIdx;

            switch(msgType)
            {
				case STATUS_UPDATE_RPT_MSG:
				{
					RD_ADD_TS(tsFunId_eThread_Update,5);
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
					RD_ADD_TS(tsFunId_eThread_Update,6);
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
                                // This type of PDU only carry fur in one section.  So only clear the fur DB before the first PDU.
                                if(firstAddAllFurPduFlag)
                                {
                                    database_gp->resetFurniture();
                                    firstAddAllFurPduFlag = false;
                                }
                                database_gp->addFurnitureListTlv(recvHeader->payload + recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
                                break;
                            case ADD_ALL_VECTORLIST:
                                // add a vector list for specified segment.
								// historyInfoP.saveHistoryLine(database_gp);
								//database_gp->resetAllVectors();// Clear each section in function below -- addVectorsInSegTlv
                                database_gp->addVectorsInSegTlv(recvHeader->payload + recvHeader->payloadHeader.pduHeader[pduIdx].pduOffset,pduLen);
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
			RD_ADD_TS(tsFunId_eThread_Update,14);
        }
    }//end while(1)
    return 0;
}