/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  AppInitCommon.cpp
* @brief 
*
* Change Log:
*      Date                Who             What
*       2015/01/09          Xin Shao        create
*******************************************************************************
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include "AppInitCommon.h"
#include "database.h"   // database
#include "databaseInVehicle.h" // databaseInVehicle
#include "typeDefine.h"
#include "LogInfo.h"
#include "messageProcessClass.h"
#include "saveLinePointInSafe.h"
#include "ImageBuffer.h"
#include "roadScan.h" // readParamRoadScan
#include "VisualizeControl.h"
#include "configure.h"
#include "utils.h"
#include "getSectionID.h" // readSectionConfig
#include "TimeStamp.h"

HANDLE g_readySema_GPS;
HANDLE g_readySema_DiffDet;
HANDLE g_readySema_SocketReady;
HANDLE g_readyEvent_ConnectSocket;

ns_database::databaseInVehicle* database_gp;
ns_historyLine::saveLinePointInSafe	historyInfoP(5,500);
ImageBufferAll imageBuffer;

SOCKET sockServer;
SOCKET sockClient;
SOCKADDR_IN serverAddr;

int g_SocketLen;

uint32 g_VehicleID;
unsigned short g_GpsPort;
unsigned long g_EmulatorIP;
unsigned short g_EmulatorPort;
HANDLE socketMutex;

std::vector<ns_roadScan::Parameters> inParamVec;
std::vector<cv::Mat> HVec, invertHVec,laneHVec;
list<vector<uint32>> g_loopSegList;

int g_CameraPort = 0;

#if(RD_SIGN_DETECT == RD_SIGN_DETECT_COLOR)
    std::vector<ns_detection::Detector_colored *> trafficSignDetectorVec;
#elif(RD_SIGN_DETECT == RD_SIGN_DETECT_WHITE_BLACK)
    std::vector<ns_detection::Detector_blackWhite *> trafficSignDetectorVec;
#endif

list<segAttributes_t> g_segCfgList;
ns_roadsegment::All_RoadSegment *roadSegConfig_gp;

void trySetConnectSocket(bool flag)
{
	DWORD dwWaitResult = WaitForSingleObject(socketMutex,0);
	if (dwWaitResult != WAIT_OBJECT_0 && dwWaitResult != WAIT_TIMEOUT)
	{

	}else
	{
		//ReleaseSemaphore(g_readySema_ConnectSocket, 1 ,NULL);
		SetEvent(g_readyEvent_ConnectSocket);
		ReleaseMutex(socketMutex);
	}
}

void appInitEvents(void)
{
    g_readySema_GPS = CreateSemaphore(NULL,0,10,"semaphore_GPS");
    g_readySema_DiffDet = CreateSemaphore(NULL,0,10,"semaphore_DiffDet");
	g_readySema_SocketReady = CreateSemaphore(NULL,0,10,"semaphore_SocketReady");
	
	g_readyEvent_ConnectSocket = CreateEvent(NULL,TRUE,FALSE,NULL);

	socketMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(socketMutex);
}

void databaseInit()
{
    database_gp = new ns_database::databaseInVehicle();

    void* input = 0;

    database_gp->readDb(&input, ns_database::file_e);
    
}

bool readOverViewPoint(char* fileName,eyeLookAt_t &eye)
{
	FILE* fp = fopen(fileName,"r");
	if(fp == NULL)
	{
		return false;
	}
	char tempBuff[100];
	while(!feof(fp))
	{
		fscanf(fp,"%s",tempBuff);
		if(std::strstr(tempBuff,"overViewPoint:") != NULL)
		{
			fscanf(fp,"%f,%f,%f",&(eye.eyePosition.x),&(eye.eyePosition.y),&(eye.eyePosition.z));
			eye.lookatPosition.x = eye.eyePosition.x;
			eye.lookatPosition.y = 0;
			eye.lookatPosition.z = eye.eyePosition.z;
		}
		if(std::strstr(tempBuff,"cameraPort:") != NULL)
		{
			fscanf(fp,"%d",&g_CameraPort);
		}
	}
	fclose(fp);
	return true;
}

int detectorInit()
{
    string segFilePath;	
    roadSegConfig_gp = new ns_roadsegment::All_RoadSegment();
	
    ns_roadScan::Parameters inParam;
    cv::Mat H_temp;

#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
    inParamVec.assign(1, inParam); 
    HVec.assign(1, H_temp); 
    invertHVec.assign(1, H_temp); 
    laneHVec.assign(1, H_temp);
    segFilePath = "./config/DE_Airport_ParamConfig.xml";
    bool readStatus = ns_roadScan::readParamRoadScan("./config/DE_Airport2.txt", inParamVec[0]);
    readStatus &= readOverViewPoint("./config/DE_Airport_overViewPoint.txt",serverEyeInfo[0]);
#elif (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
    inParamVec.assign(5, inParam); 
    HVec.assign(5, H_temp); 
    invertHVec.assign(5, H_temp); 
    laneHVec.assign(5, H_temp);
    //segFilePath = "./config/DE_Airport_manualSeg.txt";
    segFilePath = "./config/DE_Airport_ParamConfig.xml";
    bool readStatus = ns_roadScan::readParamRoadScan("./config/DE_Airport.txt", inParamVec[0]);
    readStatus &= ns_roadScan::readParamRoadScan("./config/DE_Airport2.txt", inParamVec[1]);
	readStatus &= ns_roadScan::readParamRoadScan("./config/DE_Airport3.txt", inParamVec[2]);
    readStatus &= ns_roadScan::readParamRoadScan("./config/DE_Airport4.txt", inParamVec[3]);
	readStatus &= ns_roadScan::readParamRoadScan("./config/DE_AirportTcross.txt", inParamVec[4]);
    readStatus &= readOverViewPoint("./config/DE_Airport_overViewPoint.txt",serverEyeInfo[0]);
#elif (RD_LOCATION == RD_GERMAN_LEHRE)
    inParamVec.assign(1, inParam);
    HVec.assign(1, H_temp); 
    invertHVec.assign(1, H_temp); 
    laneHVec.assign(1, H_temp);
    segFilePath = "./config/DE_Lehre_manualSeg.txt";
    bool readStatus = ns_roadScan::readParamRoadScan("./config/DE_Lehre.txt", inParamVec[0]);
	readStatus &= readOverViewPoint("./config/DE_Lehre_overViewPoint.txt",serverEyeInfo[0]);
#elif (RD_LOCATION == RD_GERMAN_LEHRE2)
    inParamVec.assign(1, inParam);
    HVec.assign(1, H_temp); 
    invertHVec.assign(1, H_temp); 
    laneHVec.assign(1, H_temp);
    segFilePath = "./config/DE_Lehre_manualSeg.txt";
    bool readStatus = ns_roadScan::readParamRoadScan("./config/DE_Lehre2.txt", inParamVec[0]);
	readStatus &= readOverViewPoint("./config/DE_Lehre_overViewPoint.txt",serverEyeInfo[0]);
#elif (RD_LOCATION == RD_US_DETROIT)
    inParamVec.assign(1, inParam);
    HVec.assign(1, H_temp); 
    invertHVec.assign(1, H_temp); 
    laneHVec.assign(1, H_temp);
    segFilePath = "./config/US_Detroit_manualSeg.txt";
	bool readStatus = ns_roadScan::readParamRoadScan("./config/US_Detroit.txt", inParamVec[0]);
	readStatus &= readOverViewPoint("./config/US_Detroit_overViewPoint.txt",serverEyeInfo[0]);
#elif (RD_LOCATION == RD_US_PALO_ALTO)
    inParamVec.assign(1, inParam);
    HVec.assign(1, H_temp); 
    invertHVec.assign(1, H_temp); 
    laneHVec.assign(1, H_temp);
    segFilePath = "./config/US_Palo_Alto_manualSeg.txt";
	bool readStatus = ns_roadScan::readParamRoadScan("./config/US_Palo_Alto.txt", inParamVec[0]);
	readStatus &= readOverViewPoint("./config/US_Palo_Alto_overViewPoint.txt",serverEyeInfo[0]);
#endif
	serverEyeInfo[1] = serverEyeInfo[0];

#if ((RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE) ||(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT))    
    list<vector<uint32>> loopseg_List;
    bool segInfoFlag = roadSegConfig_gp->AnalysisSegment(segFilePath.c_str());
	roadSegConfig_gp->getRoadSegCfg_database(g_segCfgList);
    roadSegConfig_gp->getRoadLoopSegId(loopseg_List);
    roadSegConfig_gp->setRoadLoopFlag(loopseg_List, g_segCfgList);
#else
    bool segInfoFlag = readSectionConfig(segFilePath,g_segCfgList);
#endif
    if((!readStatus) || (!segInfoFlag))
    {
        return -1;
    }else
    {
#if(RD_SIGN_DETECT != RD_SIGN_DETECT_OFF)
        ns_detection::loadModels();
#if(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
        ns_detection::Detector_colored * trafficSignDetectorTemp;
        trafficSignDetectorTemp = new ns_detection::Detector_colored(0.5,inParamVec[0].distancePerPixel,300); // 300: airport2
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
#elif(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
        ns_detection::Detector_colored * trafficSignDetectorTemp;
        trafficSignDetectorTemp = new ns_detection::Detector_colored(0.5,inParamVec[0].distancePerPixel,300);
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
        trafficSignDetectorTemp = new ns_detection::Detector_colored(0.5,inParamVec[1].distancePerPixel,300);
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
		trafficSignDetectorTemp = new ns_detection::Detector_colored(0.5,inParamVec[2].distancePerPixel,300);
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
		trafficSignDetectorTemp = new ns_detection::Detector_colored(0.5,inParamVec[3].distancePerPixel,300);
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
		trafficSignDetectorTemp = new ns_detection::Detector_colored(0.5,inParamVec[4].distancePerPixel,300);
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
#elif(RD_LOCATION == RD_GERMAN_LEHRE)
        ns_detection::Detector_blackWhite * trafficSignDetectorTemp;
        trafficSignDetectorTemp = new ns_detection::Detector_blackWhite(0.5,inParamVec[0].distancePerPixel,100); // not set the horonzition line
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
#elif(RD_LOCATION == RD_GERMAN_LEHRE2)
        ns_detection::Detector_colored * trafficSignDetectorTemp;
        trafficSignDetectorTemp = new ns_detection::Detector_colored(0.5,inParamVec[0].distancePerPixel,100);
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
#elif(RD_LOCATION == RD_US_DETROIT)
        ns_detection::Detector_blackWhite * trafficSignDetectorTemp;
        trafficSignDetectorTemp = new ns_detection::Detector_blackWhite(0.5,inParamVec[0].distancePerPixel,260); // 260: detroit
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
#elif(RD_LOCATION == RD_US_PALO_ALTO)
        ns_detection::Detector_blackWhite * trafficSignDetectorTemp;
        trafficSignDetectorTemp = new ns_detection::Detector_blackWhite(0.5,inParamVec[0].distancePerPixel,0,300); // not set the horonzition line
        trafficSignDetectorVec.push_back(trafficSignDetectorTemp);
#else

#endif
#endif // #if(RD_SIGN_DETECT != RD_SIGN_DETECT_OFF)

        // Calculate H and H inverse for road scan and traffic sign detection
        ns_roadScan::calHAndInvertH(inParamVec[0], HVec[0], invertHVec[0], laneHVec[0]);
#if(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
        ns_roadScan::calHAndInvertH(inParamVec[1], HVec[1], invertHVec[1], laneHVec[1]);
		ns_roadScan::calHAndInvertH(inParamVec[2], HVec[2], invertHVec[2], laneHVec[2]);
		ns_roadScan::calHAndInvertH(inParamVec[3], HVec[3], invertHVec[3], laneHVec[3]);
        ns_roadScan::calHAndInvertH(inParamVec[4], HVec[4], invertHVec[4], laneHVec[4]);
#endif
        return 0;
    }
}

//void getVehicleID(uint64* vehicleID)
//{
//    char buff[100];
//    FILE* f = fopen("./config//VehicleID.txt", "r");
//    fscanf(f,"%s",buff);//server IP
//    *vehicleID = atoi(buff);
//}

bool startSocket()
{
    SOCKADDR_IN clientAddr;
	int sockLen = sizeof(clientAddr);
    FILE* fp = fopen("./config//NewcoVehicleConfig.txt", "r");   
    int nRet;

    // Initialize WinSock and check version  
    WORD wVersionRequested = MAKEWORD(2,0);  
    WSADATA wsaData;   
    //logPrintf(logLevelError_e,"COMM","shaoxin");
    nRet = WSAStartup(wVersionRequested, &wsaData);  
    if (wsaData.wVersion != wVersionRequested)  
    {     
        logPrintf(logLevelError_e,"COMM","The window socket version does not supoorted!");  
        return false;  
    }  
	// get host ip and port
    char hostname[256];  
    nRet=gethostname(hostname,sizeof(hostname));  
    if (nRet==SOCKET_ERROR)  
    {  
        return false;  
    }  
    HOSTENT* host=gethostbyname(hostname);  
	char clientBuff[100];
    if (host==NULL)  
    {  
        return false;  
    }
	strcpy(clientBuff,inet_ntoa(*(in_addr*)*host->h_addr_list));
	clientAddr.sin_family           = AF_INET;
	clientAddr.sin_addr.S_un.S_addr = inet_addr(clientBuff);//inet_addr("127.0.0.1");

	g_SocketLen = sizeof(clientAddr);
	// get the serverIP and port, GPS port and vehicle ID
	while(!feof(fp))
	{
		char tempBuff[100];
		serverAddr.sin_family           = AF_INET;
		fscanf(fp,"%s",tempBuff);
		if(std::strstr(tempBuff,"serverIP:") != NULL)
		{
			serverAddr.sin_addr.S_un.S_addr = inet_addr(&tempBuff[strlen("serverIP:")]);
			continue;
		}
		int sPort;
		if(std::strstr(tempBuff,"serverPort:") != NULL)
		{	
			sPort = atoi(&tempBuff[strlen("serverPort:")]);
			serverAddr.sin_port = htons(sPort);
			continue;
		}
		int	cPort;
		if(std::strstr(tempBuff,"clientPort:") != NULL)
		{	
			cPort = atoi(&tempBuff[strlen("clientPort:")]);
			clientAddr.sin_port = htons(cPort);
			continue;
		}
		if(std::strstr(tempBuff,"gpsPort:") != NULL)
		{	
			cPort = atoi(&tempBuff[strlen("gpsPort:")]);
			g_GpsPort = htons(cPort);
			continue;
		}
		if(std::strstr(tempBuff,"vehicleId:") != NULL)
		{	
			g_VehicleID = atoi(&tempBuff[strlen("vehicleId:")]);
			continue;
		}
		if(std::strstr(tempBuff,"emulatorIP:") != NULL)
		{
			g_EmulatorIP = inet_addr(&tempBuff[strlen("emulatorIP:")]);
		}
		if(std::strstr(tempBuff,"emulatorPort:") != NULL)
		{
			 cPort = atoi(&tempBuff[strlen("emulatorPort:")]);
			 g_EmulatorPort = htons(cPort);
		}
	}

	fclose(fp);
    //open the UDP socket client, connect to server side
    sockClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockClient == INVALID_SOCKET)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
        return false;  
    }  

	//nRet = bind(sockClient,(SOCKADDR*)&clientAddr,sizeof(struct sockaddr));
	//if (nRet == SOCKET_ERROR)  
    //{  
	//	int errorCode = WSAGetLastError();
    //    logPrintf(logLevelError_e,"COMM","Bind socket failed!");   
    //    return -1;  
    //} 


	nRet = connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr));
	if(nRet == SOCKET_ERROR)
	{
		int errorCode = WSAGetLastError();
		logPrintf(logLevelFatal_e, "COMM", "Connect socket failed!");
		//closesocket(sockClient);
		//WSACleanup();
		return false;
	}
    return true;

}

unsigned int __stdcall Thread_ReconnectSocket(void *data)
{
	while(1)
	{
		WaitForSingleObject(g_readyEvent_ConnectSocket, INFINITE);
		WaitForSingleObject(socketMutex,INFINITE);
		
		RD_ADD_TS(tsFunId_eThread_ReConSocket,1);
		while(SOCKET_ERROR == connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr)))
		{
			int errorCode = WSAGetLastError();
			if(errorCode == 10056 || errorCode == 10038)
			{
				closesocket(sockClient);

				while(INVALID_SOCKET == socket(AF_INET, SOCK_STREAM, IPPROTO_TCP))
				{
					//logPrintf(logLevelFatal_e, "COMM", "Creating socket failed!");
				}
			}
			//logPrintf(logLevelFatal_e, "COMM", "Connect socket failed!");
			Sleep(50);
		}
		logPrintf(logLevelInfo_e, "COMM", "Connect socket!",FOREGROUND_BLUE);
		RD_ADD_TS(tsFunId_eThread_ReConSocket,2);

		ResetEvent(g_readyEvent_ConnectSocket);
		ReleaseMutex(socketMutex);
	}

}

void sendDatabaseVersion()
{
	
	//int sendLen;
 
	#define SEND_MAX_BYTE_NUM 10000
	char sendBuff[SEND_MAX_BYTE_NUM];
	messageProcessClass statusMessage;
	diffRptMsg_t* statusRptMsgPtr = statusMessage.getDiffRptMsg();
	uint16 headerLen = sizeof(statusRptMsgPtr->msgHeader) + sizeof(statusRptMsgPtr->payloadHeader.pduHeader[0]);
	statusMessage.setMsgHeader((uint32*)statusRptMsgPtr,STATUS_UPDATE_RPT_MSG,g_VehicleID,highLevel_e,0,1,headerLen);
	statusRptMsgPtr->payloadHeader.tlvArray[0].tag = 3;
	statusRptMsgPtr->payloadHeader.tlvArray[0].len = 2;
	statusRptMsgPtr->payloadHeader.tlvArray[0].value = 0xfffe;
	
	int nRet = send(sockClient,(char*)statusRptMsgPtr,headerLen,0);//,(SOCKADDR*)&serverAddr,g_SocketLen);//send response message data
	if ((nRet == SOCKET_ERROR) || (nRet == 0))
	{
		logPrintf(logLevelInfo_e, "COMM", "Send message to server failed!",FOREGROUND_BLUE);
	}
	else
	{
		logPrintf(logLevelInfo_e, "COMM", "<<<< Send message to server OK",FOREGROUND_BLUE);
	}
}

uint8 getLoopIdxFromFurInLoopIdx(IN int inParamIndex)
{
	uint8 loopIndex = 0;
#if(RD_GERMAN_MUNICH_AIRPORT_LARGE == RD_LOCATION)
    switch(inParamIndex)
    {
    case 0:
        loopIndex = 0; // big loop;
        break;
    case 1:
        loopIndex = 1; // small loop;
        break;
    case 2:
        loopIndex = 0; // big loop;
        break;
    case 3:
        loopIndex = 0; // big loop
        break;
	case 4:
		loopIndex = 2; // T road
		break;
    default:
        loopIndex = 0;
        break;
        
    }
#endif
    return loopIndex;
}