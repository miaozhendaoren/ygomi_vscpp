/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_ImageSensorCollector.cpp
* @brief get the image data from Image sensors and get current GPS information 
*        add circle buffer to save continuous image frame, it is depend on ImageSensor's configure
*
* Change Log:
*      Date                Who             What
*      2015/06/01         Xin Shao        Create
*******************************************************************************
*/
#include <stdlib.h>
#include <Windows.h>
#include <MMSystem.h>  //for window timer.
#include <time.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
#include <math.h>
#include <gl/glut.h>

#include "configure.h"
#include "AppInitCommon.h"
#include "ImageBuffer.h"
#include "CacheBuffer.h"
#include "NEMA_GPGGA_Proc.h"
#include "saveLinePointInSafe.h"
#include "Signal_Thread_Sync.h"
#include "Thread_ImageSensorCollector.h"
#include "LogInfo.h"
#include "VisualizeControl.h"
#include "TimeStamp.h"

using namespace ns_database;
using namespace cv;

VideoCapture capture;

#define LAST_FRAME_NUM 100
#if(RD_MODE == RD_CAMERA_MODE) 
    CNEMA_GPGGA_PROC gNemaGpggaProc;
    #define NC_UDP_GPS_DATA_BUF_LEN        (1500)
    char nc_udpGpsBuffer[NC_UDP_GPS_DATA_BUF_LEN];
    CacheBuffer cacheBuffer;
    volatile SOCKET g_ServerSockUDP;
#elif(RD_MODE == RD_VIDEO_BUFFER_MODE)
    char aviNames[300][100];
    char gpsNames[300][100];
    int inParamIdxs[300];
    HANDLE g_readySema_VideoReader;
    unsigned int timeDelay;
    volatile SOCKET g_EmulatorSockUDP;
    SOCKADDR_IN emulatorAddr;
#elif(RD_MODE == RD_VIDEO_LOAD_MODE)
    char aviNames[300][100];
    char gpsNames[300][100];
    int inParamIdxs[300];
#endif

#if(RD_MODE == RD_CAMERA_MODE)

cv::Size showSize;

unsigned int getSysTimeMs(void)
{
	SYSTEMTIME sysTime;
	GetLocalTime(&sysTime);
	unsigned int time_ms = sysTime.wMilliseconds + sysTime.wSecond*1000 
		+ sysTime.wMinute*60*1000 + sysTime.wHour*60*60*1000;
	return time_ms;
}

bool openImageSensor_Cam(int device,VideoCapture& capture ,int &numFrame)
{
	capture.open(device);
	if(!capture.isOpened())
	{
		return false;
	}else
	{
		numFrame = capture.get(CV_CAP_PROP_FRAME_COUNT);
		//capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		//numFrame = capture.get(CV_CAP_PROP_POS_FRAMES);
		//double fps = capture.get(CV_CAP_PROP_FPS);
		capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
		capture.set(CV_CAP_PROP_FPS ,30);
		capture.set(CV_CAP_PROP_FRAME_WIDTH ,1280);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT ,720);
		showSize.height = 360;
		showSize.width  = 640;
	}
	return true;
}

bool imageSensorCollect_Cam(VideoCapture& capture,Mat& image)
{
	if(!capture.read(image))
	{
		return false;
	}
	return true;
}

void gpsSensorCollect_UDP(void)
{
	sockaddr_in from;
	int fromlen = sizeof(from);
	
	//receive GPS data in NEMA 0183 GPGGA
	int numRecv = recvfrom(g_ServerSockUDP, nc_udpGpsBuffer, NC_UDP_GPS_DATA_BUF_LEN, 0, (struct sockaddr*)&from, &fromlen);

	//cacheBuffer.swatchBuffer();
	if (numRecv != SOCKET_ERROR)
    {
		//update the receive system time.
        unsigned int systime = getSysTimeMs();
		cacheBuffer.swatchBuffer();

        if(gNemaGpggaProc.NEMA_GPGGA_checksum(nc_udpGpsBuffer))
        {
			gGpsInfo.stPre = gGpsInfo.st;
			gGpsInfo.st = systime;
            gNemaGpggaProc.NEMA_GPGGA_parser(nc_udpGpsBuffer);
        }

        //printf("receive UDP data: %s\n", nc_udpGpsBuffer);
		//printf("Lat: %f\n",gGpsInfo.dLatitude);
		//printf("longi: %f\n",gGpsInfo.dLongitude);
		//printf("alti: %f\n",gGpsInfo.altitude);
		//printf("durantion is %d\n",(gGpsInfo.st - gGpsInfo.stPre));
    }

}

//void imageTimer(int value)
void WINAPI imageTimer(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	if(11 == dwUser)
	{
		static int imageCount = 0;
		imageCamera_t tempImage;
		//glutTimerFunc((unsigned int)(33),&imageTimer,3);

		//get the 
		cacheBuffer.lockCacheBuffer();
		if(imageSensorCollect_Cam(capture,tempImage.image))
		{
			tempImage.st = getSysTimeMs();

			cacheBuffer.addImage(tempImage);
			cacheBuffer.releaseCacheBuffer();

			imageCount++;
			if(2 == imageCount)
			{
				imageCount = 0;
				cv::namedWindow("image",CV_WINDOW_NORMAL);
				cv::resize(tempImage.image,tempImage.image,showSize);
				cv::imshow("image",tempImage.image);
				cv::waitKey(1);
			}
			RD_ADD_TS(tsFunId_eImageTimerIsr,1);
		}else
		{
			cacheBuffer.releaseCacheBuffer();
			RD_ADD_TS(tsFunId_eImageTimerIsr,2);
		}
	}
}

void insertGps(point3D_t& startPoint, point3D_t& endPoint, double ratio, point3D_t& outPoint)
{
	double dif_lat = endPoint.lat - startPoint.lat;
	double dif_lon = endPoint.lon - startPoint.lon;
	double dif_alt = endPoint.alt - startPoint.alt;

	outPoint.lat = startPoint.lat + dif_lat*ratio;
	outPoint.lon = startPoint.lon + dif_lon*ratio;
	outPoint.alt = startPoint.alt + dif_alt*ratio;
}

bool InitGpsSocket_UDP()
{
	//UDP connect to receive the gps information
	// zero the sockaddr_in structure
	sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = g_GpsPort;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
	//open the UDP socket server, and listening
    if((g_ServerSockUDP = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
    {
        return false;
    }

    if( SOCKET_ERROR == bind(g_ServerSockUDP, (struct sockaddr*)&local, sizeof(local)) )
    {
        return false;
    }
	return true;
}

void coordinateChange(point3D_t* changePoint,point3D_t* standPoint,point3D_t *outPoint)
{
	GLfloat dif_x = changePoint->lat - standPoint->lat;
	GLfloat dif_y = changePoint->lon - standPoint->lon;
	GLfloat dif_z = changePoint->alt - standPoint->alt;
	float latitude = (standPoint->lat)*PI/180;

	outPoint->lat = dif_x*COEFF_DD2METER;  //latitude
	outPoint->lon = dif_y*(111413*cos(latitude)-94*cos(3*latitude));  //longitude
	outPoint->alt = 0;//dif_z;

}

unsigned int __stdcall Thread_ImageSensorCollector(void *data)
{
	int numFrame;
	bool init = true;
	point3D_t currentGps;
	point3D_t preGps;
	RD_ADD_TS(tsFunId_eThread_ImageCollect,1);

	if(!openImageSensor_Cam(g_CameraPort,capture ,numFrame))
	{
		logPrintf(logLevelInfo_e, "IMAGE_COLLECTOR", "Open Camera failed!");
		return -1;
	}
	
    imageBuffer.setImageSize(capture.get(CV_CAP_PROP_FRAME_WIDTH),
        capture.get(CV_CAP_PROP_FRAME_HEIGHT));

	WaitForSingleObject(g_readySema_SocketReady, INFINITE);

	if(!InitGpsSocket_UDP())
	{
		logPrintf(logLevelInfo_e, "IMAGE_COLLECTOR", "Open GPS UDP socket failed!");
		return -1;
	}

	//start the timer to get the camera image
	//glutTimerFunc((unsigned int)(50),&imageTimer,3);
	MMRESULT timer_id;
	timer_id = timeSetEvent(50,1,(LPTIMECALLBACK)imageTimer, DWORD(11),TIME_PERIODIC);
	RD_ADD_TS(tsFunId_eThread_ImageCollect,2);
	while(1)
	{
		//recevie the GPS information
		RD_ADD_TS(tsFunId_eThread_ImageCollect,3);
		gpsSensorCollect_UDP();

		gGpsInfo.inParamIdxsPrePre = gGpsInfo.inParamIdxsPre;
		gGpsInfo.inParamIdxsPrePre = gGpsInfo.inParamIdxs;
#if(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
		gGpsInfo.inParamIdxs = 1;
#else
		gGpsInfo.inParamIdxs = 0;
#endif

		RD_ADD_TS(tsFunId_eThread_ImageCollect,4);
		if(init)
		{
			preGps.alt = gGpsInfo.altitude;
			preGps.lat = gGpsInfo.dLatitude;
			preGps.lon = gGpsInfo.dLongitude;
			init = false;
		}else
		{
			//insert the GPS information to each image
			vector<imageCamera_t> *imageTempVec;
			cacheBuffer.getBackendBuffer(&imageTempVec);
			int numImage = imageTempVec->size();
			double totalTime = (gGpsInfo.st > gGpsInfo.stPre)?(gGpsInfo.st - gGpsInfo.stPre):(0xffffffff-(gGpsInfo.stPre -gGpsInfo.st)+1);
			if((totalTime < 800)||(totalTime > 1200))
			{
				if(imageBuffer.getCurrentImageNum() < LAST_FRAME_NUM)
				{
					imageBuffer.cleanCurrentBuffer();
				}else
				{
					imageBuffer.setReadyFlag();
				}
				preGps.alt = gGpsInfo.altitude;
				preGps.lat = gGpsInfo.dLatitude;
				preGps.lon = gGpsInfo.dLongitude;
				continue;
			}

			point3D_t startPoint;
			point3D_t endPoint;
			startPoint.alt = gGpsInfo.altitudePre;
			startPoint.lat = gGpsInfo.dLatitudePre;
			startPoint.lon = gGpsInfo.dLongitudePre;

			endPoint.alt = gGpsInfo.altitude;
			endPoint.lat = gGpsInfo.dLatitude;
			endPoint.lon = gGpsInfo.dLongitude;

			point3D_t outPoint;
			coordinateChange(&endPoint,&startPoint,&outPoint);
			if(sqrt((outPoint.lat*outPoint.lat)+(outPoint.lon*outPoint.lon)) > 100)
			{
				if(imageBuffer.getCurrentImageNum() < LAST_FRAME_NUM)
				{
					imageBuffer.cleanCurrentBuffer();
				}else
				{
					imageBuffer.setReadyFlag();
				}
				preGps.alt = gGpsInfo.altitude;
				preGps.lat = gGpsInfo.dLatitude;
				preGps.lon = gGpsInfo.dLongitude;
				continue;
			}

			if(imageTempVec->empty())
			{
				preGps.alt = gGpsInfo.altitude;
				preGps.lat = gGpsInfo.dLatitude;
				preGps.lon = gGpsInfo.dLongitude;
				continue;
			}
			//cv::namedWindow("image",CV_WINDOW_NORMAL);
			//cv::imshow("image",(*imageTempVec->begin()).image);
			//cv::waitKey(1);

			for(std::vector<imageCamera_t>::iterator imageIter = imageTempVec->begin();
				imageIter != imageTempVec->end();
				++imageIter)
			{
				//calc each image's gps
				double timeOffset = ((*imageIter).st > gGpsInfo.stPre)?((*imageIter).st - gGpsInfo.stPre):(0xffffffff - (gGpsInfo.stPre - (*imageIter).st)+1);
				double ratio = timeOffset/totalTime;
				insertGps(startPoint, endPoint, ratio, currentGps);

				coordinateChange(&currentGps,&preGps,&outPoint);
				//if(sqrt((outPoint.lat*outPoint.lat)+(outPoint.lon*outPoint.lon)) > 10)

				//save image to imagebuffer
				imageBuffer.setSaveFlag();
#if(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
#if(ON == RD_REPORT_RESULT)
				imageBuffer.addImage((*imageIter).image,currentGps,preGps,0,0, 1); // camera mode only use small loop for airport area
#else
				imageBuffer.addImage((*imageIter).image,currentGps,preGps,0,0, 3);
#endif
#else
                imageBuffer.addImage((*imageIter).image,currentGps,preGps,0,0, 0);
#endif
				preGps = currentGps;
			}
			RD_ADD_TS(tsFunId_eThread_ImageCollect,14);
		}//end if(init)

	}//end while(1)
	timeKillEvent(timer_id);  
}
#elif(RD_MODE == RD_VIDEO_BUFFER_MODE)
std::string GetFileNameByFilePath(const std::string filepath)
{
	std::string filename;
	if(filepath.size() < 0)
	return "";
	string::size_type ix = filepath.find_last_of('/');
	if(ix != string::npos)
	{
		return filepath.substr(ix+1,filepath.size()-ix);
	}
	else
	{
		string::size_type ix = filepath.find_last_of('\\');
		if(ix != string::npos)
		{
			return filepath.substr(ix+1,filepath.size()-ix);
		}
	}

	return "";
}

void updateGpsInfo(point3D_t gpsPoint, int inParam)
{
	gGpsInfo.dLatitudePrePre = gGpsInfo.dLatitudePre;
	gGpsInfo.dLongitudePrePre = gGpsInfo.dLongitudePre;
	gGpsInfo.altitudePrePre = gGpsInfo.altitudePre;
	gGpsInfo.inParamIdxsPrePre = gGpsInfo.inParamIdxsPre;

	gGpsInfo.dLatitudePre = gGpsInfo.dLatitude;
	gGpsInfo.dLongitudePre = gGpsInfo.dLongitude;
	gGpsInfo.altitudePre = gGpsInfo.altitude;
	gGpsInfo.inParamIdxsPre = gGpsInfo.inParamIdxs;

	gGpsInfo.dLatitude = gpsPoint.lat;
	gGpsInfo.dLongitude = gpsPoint.lon;
	gGpsInfo.altitude = gpsPoint.alt;
	gGpsInfo.inParamIdxs = inParam;
}

bool openVideoFile(char* videoFileName,VideoCapture& capture ,int &numFrame)
{
	capture.open(videoFileName);
	if(!capture.isOpened())
	{
		return false;
	}else
	{
		numFrame = capture.get(CV_CAP_PROP_FRAME_COUNT);
		//capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		//numFrame = capture.get(CV_CAP_PROP_POS_FRAMES);
		double fps = capture.get(CV_CAP_PROP_FPS);
		capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0);

	}
	return true;
}

bool readImageAndGps(VideoCapture& capture, FILE* fp, Mat& image, point3D_t& gpsPoint)
{
	if(!capture.read(image))
	{
		return false;
	}

	if(!feof(fp))
	{
		fscanf(fp,"%lf,%lf\n",&gpsPoint.lat,&gpsPoint.lon);
		gpsPoint.alt = 0;
	}else
	{
		return false;
	}
	return true;
}

int generateRandFileIdx(int numFiles)
{
	int randNum;
	randNum = rand();
	return (randNum%numFiles);
}

void getSpeedAndDirectionInfo(point3D_t currentGps, point3D_t preGps, double fps, float &speed, float &direction)
{
	float dif_x = currentGps.lat - preGps.lat;
	float dif_y = currentGps.lon - preGps.lon;
	float dif_z = currentGps.alt - preGps.alt;
	float latitude = (preGps.lat)*PI/180;

	float x = dif_x*COEFF_DD2METER;  //latitude
	float y = dif_y*(111413*cos(latitude)-94*cos(3*latitude));  //longitude
	
	speed = sqrt(x*x + y*y)*fps;
	
	if(y == 0)
	{
		direction = 0;
	}else{
	direction = x/y;     //based on latitude
	}
}

bool InitTimeOffsetSocket_UDP()
{
	g_EmulatorSockUDP = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_EmulatorSockUDP == INVALID_SOCKET)  
    {  
        logPrintf(logLevelFatal_e, "COMM", "Creating emulator UDP socket failed!");
        return false;
    }
	emulatorAddr.sin_family = AF_INET;
	emulatorAddr.sin_addr.S_un.S_addr = g_EmulatorIP;
	emulatorAddr.sin_port = g_EmulatorPort;
	return true;
}

//void imageTimer(int value)
void WINAPI imageTimer(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	if(11 == dwUser)
	{
		RD_ADD_TS(tsFunId_eImageTimerIsr,1);
		ReleaseSemaphore(g_readySema_VideoReader, 1 ,NULL);
	}
}

unsigned int __stdcall Thread_ImageSensorCollector(void *data)
{
	int numFiles;
	int idxFile = 0;
	int numFrame;
	int totalNumFrame;
	int videoSpeedCnt = 0;
	double fps;
	int counter = 0;
	point3D_t currentGps;
	point3D_t preGps;
	g_readySema_VideoReader = CreateSemaphore(NULL,0,10,"semaphore_VideoReader");
	cv::Size showSize;
	cv::Mat image;
	RD_ADD_TS(tsFunId_eThread_ImageCollect,1);

	WaitForSingleObject(g_readySema_SocketReady, INFINITE);
	InitTimeOffsetSocket_UDP();
    
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/DE_Airport2_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
    vector<string> aviGpsFileName(5, "");
    aviGpsFileName[0] = "./config/DE_Airport_aviGpsFiles.txt";
    aviGpsFileName[1] = "./config/DE_Airport2_aviGpsFiles.txt";
	aviGpsFileName[2] = "./config/DE_Airport3_aviGpsFiles.txt";
    aviGpsFileName[3] = "./config/DE_Airport4_aviGpsFiles.txt";
	aviGpsFileName[4] = "./config/DE_AirportTcross_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_GERMAN_LEHRE)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/DE_Lehre_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_GERMAN_LEHRE2)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/DE_Lehre2_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_US_DETROIT)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/US_Detroit_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_US_PALO_ALTO)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/US_Palo_Alto_aviGpsFiles.txt";
#endif

    int readIdx = 0;
    for(int aviGpsFileIdx = 0; aviGpsFileIdx < aviGpsFileName.size(); ++aviGpsFileIdx)
    {
        FILE* fp = fopen(aviGpsFileName[aviGpsFileIdx].c_str(), "r");
	
	    if(fp == NULL)
	    {
		    //printf("cannot open the aviGpsFiles.txt file\n");
		    logPrintf(logLevelError_e, "ImageCollector", "cannot open the aviGpsFiles.txt file");
		    return -1;
	    }
	    
	    while(!feof(fp))
	    {
            int returnVal;

            if((readIdx&0x1) == 0)
            {
                returnVal = fscanf(fp,"%s",aviNames[readIdx>>1]);
            }else
            {
                returnVal = fscanf(fp,"%s",gpsNames[readIdx>>1]);
            }

            if(returnVal > 0)
            {
                inParamIdxs[readIdx>>1] = aviGpsFileIdx;

		        readIdx++;
            }
	    }

	    fclose(fp);
    }

    numFiles = (readIdx>>1);

	srand((unsigned)time(NULL));
	//idxFile = generateRandFileIdx(numFiles);
	//printf("select video %d\n",idxFile);
	{
		std::stringstream msgStr;
        msgStr << "select video " << idxFile;
		logPrintf(logLevelInfo_e, "ImageCollector", msgStr.str(),FOREGROUND_BLUE|FOREGROUND_GREEN);
	}

	FILE* gpsFile = fopen(gpsNames[idxFile],"r");
	if(NULL == gpsFile)
	{
		std::stringstream msgStr;
        msgStr << "cannot open file " << gpsNames[idxFile];
		logPrintf(logLevelInfo_e, "ImageCollector",msgStr.str());
		return -1;
	}
	fseek(gpsFile, 0, SEEK_SET);
	fscanf(gpsFile,"%lf,%lf\n",&preGps.lat,&preGps.lon);
	preGps.alt = 0;
	fseek(gpsFile, 0, SEEK_SET);

	if( !openVideoFile(aviNames[idxFile],capture ,numFrame))
	{
		//printf("can't open file: %s",aviNames[idxFile]);
		std::stringstream msgStr;
        msgStr << "cannot open file " << aviNames[idxFile];
		logPrintf(logLevelError_e, "ImageCollector", msgStr.str());
		return -1;
	}
	totalNumFrame = numFrame;
	fps = capture.get(CV_CAP_PROP_FPS);  //get the frames per seconds of the video

	timeDelay = (unsigned int)(1000/fps);
	//glutTimerFunc(timeDelay,&imageTimer,3);
	MMRESULT timer_id;
	timer_id = timeSetEvent(timeDelay,1,(LPTIMECALLBACK)imageTimer, DWORD(11),TIME_PERIODIC);

    int inParamIdx = inParamIdxs[idxFile];

	showSize.height = capture.get(CV_CAP_PROP_FRAME_HEIGHT) * inParamVec[inParamIdx].imageScaleHeight;
	showSize.width = capture.get(CV_CAP_PROP_FRAME_WIDTH) * inParamVec[inParamIdx].imageScaleWidth;

    imageBuffer.setImageSize(capture.get(CV_CAP_PROP_FRAME_WIDTH),
	    capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    imageBuffer.setInParamIdx(inParamIdx);

    const int MAX_FRAME_INTERVAL = 1;
    int frameInterval = 0;
	RD_ADD_TS(tsFunId_eThread_ImageCollect,2);

	bool addFlag = true;
	while(1)
	{
		//cv::Mat image;
		RD_ADD_TS(tsFunId_eThread_ImageCollect,3);
		if(numFrame <= 0)
		{

			idxFile++;
			if(idxFile >= numFiles)
			{
				idxFile = 0;
#if(ON == RD_PAUSE_BUFFER_FULL)
				logPrintf(logLevelNotice_e, "ImageCollector", "play video finished!");
				return -1;
#endif
			}

			//idxFile = generateRandFileIdx(numFiles);
			//printf("select video %d\n",idxFile);
			{
				std::stringstream msgStr;
				msgStr << "select video " << idxFile;
				logPrintf(logLevelInfo_e, "ImageCollector", msgStr.str(),FOREGROUND_BLUE|FOREGROUND_GREEN);
			}
			capture.release();
			if( !openVideoFile(aviNames[idxFile],capture ,numFrame))
			{
				//printf("can't open file: %s",aviNames[idxFile]);
				std::stringstream msgStr;
				msgStr << "cannot open file " << aviNames[idxFile];
				logPrintf(logLevelError_e, "ImageCollector", msgStr.str());
				continue;
			}
			fclose(gpsFile);
			totalNumFrame = numFrame;

			gpsFile = fopen(gpsNames[idxFile],"r");
			if(NULL == gpsFile)
			{
				std::stringstream msgStr;
				msgStr << "cannot open file " << gpsNames[idxFile];
				logPrintf(logLevelInfo_e, "ImageCollector",msgStr.str());
				return -1;
			}
			fseek(gpsFile, 0, SEEK_SET);
			
			//make sure the video start point the previous GPS is the same.
			fscanf(gpsFile,"%lf,%lf\n",&preGps.lat,&preGps.lon);
			preGps.alt = 0;
			fseek(gpsFile, 0, SEEK_SET);

			if(imageBuffer.getCurrentImageNum() < LAST_FRAME_NUM)
			{
				imageBuffer.cleanCurrentBuffer();
			}else
			{
				imageBuffer.setReadyFlag();
			}
			RD_ADD_TS(tsFunId_eThread_ImageCollect,4);
		}

		RD_ADD_TS(tsFunId_eThread_ImageCollect,5);
		WaitForSingleObject(g_readySema_VideoReader, INFINITE);
		//send the information to server side
		{
			if(counter >= (int)(fps))
			{
				counter = 0;
				
				char sendBuff[200];
	            int sendLen = 1;
				int checkSum = 0;
				int videoOffset = (totalNumFrame - numFrame) * 1000 / fps;
				string fileName = GetFileNameByFilePath(aviNames[idxFile]);

	            sprintf(sendBuff, "$VEHICLE,%ld,%s,%i", g_VehicleID, fileName.c_str(), videoOffset);

				for(int index = sendLen; index < sizeof(sendBuff); index++)
				{
					if('\0' != sendBuff[index])
					{
						checkSum ^= sendBuff[index];
						sendLen++;
					}
					else
					{
						break;
					}
				}
				sprintf(&sendBuff[sendLen],"*%2X",checkSum);
				sendLen += 3;
					
				int nRet = sendto(g_EmulatorSockUDP,(char*)sendBuff,sendLen,0,(SOCKADDR*)&emulatorAddr,g_SocketLen);
				if ((nRet == SOCKET_ERROR) || (nRet == 0))
				{
					logPrintf(logLevelInfo_e, "COMM", "Send message failed!");
				}
				else
				{
					//logPrintf(logLevelInfo_e, "COMM", "<<<< Send message OK");
				}
			}
			counter++;			
		}

		//get one frame image and GPS
		RD_ADD_TS(tsFunId_eThread_ImageCollect,6);
		{
			if(VideoPlayEnum_pause == videoPlaySpeed)
			{
				if (frameInterval <= 0)
				{
					frameInterval = MAX_FRAME_INTERVAL;

					cv::namedWindow("image",CV_WINDOW_NORMAL);
					cv::resize(image,image,showSize);
					cv::imshow("image",image);
					cv::waitKey(1);
				}else
				{
					frameInterval--;
				}
				continue;
			}else
			{
				int totalLimit = (1<<(int)(videoPlaySpeed));
				videoSpeedCnt++;
				if( videoSpeedCnt < totalLimit)
				{
					continue;
				}else
				{
					videoSpeedCnt = 0;
				}
			}

#if(ON == RD_PAUSE_BUFFER_FULL)
			if(addFlag)
#endif
			{
				if(!readImageAndGps(capture, gpsFile, image, currentGps))
				{
					numFrame--;
					continue;
				}
				numFrame--;
			}
		}

		//historyInfoP.saveCurrentGps(currentGps);
		
		float speed, direction;
		getSpeedAndDirectionInfo(currentGps,preGps,fps,speed,direction);

		//linkun function to get if the point need to save image
		//if false
		//if(!database_gp->checkDbCompleteByGps(&currentGps))
		{
			imageBuffer.setSaveFlag();
		}
		
		addFlag = imageBuffer.addImage(image,currentGps,preGps,speed,direction, inParamIdxs[idxFile]);

#if(ON == RD_PAUSE_BUFFER_FULL)
		if(!addFlag)
		{
			continue;
		}
#endif
		updateGpsInfo(currentGps,inParamIdxs[idxFile]);
		preGps = currentGps;
        
        // only display image in certian interval
        if (frameInterval <= 0)
        {
            frameInterval = MAX_FRAME_INTERVAL;

		    cv::namedWindow("image",CV_WINDOW_NORMAL);
		    cv::resize(image,image,showSize);
		    cv::imshow("image",image);
		    cv::waitKey(1);
        }else
        {
            frameInterval--;
        }
		RD_ADD_TS(tsFunId_eThread_ImageCollect,14);
	}//end while(1)
	timeKillEvent(timer_id); 
}
#elif(RD_MODE == RD_VIDEO_LOAD_MODE)


unsigned int __stdcall Thread_ImageSensorCollector(void *data)
{
	int numFiles;
	int idxFile = 0;
	int numFrame;
	int totalNumFrame;
	double fps;
	int counter = 0;

	RD_ADD_TS(tsFunId_eThread_ImageCollect,1);
	
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/DE_Airport2_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
    vector<string> aviGpsFileName(5, "");
    aviGpsFileName[0] = "./config/DE_Airport_aviGpsFiles.txt";
    aviGpsFileName[1] = "./config/DE_Airport1_aviGpsFiles.txt";
    aviGpsFileName[2] = "./config/DE_Airport2_aviGpsFiles.txt";
    aviGpsFileName[3] = "./config/DE_Airport3_aviGpsFiles.txt";
    aviGpsFileName[4] = "./config/DE_AirportTcross_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_GERMAN_LEHRE)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/DE_Lehre_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_GERMAN_LEHRE2)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/DE_Lehre2_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_US_DETROIT)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/US_Detroit_aviGpsFiles.txt";
#elif (RD_LOCATION == RD_US_PALO_ALTO)
    vector<string> aviGpsFileName(1, "");
    aviGpsFileName[0] = "./config/US_Palo_Alto_aviGpsFiles.txt";
#endif

    for(int aviGpsFileIdx = 0; aviGpsFileIdx < aviGpsFileName.size(); ++aviGpsFileIdx)
    {
        FILE* fp = fopen(aviGpsFileName[aviGpsFileIdx].c_str(), "r");

	    if(fp == NULL)
	    {
		    //printf("cannot open the aviGpsFiles.txt file\n");
		    logPrintf(logLevelError_e, "ImageCollector", "cannot open the aviGpsFiles.txt file");
	    }

        int readIdx = 0;

	    while(!feof(fp))
	    {
		    if((readIdx&0x1) == 0)
		    {
			    fscanf(fp,"%s",aviNames[readIdx>>1]);
		    }else
		    {
			    fscanf(fp,"%s",gpsNames[readIdx>>1]);
		    }
		    readIdx++;
	    }

	    fclose(fp);

        numFiles = (readIdx>>1);

        for(int Idx = 0; Idx < numFiles; Idx++)
	    {
		    imageBuffer.addVideoAndGpsName(aviNames[Idx],gpsNames[Idx], aviGpsFileIdx);
	    }
    }
    
	imageBuffer.addVideoFinish();
	RD_ADD_TS(tsFunId_eThread_ImageCollect,2);
	while(1)
	{
		Sleep(10000000);
	}

}

#endif



