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

using namespace ns_database;
using namespace cv;

VideoCapture capture;

#define LAST_FRAME_NUM 100
#if(RD_USE_CAMERA == ON) 
CNEMA_GPGGA_PROC gNemaGpggaProc;
#define NC_UDP_GPS_DATA_BUF_LEN        (1500)
char nc_udpGpsBuffer[NC_UDP_GPS_DATA_BUF_LEN];
CacheBuffer cacheBuffer;
volatile SOCKET g_ServerSockUDP;
#else
char aviNames[50][100];
char gpsNames[50][100];
HANDLE g_readySema_VideoReader;
unsigned int timeDelay;
volatile SOCKET g_EmulatorSockUDP;
SOCKADDR_IN emulatorAddr;
#endif

#if(RD_USE_CAMERA == ON)

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

void imageTimer(int value)
{
	if(3 == value)
	{
		imageCamera_t tempImage;
		glutTimerFunc((unsigned int)(33),&imageTimer,3);

		//get the 
		cacheBuffer.lockCacheBuffer();
		if(imageSensorCollect_Cam(capture,tempImage.image))
		{
			tempImage.st = getSysTimeMs();

			cacheBuffer.addImage(tempImage);
			cacheBuffer.releaseCacheBuffer();

			//cv::namedWindow("image");
			//cv::imshow("image",tempImage.image);
			//cv::waitKey(1);
		}else
		{
			cacheBuffer.releaseCacheBuffer();
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

	if(!openImageSensor_Cam(g_CameraPort,capture ,numFrame))
	{
		logPrintf(logLevelInfo_e, "IMAGE_COLLECTOR", "Open Camera failed!");
		return -1;
	}
	
    imageBuffer.setImageSize(capture.get(CV_CAP_PROP_FRAME_WIDTH),
        capture.get(CV_CAP_PROP_FRAME_HEIGHT));

	if(!InitGpsSocket_UDP())
	{
		logPrintf(logLevelInfo_e, "IMAGE_COLLECTOR", "Open GPS UDP socket failed!");
		return -1;
	}

	//start the timer to get the camera image
	glutTimerFunc((unsigned int)(50),&imageTimer,3);

	while(1)
	{
		//recevie the GPS information
		gpsSensorCollect_UDP();

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
			if(sqrt((outPoint.lat*outPoint.lat)+(outPoint.lon*outPoint.lon)) > 200)
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
			cv::namedWindow("image",CV_WINDOW_NORMAL);
			cv::imshow("image",(*imageTempVec->begin()).image);
			cv::waitKey(1);

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
				imageBuffer.addImage((*imageIter).image,currentGps,preGps,0,0);

				preGps = currentGps;
			}
		}//end if(init)

	}//end while(1)

}
#else
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

void updateGpsInfo(point3D_t gpsPoint)
{
	gGpsInfo.dLatitudePrePre = gGpsInfo.dLatitudePre;
	gGpsInfo.dLongitudePrePre = gGpsInfo.dLongitudePre;
	gGpsInfo.altitudePrePre = gGpsInfo.altitudePre;

	gGpsInfo.dLatitudePre = gGpsInfo.dLatitude;
	gGpsInfo.dLongitudePre = gGpsInfo.dLongitude;
	gGpsInfo.altitudePre = gGpsInfo.altitude;

	gGpsInfo.dLatitude = gpsPoint.lat;
	gGpsInfo.dLongitude = gpsPoint.lon;
	gGpsInfo.altitude = gpsPoint.alt;
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

void imageTimer(int value)
{
	if(3 == value)
	{
		glutTimerFunc(timeDelay,&imageTimer,3);
		ReleaseSemaphore(g_readySema_VideoReader, 1 ,NULL);
	}
}

unsigned int __stdcall Thread_ImageSensorCollector(void *data)
{
	int numFiles;
	int idxFile = 0;
	int numFrame;
	int totalNumFrame;
	double fps;
	int counter = 0;
	point3D_t currentGps;
	point3D_t preGps;
	g_readySema_VideoReader = CreateSemaphore(NULL,0,10,"semaphore_VideoReader");
	cv::Size showSize;

	InitTimeOffsetSocket_UDP();
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
    FILE* fp = fopen("./config/DE_Airport2_aviGpsFiles.txt", "r");
#elif (RD_LOCATION == RD_GERMAN_LEHRE)
    FILE* fp = fopen("./config/DE_Lehre_aviGpsFiles.txt", "r");
#elif (RD_LOCATION == RD_GERMAN_LEHRE2)
    FILE* fp = fopen("./config/DE_Lehre2_aviGpsFiles.txt", "r");
#elif (RD_LOCATION == RD_US_DETROIT)
	FILE* fp = fopen("./config/US_Detroit_aviGpsFiles.txt", "r"); 
#elif (RD_LOCATION == RD_US_PALO_ALTO)
	FILE* fp = fopen("./config/aviGpsFiles.txt", "r"); 
#endif
	
	if(fp == NULL)
	{
		printf("cannot open the aviGpsFiles.txt file\n");
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

	numFiles = (readIdx>>1);
	fclose(fp);
	srand((unsigned)time(NULL));
	//idxFile = generateRandFileIdx(numFiles);
	printf("select video %d\n",idxFile);

	FILE* gpsFile = fopen(gpsNames[idxFile],"r");
	fseek(gpsFile, 0, SEEK_SET);
	fscanf(gpsFile,"%lf,%lf\n",&preGps.lat,&preGps.lon);
	preGps.alt = 0;
	fseek(gpsFile, 0, SEEK_SET);

	if( !openVideoFile(aviNames[idxFile],capture ,numFrame))
	{
		printf("can't open file: %s",aviNames[idxFile]);
	}
	totalNumFrame = numFrame;
	fps = capture.get(CV_CAP_PROP_FPS);  //get the frames per seconds of the video

	imageBuffer.setImageSize(capture.get(CV_CAP_PROP_FRAME_WIDTH),
	    capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	showSize.height = capture.get(CV_CAP_PROP_FRAME_HEIGHT)/2;
	showSize.width = capture.get(CV_CAP_PROP_FRAME_WIDTH)/2;

	timeDelay = (unsigned int)(1000/fps);
	glutTimerFunc(timeDelay,&imageTimer,3);

    const int MAX_FRAME_INTERVAL = 1;
    int frameInterval = 0;

	while(1)
	{
		cv::Mat image;
		
		if(numFrame <= 0)
		{

			idxFile++;
			if(idxFile >= numFiles)
			{
				idxFile = 0;
			}

			//idxFile = generateRandFileIdx(numFiles);
			printf("select video %d\n",idxFile);
			capture.release();
			if( !openVideoFile(aviNames[idxFile],capture ,numFrame))
			{
				printf("can't open file: %s",aviNames[idxFile]);
				continue;
			}
			fclose(gpsFile);
			totalNumFrame = numFrame;

			gpsFile = fopen(gpsNames[idxFile],"r");
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
		}

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
		{
			if(!readImageAndGps(capture, gpsFile, image, currentGps))
			{
				numFrame--;
				continue;
			}
			numFrame--;
		}

		updateGpsInfo(currentGps);
		historyInfoP.saveCurrentGps(currentGps);
		
		float speed, direction;
		getSpeedAndDirectionInfo(currentGps,preGps,fps,speed,direction);

		//linkun function to get if the point need to save image
		//if false
		//if(!database_gp->checkDbCompleteByGps(&currentGps))
		{
			imageBuffer.setSaveFlag();
		}
		
		imageBuffer.addImage(image,currentGps,preGps,speed,direction);

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
	}
}
#endif



