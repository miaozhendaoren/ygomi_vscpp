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

#if(RD_USE_CAMERA == ON) 
CNEMA_GPGGA_PROC gNemaGpggaProc;
#define NC_UDP_GPS_DATA_BUF_LEN        (1500)
char nc_udpGpsBuffer[NC_UDP_GPS_DATA_BUF_LEN];
CacheBuffer cacheBuffer;
#else
#define LAST_FRAME_NUM 50
char aviNames[50][100];
char gpsNames[50][100];
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
		double fps = capture.get(CV_CAP_PROP_FPS);
		capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
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

	cacheBuffer.swatchBuffer();
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
		cv::Mat currentImage;
		glutTimerFunc((unsigned int)(50),&imageTimer,3);

		//get the 
		if(imageSensorCollect_Cam(capture,currentImage))
		{
			imageCamera_t tempImage;
			tempImage.st = getSysTimeMs();
			resize(currentImage, currentImage, cv::Size(IMAGE_SENSOR_WIDTH, IMAGE_SENSOR_HEIGHT));
			tempImage.image = currentImage.clone();

			cacheBuffer.addImage(tempImage);

			cv::namedWindow("image");
			cv::imshow("image",currentImage);
			cv::waitKey(1);
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

unsigned int __stdcall Thread_ImageSensorCollector(void *data)
{
	int device = 0;
	int numFrame;
	bool init = true;
	point3D_t currentGps;
	point3D_t preGps;

	if(!openImageSensor_Cam(device,capture ,numFrame))
	{
		logPrintf(logLevelInfo_e, "IMAGE_COLLECTOR", "Open Camera failed!");
		return -1;
	}
	
	if(!InitGpsSocket_UDP())
	{
		logPrintf(logLevelInfo_e, "IMAGE_COLLECTOR", "Open GPS UDP socket failed!");
		return -1;
	}

	//start the timer to get the camera image
	glutTimerFunc((unsigned int)(1000),&imageTimer,3);

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
			point3D_t startPoint;
			point3D_t endPoint;
			startPoint.alt = gGpsInfo.altitudePre;
			startPoint.lat = gGpsInfo.dLatitudePre;
			startPoint.lon = gGpsInfo.dLongitudePre;

			endPoint.alt = gGpsInfo.altitude;
			endPoint.lat = gGpsInfo.dLatitude;
			endPoint.lon = gGpsInfo.dLongitude;

			for(std::vector<imageCamera_t>::iterator imageIter = imageTempVec->begin();
				imageIter != imageTempVec->end();
				++imageIter)
			{
				//calc each image's gps
				double timeOffset = ((*imageIter).st > gGpsInfo.stPre)?((*imageIter).st - gGpsInfo.stPre):(0xffffffff - (gGpsInfo.stPre - (*imageIter).st)+1);
				double ratio = timeOffset/totalTime;
				insertGps(startPoint, endPoint, ratio, currentGps);

				//save image to imagebuffer
				imageBuffer.setSaveFlag();
				imageBuffer.addImage((*imageIter).image,currentGps,preGps,0,0);

				preGps = currentGps;
			}
		}//end if(init)

	}//end while(1)

}
#else
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

unsigned int __stdcall Thread_ImageSensorCollector(void *data)
{
	int numFiles;
	int idxFile = 0;
	int numFrame;
	double fps;
	point3D_t currentGps;
	point3D_t preGps;
	//currentGps.alt = 0;
	//currentGps.lat = 0;
	//currentGps.lon = 0;
	FILE* fp = fopen("./config/aviGpsFiles.txt", "r"); 
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
	fps = capture.get(CV_CAP_PROP_FPS);  //get the frames per seconds of the video

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
			gpsFile = fopen(gpsNames[idxFile],"r");
			fseek(gpsFile, 0, SEEK_SET);
			
			//make sure the video start point the previous GPS is the same.
			fscanf(gpsFile,"%lf,%lf\n",&preGps.lat,&preGps.lon);
			preGps.alt = 0;
			fseek(gpsFile, 0, SEEK_SET);
#if 1
			if(imageBuffer.getCurrentImageNum() < LAST_FRAME_NUM)
			{
				imageBuffer.cleanCurrentBuffer();
			}else
			{
				imageBuffer.setReadyFlag();
                //WaitForSingleObject(g_readySema_DiffDet, INFINITE);
			}
#endif
		}

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
		resize(image, image, cv::Size(IMAGE_SENSOR_WIDTH, IMAGE_SENSOR_HEIGHT));
		imageBuffer.addImage(image,currentGps,preGps,speed,direction);

		preGps = currentGps;
		cv::namedWindow("image");
		cv::imshow("image",image);
		cv::waitKey(1);
		Sleep(50);
	}
}
#endif



