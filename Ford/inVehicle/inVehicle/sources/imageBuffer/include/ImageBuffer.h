#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "database.h"
#include "databaseDef.h"

#define IMAGE_BUFFER_DEPTH          100
#define IMAGE_BUFFER_NUM            2
#define IMAGE_SENSOR_WIDTH          640
#define IMAGE_SENSOR_HEIGHT         480

using namespace ns_database;

struct imageInfo_t
{
	cv::Mat image;
	point3D_t gpsInfo;
	point3D_t gpsInfoPre;
	float    speed;
	float    direction;
};

class ImageBuffer
{
	public:
		ImageBuffer();
		bool addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction);
		bool getCurrentImage(imageInfo_t **outImage);
		bool getNextImage(imageInfo_t **outImage);
		bool getPreImage(imageInfo_t **outImage);
		int getImageNumber(void);
		void cleanBuffer(void);
		bool getReadyFlag(void);
		void setReadyFlag(void);

	private:
		imageInfo_t Buffer[IMAGE_BUFFER_DEPTH];
		int readIdx;
		int readNum;
		int writeIdx;
		bool ready_flag;
		HANDLE _hMutex;
};

class ImageBufferAll
{
	public:
		ImageBufferAll();
		void addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction);
		bool getBuffer(ImageBuffer **buffer);
		void cleanCurrentBuffer(void);
		void setReadyFlag(void);
		int  getCurrentImageNum(void);
		void setSaveFlag(void);

	private:
		ImageBuffer imageBuffer[IMAGE_BUFFER_NUM];
		int readIdx;
		bool saveFlag;
		HANDLE _hMutex;
};