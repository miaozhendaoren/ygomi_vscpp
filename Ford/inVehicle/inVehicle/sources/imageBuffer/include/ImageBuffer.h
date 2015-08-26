#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "database.h"
#include "databaseDef.h"

#define IMAGE_BUFFER_DEPTH          1500
#define IMAGE_BUFFER_NUM            2

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
		ImageBuffer(const char* imageFileName, const char* gpsFileName);
		bool addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction);
		bool getCurrentImage(imageInfo_t *outImage);
		int getImageNumber(void);
		void cleanBuffer(void);
		bool getReadyFlag(void);
		void setReadyFlag(void);
		bool openWriteFiles(cv::Size frameSize);
		bool closeWriteFiles(void);
		bool openReadFiles(void);
		bool closeReadFiles(void);
		bool setImageToStart(void);

	private:
		//imageInfo_t Buffer[IMAGE_BUFFER_DEPTH];
		char saveFileName[2][100];
		cv::VideoWriter *writer; 
		cv::VideoCapture reader;
		FILE *writeFp;
		FILE *readFp;
		int readIdx;
		int bufferSize;
		int writeIdx;
		bool ready_flag;
		bool writeFileFlag;
		bool readFileFlag;
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
        void setImageSize(int width,int height);
        void getImageSize(int& width,int& height);

	private:
		ImageBuffer *imageBuffer[IMAGE_BUFFER_NUM];
		int readIdx;
		bool saveFlag;
        int _imageHeight;
        int _imageWidth;
		HANDLE _hMutex;
};