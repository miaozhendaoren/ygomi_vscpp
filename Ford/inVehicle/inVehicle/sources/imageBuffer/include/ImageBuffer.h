#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "database.h"
#include "databaseDef.h"
#include "configure.h"

using namespace ns_database;

#define MAX_VIDEO_NUM            50
#define MAX_VIDEO_FILENAME_LEN   100

#if (RD_LOCATION == RD_GERMAN_LEHRE || RD_LOCATION == RD_GERMAN_LEHRE2)
    #define IMAGE_BUFFER_DEPTH          400
#elif (RD_LOCATION == RD_US_DETROIT)
    #define IMAGE_BUFFER_DEPTH          400
#else
    #define IMAGE_BUFFER_DEPTH          1500
#endif

#define IMAGE_BUFFER_NUM            2

struct imageInfo_t
{
	cv::Mat image;
	point3D_t gpsInfo;
	point3D_t gpsInfoPre;
	float    speed;
	float    direction;
};

#if(RD_MODE == RD_VIDEO_LOAD_MODE)

class ImageBuffer
{
	public:
		ImageBuffer();
		bool getCurrentImage(imageInfo_t *outImage);
		int  getImageNumber(void);
		void cleanBuffer(void);
		bool openReadFiles(void);
		bool closeReadFiles(void);
		bool setImageToStart(void);
		bool setFileName(const char* imageFileName, const char* gpsFileName);
		bool getImageSize(int& width, int& height);

	private:
		char saveFileName[2][MAX_VIDEO_FILENAME_LEN];
		cv::VideoCapture reader;
		FILE *readFp;
		int readIdx;
		int bufferSize;
		bool ready_flag;
		bool readFileFlag;
		point3D_t preGps;
		int _imageHeight;
		int _imageWidth;
};

class ImageBufferAll
{
	public:
		ImageBufferAll();
		~ImageBufferAll();
		bool getBuffer(ImageBuffer **buffer);
        void getImageSize(int& width,int& height);
		bool addVideoAndGpsName(const char* imageFileName, const char* gpsFileName);
		void addVideoFinish(void);

	private:
		ImageBuffer *imageBuffer;
	    int totalNum;
		int proIdx;
		bool readyFlag;
		char aviNames[MAX_VIDEO_NUM][MAX_VIDEO_FILENAME_LEN];
		char gpsNames[MAX_VIDEO_NUM][MAX_VIDEO_FILENAME_LEN];
};

#else

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
		char saveFileName[2][MAX_VIDEO_FILENAME_LEN];
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

#endif
