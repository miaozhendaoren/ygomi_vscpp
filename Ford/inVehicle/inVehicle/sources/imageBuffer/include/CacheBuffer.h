#pragma once

#include <vector>
#include <windows.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using namespace std;
 
#define CACHE_MAX_NUM_IMAGE    50 
#define CACHE_BUFFER_NUM       2
 
typedef struct
{
	cv::Mat image;
	unsigned int st;
}imageCamera_t;

class CacheBuffer
{
public:
	CacheBuffer();
	~CacheBuffer();
	void addImage(imageCamera_t& image);
	void swatchBuffer();
	void getBackendBuffer(vector<imageCamera_t> **bufPtr);
	void lockCacheBuffer();
	void releaseCacheBuffer();
	
private:
	vector<imageCamera_t> imageVec[CACHE_BUFFER_NUM];  //ping pong cache buffer to save the image 
	int writeIdx;
	HANDLE _hMutex;
};