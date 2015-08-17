#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "ImageBuffer.h"
#include "Signal_Thread_Sync.h"

ImageBuffer::ImageBuffer()
{
	readIdx = 0;
	readNum = 0;
	writeIdx = 0;
	ready_flag = false;

	_hMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(_hMutex);
}

bool ImageBuffer::addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction)
{
	WaitForSingleObject(_hMutex,INFINITE);
	
	//int pendFlag = 0;
	
	if(!ready_flag)
	{
		Buffer[writeIdx].image = image.clone();
		Buffer[writeIdx].gpsInfo = gpsInfo;
		Buffer[writeIdx].gpsInfoPre = gpsInfoPre;
		Buffer[writeIdx].speed = speed;
		Buffer[writeIdx].direction = direction;
	
		writeIdx++;
		if(writeIdx >= IMAGE_BUFFER_DEPTH)
		{
			writeIdx = 0;
			ready_flag = true;
			printf("image buffer is full\n");
            //pendFlag = 1;
		}
	}
	
	ReleaseMutex(_hMutex);
    //if (pendFlag == 1)
    //{
    //    WaitForSingleObject(g_readySema_DiffDet, INFINITE);    
    //}

	return true;
}

bool ImageBuffer::getCurrentImage(imageInfo_t **outImage)
{
	WaitForSingleObject(_hMutex,INFINITE);
	*outImage = &Buffer[readIdx];
	readIdx = ((readIdx+1)%IMAGE_BUFFER_DEPTH);
	ReleaseMutex(_hMutex);
	return true;
}

bool ImageBuffer::getNextImage(imageInfo_t **outImage)
{
	WaitForSingleObject(_hMutex,INFINITE);
	int Idx = ((readIdx+1)%IMAGE_BUFFER_DEPTH);
	if(Idx == writeIdx)
	{
		ReleaseMutex(_hMutex);
		return false;
	}
	*outImage = &Buffer[Idx];
	ReleaseMutex(_hMutex);
	return true;
}

bool ImageBuffer::getPreImage(imageInfo_t **outImage)
{
	WaitForSingleObject(_hMutex,INFINITE);
	int Idx = ((readIdx-1)%IMAGE_BUFFER_DEPTH);
	if(Idx == writeIdx)
	{
		ReleaseMutex(_hMutex);
		return false;
	}
	*outImage = &Buffer[Idx];
	ReleaseMutex(_hMutex);
	return true;
}

int ImageBuffer::getImageNumber(void)
{
	return (readIdx < writeIdx)?(writeIdx - readIdx):(IMAGE_BUFFER_DEPTH + writeIdx - readIdx);
}

void ImageBuffer::cleanBuffer(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	readIdx = 0;
	writeIdx = 0;
	ready_flag = false;
	ReleaseMutex(_hMutex);
}

bool ImageBuffer::getReadyFlag(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	ReleaseMutex(_hMutex);
	return ready_flag;
}

void ImageBuffer::setReadyFlag(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	ready_flag = true;
	ReleaseMutex(_hMutex);
}

ImageBufferAll::ImageBufferAll()
{
	readIdx = 0;
	saveFlag = true;
	_hMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(_hMutex);
}

void ImageBufferAll::addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction)
{
	if(saveFlag)
	{
		imageBuffer[readIdx].addImage(image,gpsInfo,gpsInfoPre,speed,direction);
	}
}

bool ImageBufferAll::getBuffer(ImageBuffer **buffer)
{
	WaitForSingleObject(_hMutex,INFINITE);
	saveFlag = true;
	if(imageBuffer[readIdx].getReadyFlag())
	{
		*buffer = &(imageBuffer[readIdx]);
		readIdx = (readIdx+1)%IMAGE_BUFFER_NUM;
		imageBuffer[readIdx].cleanBuffer();
		saveFlag = false;
		ReleaseMutex(_hMutex);
		return true;
	}
	else
	{
		ReleaseMutex(_hMutex);
		return false;
	}
	
}

void ImageBufferAll::setSaveFlag(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	saveFlag = true;
	ReleaseMutex(_hMutex);
}

void ImageBufferAll::cleanCurrentBuffer(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	imageBuffer[readIdx].cleanBuffer();
	ReleaseMutex(_hMutex);
}

int ImageBufferAll::getCurrentImageNum(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	ReleaseMutex(_hMutex);
	return imageBuffer[readIdx].getImageNumber();
}

void ImageBufferAll::setReadyFlag(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	imageBuffer[readIdx].setReadyFlag();
	ReleaseMutex(_hMutex);
}