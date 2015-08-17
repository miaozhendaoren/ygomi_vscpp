#include "CacheBuffer.h"

CacheBuffer::CacheBuffer()
{
	writeIdx = 0;
	_hMutex = CreateMutex(NULL,FALSE,NULL);
	ReleaseMutex(_hMutex);
}

CacheBuffer::~CacheBuffer()
{
	for(int i = 0; i < CACHE_BUFFER_NUM; i++){
		imageVec[i].clear();
	}
	CloseHandle(_hMutex);
}

void CacheBuffer::addImage(imageCamera_t& image)
{
	WaitForSingleObject(_hMutex,INFINITE);
	if(imageVec[writeIdx].size() < CACHE_MAX_NUM_IMAGE)
	imageVec[writeIdx].push_back(image);
	ReleaseMutex(_hMutex);
}

void CacheBuffer::swatchBuffer()
{
	WaitForSingleObject(_hMutex,INFINITE); 
	writeIdx ^= 1;
	imageVec[writeIdx].clear();
	ReleaseMutex(_hMutex);
}

//do not add mutex because the function will be called after swatchBuffer(), in the same thread
void CacheBuffer::getBackendBuffer(vector<imageCamera_t> **bufPtr)
{
	*bufPtr = &imageVec[writeIdx^1];
}
