
#include "AppInitCommon.h" // inParam.offsetDist // FIXME: do not change the order of this header file

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "ImageBuffer.h"
#include "Signal_Thread_Sync.h"
#include "database.h"   // lookAheadOnTrack

#if(RD_MODE == RD_VIDEO_LOAD_MODE)
ImageBuffer::ImageBuffer()
{
	readIdx = 0;
	bufferSize = 0;
	_imageHeight = 0;
	_imageWidth = 0;
	readFileFlag = false;
}

bool ImageBuffer::getCurrentImage(imageInfo_t *outImage)
{
	reader.read(outImage->image);
	if(!feof(readFp))
	{
		fscanf(readFp,"%lf,%lf\n",&outImage->gpsInfo.lat,
			&outImage->gpsInfo.lon);
		
		outImage->gpsInfo.alt = 0;
		
		//if it is the first point, the previous gps should be the same
		if(readIdx == 0)
		{
			outImage->gpsInfoPre = outImage->gpsInfo;
		}else
		{
			outImage->gpsInfoPre = preGps;
		}
		preGps = outImage->gpsInfo;
	}
	readIdx = ((readIdx+1)%bufferSize);
	if(readIdx == 0)
	{
		reader.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
		fseek(readFp,0,SEEK_SET);
	}

	return true;
}

bool ImageBuffer::setImageToStart(void)
{
	reader.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
	fseek(readFp,0,SEEK_SET);
	readIdx = 0;
	return true;
}

int ImageBuffer::getImageNumber(void)
{
	return bufferSize;
}

void ImageBuffer::cleanBuffer(void)
{
	readIdx = 0;
	bufferSize = 0;
	_imageWidth = 0;
	_imageHeight = 0;
}

bool ImageBuffer::openReadFiles()
{
    printf("Open video: %s\n",saveFileName[0]);
	if(!readFileFlag)
	{
		reader.open(saveFileName[0]);
		if(!reader.isOpened())
		{
			printf("cannot open reader %s file\n",saveFileName[0]);
			return false;
		}
		bufferSize = reader.get(CV_CAP_PROP_FRAME_COUNT);
		_imageWidth = reader.get(CV_CAP_PROP_FRAME_WIDTH);
		_imageHeight = reader.get(CV_CAP_PROP_FRAME_HEIGHT);
		
		readFp = fopen(saveFileName[1],"r");
		if(readFp == NULL)
		{
			printf("cannot open the reader %s file\n",saveFileName[1]);
			return false;
		}
		readFileFlag = true;
	}

    // Distance compensation to minimize systematic error
    //if (0)
    {
        // read existing location from file
        std::vector<point3D_t> locVec, locVecAhead;

        for(int idx = 0; idx < bufferSize; idx++)
        {
            point3D_t locPoint;

            fscanf(readFp,"%lf,%lf\n",&locPoint.lat, &locPoint.lon);

            locVec.push_back(locPoint);
        }

        fclose(readFp);

        // look ahead
        ns_database::lookAheadOnTrack(locVec, inParam.offsetDist, locVecAhead);

        // write new location to file
        readFp = fopen("c.txt", "wt");

        for(int idx = 0; idx < locVecAhead.size(); idx++)
        {
            point3D_t locPoint;

            fprintf(readFp, "%.14lf,%.14lf\n",locVecAhead[idx].lat, locVecAhead[idx].lon);
        }

        fclose(readFp);
        readFp = fopen("c.txt", "rt");

        bufferSize = locVecAhead.size();
    }
	return true;
}

bool ImageBuffer::closeReadFiles()
{
	if(readFileFlag)
	{
		reader.release();
		fclose(readFp);
	}
	readFileFlag = false;
	return true;
}

bool ImageBuffer::setFileName(const char* imageFileName, const char* gpsFileName)
{
	memset(saveFileName[0],0,MAX_VIDEO_FILENAME_LEN*sizeof(char));
	memset(saveFileName[1],0,MAX_VIDEO_FILENAME_LEN*sizeof(char));
	
	strcpy(saveFileName[0],imageFileName);
	strcpy(saveFileName[1],gpsFileName);
	return true;
}

bool ImageBuffer::getImageSize(int& width, int& height)
{
	width  = _imageWidth;
	height = _imageHeight; 
	return true;
}

ImageBufferAll::ImageBufferAll()
{	
	imageBuffer = new ImageBuffer();
	totalNum = 0;
	proIdx   = 0;
	readyFlag = false;
	for(int idx = 0; idx < MAX_VIDEO_NUM; idx++)
	{
		memset(aviNames[idx],0,MAX_VIDEO_FILENAME_LEN*sizeof(char));
		memset(gpsNames[idx],0,MAX_VIDEO_FILENAME_LEN*sizeof(char));
	}
}

ImageBufferAll::~ImageBufferAll()
{
	delete imageBuffer;
}

bool ImageBufferAll::addVideoAndGpsName(const char* imageFileName, const char* gpsFileName)
{
	if(totalNum >= MAX_VIDEO_NUM)
	{
		printf("file number is bigger than %d\n",MAX_VIDEO_NUM );
		return false;
	}
	strcpy(aviNames[totalNum],imageFileName);
	strcpy(gpsNames[totalNum],gpsFileName);
	totalNum++;
	
	return true;
}

void ImageBufferAll::addVideoFinish(void)
{
	readyFlag = true;
}

void ImageBufferAll::getImageSize(int& width,int& height)
{
	imageBuffer->getImageSize(width, height);
}

bool ImageBufferAll::getBuffer(ImageBuffer **buffer)
{
	if(!readyFlag)
	{
		return false;
	}
	imageBuffer->cleanBuffer();
	imageBuffer->closeReadFiles();
	
	imageBuffer->setFileName(aviNames[proIdx],gpsNames[proIdx]);
	
	proIdx++;
	if(proIdx > totalNum)
	{
		//proIdx = 0;
        proIdx = totalNum;

        static bool firstTimeFlag = true;
        if (firstTimeFlag == true)
        {
            printf("load files finished\n");
            firstTimeFlag = false;
        }
        return false;
	}

    imageBuffer->openReadFiles();

	*buffer = imageBuffer;
	return true;
}

#else
ImageBuffer::ImageBuffer(const char* imageFileName, const char* gpsFileName)
{
	readIdx = 0;
	bufferSize = 0;
	writeIdx = 0;
	ready_flag = false;
	writeFileFlag = false;
	readFileFlag = false;

	_hMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(_hMutex);
	strcpy(saveFileName[0],imageFileName);
	strcpy(saveFileName[1],gpsFileName);
}

bool ImageBuffer::openWriteFiles(cv::Size frameSize)
{
	if(!writeFileFlag)
	{
		writer = new cv::VideoWriter(saveFileName[0],CV_FOURCC('P','I','M','1'),30,frameSize);
		if(!writer->isOpened())
		{
			printf("cannot open the %s file\n",saveFileName[0]);
			return false;
		}
		writeFp = fopen(saveFileName[1],"w");
		if(writeFp == NULL)
		{
			printf("cannot open the %s file\n",saveFileName[1]);
			return false;
		}
		writeFileFlag = true;
	}
	return true;
}

bool ImageBuffer::closeWriteFiles()
{
	if(writeFileFlag)
	{
	delete writer;
	fclose(writeFp);
	writeFileFlag = false;
	}
	return true;
}

bool ImageBuffer::openReadFiles()
{
	if(!readFileFlag)
	{
		reader.open(saveFileName[0]);
		if(!reader.isOpened())
		{
			printf("cannot open reader %s file\n",saveFileName[0]);
			return false;
		}
		readFp = fopen(saveFileName[1],"r");
		if(readFp == NULL)
		{
			printf("cannot open the reader %s file\n",saveFileName[1]);
			return false;
		}
		readFileFlag = true;
	}
	return true;
}

bool ImageBuffer::closeReadFiles()
{
	if(readFileFlag)
	{
		reader.release();
		fclose(readFp);
	}
	readFileFlag = false;
	return true;
}

bool ImageBuffer::addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction)
{
	WaitForSingleObject(_hMutex,INFINITE);
	
	//int pendFlag = 0;
	
	if(!ready_flag)
	{
		if(writeIdx == 0)
		{
			bool flag = openWriteFiles(image.size());
		}
		//Buffer[writeIdx].image = image.clone();
		if(writeFileFlag)
		{
			writer->write(image);
			fprintf(writeFp,"%.14lf,%.14lf,%.14lf,%.14lf,%.14lf,%.14lf\n",gpsInfo.lat,gpsInfo.lon,gpsInfo.alt,gpsInfoPre.lat,gpsInfoPre.lon,gpsInfoPre.alt);
			writeIdx++;
			bufferSize = writeIdx;
			if(writeIdx >= IMAGE_BUFFER_DEPTH)
			{
				writeIdx = 0;
				closeWriteFiles();
				ready_flag = true;
				printf("image buffer is full\n");
			}
		}

	}
	
	ReleaseMutex(_hMutex);
	return true;
}

bool ImageBuffer::setImageToStart(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	reader.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
	fseek(readFp,0,SEEK_SET);
	readIdx = 0;
	ReleaseMutex(_hMutex);
	return true;
}

bool ImageBuffer::getCurrentImage(imageInfo_t *outImage)
{
	WaitForSingleObject(_hMutex,INFINITE);
	//*outImage = &Buffer[readIdx];
	reader.read(outImage->image);
	if(!feof(readFp))
	{
		fscanf(readFp,"%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&outImage->gpsInfo.lat,
			&outImage->gpsInfo.lon,
			&outImage->gpsInfo.alt,
			&outImage->gpsInfoPre.lat,
			&outImage->gpsInfoPre.lon,
			&outImage->gpsInfoPre.alt);
	}
	readIdx = ((readIdx+1)%bufferSize);
	if(readIdx == 0)
	{
		reader.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
		fseek(readFp,0,SEEK_SET);
	}
	ReleaseMutex(_hMutex);
	return true;
}

int ImageBuffer::getImageNumber(void)
{
	//return (readIdx < writeIdx)?(writeIdx - readIdx):(IMAGE_BUFFER_DEPTH + writeIdx - readIdx);
	return bufferSize;
}

void ImageBuffer::cleanBuffer(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	readIdx = 0;
	writeIdx = 0;
	bufferSize = 0;
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
	closeWriteFiles();
	ReleaseMutex(_hMutex);
}

ImageBufferAll::ImageBufferAll()
{
	readIdx = 0;
	saveFlag = true;
	_hMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(_hMutex);
	imageBuffer[0] = new ImageBuffer("a.avi","a.txt");
	imageBuffer[1] = new ImageBuffer("b.avi","b.txt");
}

void ImageBufferAll::addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction)
{
	if(saveFlag)
	{
		imageBuffer[readIdx]->addImage(image,gpsInfo,gpsInfoPre,speed,direction);
	}
}

bool ImageBufferAll::getBuffer(ImageBuffer **buffer)
{
	WaitForSingleObject(_hMutex,INFINITE);
	saveFlag = true;
	if(imageBuffer[readIdx]->getReadyFlag())
	{
		*buffer = (imageBuffer[readIdx]);
		//imageBuffer[readIdx]->closeWriteFiles();
		imageBuffer[readIdx]->openReadFiles();
		readIdx = (readIdx+1)%IMAGE_BUFFER_NUM;
		imageBuffer[readIdx]->cleanBuffer();
		imageBuffer[readIdx]->closeReadFiles();
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
	imageBuffer[readIdx]->cleanBuffer();
	imageBuffer[readIdx]->closeWriteFiles();
	ReleaseMutex(_hMutex);
}

int ImageBufferAll::getCurrentImageNum(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	ReleaseMutex(_hMutex);
	return imageBuffer[readIdx]->getImageNumber();
}

void ImageBufferAll::setReadyFlag(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	imageBuffer[readIdx]->setReadyFlag();
	ReleaseMutex(_hMutex);
}

void ImageBufferAll::setImageSize(int width,int height)
{
    _imageHeight = height;
    _imageWidth = width;
}

void ImageBufferAll::getImageSize(int& width,int& height)
{
    width = _imageWidth;
    height = _imageHeight;
}

#endif