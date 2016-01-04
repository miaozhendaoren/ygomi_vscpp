
#include "AppInitCommon.h" // inParam.offsetDist // FIXME: do not change the order of this header file

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "ImageBuffer.h"
#include "Signal_Thread_Sync.h"
#include "database.h"   // lookAheadOnTrack
#include "LogInfo.h"

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
        // skip 3 frames to match with smoothGps
		imageInfo_t image;
        reader.read(image.image);
        reader.read(image.image);
        reader.read(image.image);

		fseek(readFp,0,SEEK_SET);
	}

	return true;
}

bool ImageBuffer::setImageToStart(void)
{
	reader.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
    // skip 3 frames to match with smoothGps
    imageInfo_t image;
    getCurrentImage(&image);
    getCurrentImage(&image);
    getCurrentImage(&image);

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
    //printf("Open video: %s\n",saveFileName[0]);
	{
		std::stringstream msgStr;
		msgStr << "Open video "<< saveFileName[0];
		logPrintf(logLevelInfo_e,"ImageBuffer", msgStr.str().c_str());
	}

	if(!readFileFlag)
	{
		reader.open(saveFileName[0]);
		if(!reader.isOpened())
		{
			//printf("cannot open reader %s file\n",saveFileName[0]);
			std::stringstream msgStr;
			msgStr << "cannot open reader "<< saveFileName[0] << "file";
			logPrintf(logLevelError_e,"ImageBuffer", msgStr.str().c_str());
			return false;
		}
		bufferSize = reader.get(CV_CAP_PROP_FRAME_COUNT);
		_imageWidth = reader.get(CV_CAP_PROP_FRAME_WIDTH);
		_imageHeight = reader.get(CV_CAP_PROP_FRAME_HEIGHT);

		readFp = fopen(saveFileName[1],"r");
		if(readFp == NULL)
		{
			//printf("cannot open the reader %s file\n",saveFileName[1]);
			std::stringstream msgStr;
			msgStr << "cannot open reader "<< saveFileName[1] << "file";
			logPrintf(logLevelError_e,"ImageBuffer", msgStr.str().c_str());
			return false;
		}

        // Distance compensation to minimize systematic error
        //if (0)
        {
            // read existing location from file
            std::vector<point3D_t> locVec, locSmoothed,locVecAhead;

            for(int idx = 0; idx < bufferSize; idx++)
            {
                point3D_t locPoint;

                fscanf(readFp,"%lf,%lf\n",&locPoint.lat, &locPoint.lon);

                locVec.push_back(locPoint);
    		}

            // smooth the gps by 7-order average filter
            ns_database::smoothGps(7,locVec,locSmoothed);

            // skip 3 frames
            imageInfo_t image;
            getCurrentImage(&image);
            getCurrentImage(&image);
            getCurrentImage(&image);

    		fclose(readFp);

            // look ahead
            ns_database::lookAheadOnTrack(locSmoothed, inParamVec[0].offsetDist, locVecAhead);

            char compensationFileName[50] = "c.txt";
            // write new location to file
            readFp = fopen(compensationFileName, "wt");
            if(readFp == NULL)
    		{
    			printf("cannot open the reader %s file\n", compensationFileName);
    			return false;
    		}

            for(int idx = 0; idx < locVecAhead.size(); idx++)
            {
                point3D_t locPoint;

                fprintf(readFp, "%.14lf,%.14lf\n",locVecAhead[idx].lat, locVecAhead[idx].lon);
            }

            fclose(readFp);
            readFp = fopen(compensationFileName, "rt");
            if(readFp == NULL)
    		{
    			printf("cannot open the reader %s file\n", compensationFileName);
    			return false;
    		}

            bufferSize = locVecAhead.size();

            if(0 >= bufferSize)
            {                
                readFileFlag = true; // read file already open, set the flag to true before return
                return false;
            }

    		// reset read index to 0
            readIdx = 0;
        }
		readFileFlag = true;
		return true;
	}    
	return false;
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

void ImageBuffer::getImageSize(int& width, int& height)
{
	width  = _imageWidth;
	height = _imageHeight; 
}

void ImageBuffer::setInParamIdx(int input)
{
    inParamIdx = input;
}

int ImageBuffer::getInParamIdx()
{
    return inParamIdx;
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

bool ImageBufferAll::addVideoAndGpsName(const char* imageFileName, const char* gpsFileName, const int aviGpsFileIdx)
{
	if(totalNum >= MAX_VIDEO_NUM)
	{
		//printf("file number is bigger than %d\n",MAX_VIDEO_NUM );
		std::stringstream msgStr;
		msgStr << "file number is bigger than "<< MAX_VIDEO_NUM;
		logPrintf(logLevelError_e,"ImageBuffer", msgStr.str().c_str());
		return false;
	}
	strcpy(aviNames[totalNum],imageFileName);
	strcpy(gpsNames[totalNum],gpsFileName);

    inParamIdxs[totalNum] = aviGpsFileIdx;

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
	imageBuffer->setInParamIdx(inParamIdxs[proIdx]);

	proIdx++;
	if(proIdx > totalNum)
	{
		//proIdx = 0;
        proIdx = totalNum;

        static bool firstTimeFlag = true;
        if (firstTimeFlag == true)
        {
            //printf("load files finished\n");
			logPrintf(logLevelInfo_e,"imageBuffer","load files finished!");

            firstTimeFlag = false;

            // Output kml file
            Sleep(30000); // 30000 = 30s

            point3D_t standPoint;
            standPoint.lat = inParamVec[0].GPSref.x;
            standPoint.lon = inParamVec[0].GPSref.y;
            database_gp->saveRoadVecAndFurToKml("log/roadVec_furniture_vehicle.kml", standPoint);
        }
        return false;
    }

    if(false == imageBuffer->openReadFiles())
    {
        return false;
    }

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
			//printf("cannot open the %s file\n",saveFileName[0]);
			std::stringstream msgStr;
			msgStr << "cannot open the "<< saveFileName[0] << "file";
			logPrintf(logLevelInfo_e,"ImageBuffer",msgStr.str().c_str());
			return false;
		}
		writeFp = fopen(saveFileName[1],"w");
		if(writeFp == NULL)
		{
			//printf("cannot open the %s file\n",saveFileName[1]);
			std::stringstream msgStr;
			msgStr << "cannot open the "<< saveFileName[1] << "file";
			logPrintf(logLevelInfo_e,"ImageBuffer",msgStr.str().c_str());
			return false;
		}
		writeFileFlag = true;
	}
	return true;
}

bool ImageBuffer::closeWriteFiles()
{
    WaitForSingleObject(_hMutex,INFINITE);
	if(writeFileFlag)
	{
	delete writer;
	fclose(writeFp);
	writeFileFlag = false;
	}
    ReleaseMutex(_hMutex);
	return true;
}

bool ImageBuffer::openReadFiles()
{
	if(!readFileFlag)
	{
		reader.open(saveFileName[0]);
		if(!reader.isOpened())
		{
			//printf("cannot open reader %s file\n",saveFileName[0]);
			std::stringstream msgStr;
			msgStr << "cannot open reader "<< saveFileName[0] << "file";
			logPrintf(logLevelInfo_e,"ImageBuffer",msgStr.str().c_str());
			return false;
		}
		readFp = fopen(saveFileName[1],"r");
		if(readFp == NULL)
		{
			//printf("cannot open the reader %s file\n",saveFileName[1]);
			std::stringstream msgStr;
			msgStr << "cannot open reader "<< saveFileName[1] << "file";
			logPrintf(logLevelInfo_e,"ImageBuffer",msgStr.str().c_str());
			return false;
		}

        // Distance compensation to minimize systematic error
        //if (0)
        {
            // read existing location from file
            std::vector<point3D_t> locVec, locSmoothed,locVecAhead;

            for(int idx = 0; idx < bufferSize; idx++)
            {
                point3D_t locPoint;

                fscanf(readFp,"%lf,%lf\n",&locPoint.lat, &locPoint.lon);

                locVec.push_back(locPoint);
    		}

            // smooth the gps by 7-order average filter
            ns_database::smoothGps(7,locVec,locSmoothed);

            // skip 3 frames
            imageInfo_t image;
            getCurrentImage(&image);
            getCurrentImage(&image);
            getCurrentImage(&image);

    		fclose(readFp);

            // look ahead
            ns_database::lookAheadOnTrack(locSmoothed, inParamVec[inParamIdx].offsetDist, locVecAhead);

            char compensationFileName[50];
            int fileNameLen = strlen(saveFileName[1]);
            
            memset(compensationFileName, 0, sizeof(compensationFileName));
            memcpy(compensationFileName, saveFileName[1], fileNameLen-4);
            strcat(compensationFileName, "_c.txt");

            // write new location to file
            readFp = fopen(compensationFileName, "wt");
            if(readFp == NULL)
    		{
    			printf("cannot open the reader %s file\n", compensationFileName);
    			return false;
    		}		

            for(int idx = 0; idx < locVecAhead.size(); idx++)
            {
                point3D_t locPoint;

                fprintf(readFp, "%.14lf,%.14lf\n",locVecAhead[idx].lat, locVecAhead[idx].lon);
            }

            fclose(readFp);
            readFp = fopen(compensationFileName, "rt");
            if(readFp == NULL)
    		{
    			printf("cannot open the reader %s file\n", compensationFileName);
    			return false;
    		}

            bufferSize = locVecAhead.size();
            if(0 >= bufferSize)
            {                
		        readFileFlag = true; // read file already open, set the flag to true before return
                return false;
            }

            // reset read index to 0
            readIdx = 0;
        }
		readFileFlag = true;
		return true;
	}
    
	return false;
}

bool ImageBuffer::openReadFiles_nofilter(void)
{
	if(!readFileFlag)
	{
		reader.open(saveFileName[0]);
		if(!reader.isOpened())
		{
			//printf("cannot open reader %s file\n",saveFileName[0]);
			std::stringstream msgStr;
			msgStr << "cannot open reader "<< saveFileName[0] << "file";
			logPrintf(logLevelInfo_e,"ImageBuffer",msgStr.str().c_str());
			return false;
		}
		readFp = fopen(saveFileName[1],"r");
		if(readFp == NULL)
		{
			//printf("cannot open the reader %s file\n",saveFileName[1]);
			std::stringstream msgStr;
			msgStr << "cannot open reader "<< saveFileName[1] << "file";
			logPrintf(logLevelInfo_e,"ImageBuffer",msgStr.str().c_str());
			return false;
		}
		readFileFlag = true;
		return true;
	}
	return false;
}


bool ImageBuffer::closeReadFiles()
{
    WaitForSingleObject(_hMutex,INFINITE);
	if(readFileFlag)
	{
		reader.release();
		fclose(readFp);
	}
	readFileFlag = false;
    ReleaseMutex(_hMutex);
	return true;
}

bool ImageBuffer::addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction, int inParamIdx)
{
	WaitForSingleObject(_hMutex,INFINITE);
	
	//int pendFlag = 0;
	
	if(!ready_flag)
	{
		if(writeIdx == 0)
		{
			bool flag = openWriteFiles(image.size());

            setImageSize(image.cols, image.rows);

            setInParamIdx(inParamIdx);
		}
		//Buffer[writeIdx].image = image.clone();
		if(writeFileFlag)
		{
			writer->write(image);
			fprintf(writeFp,"%.14lf,%.14lf\n",gpsInfo.lat,gpsInfo.lon);
			writeIdx++;
			bufferSize = writeIdx;
			if(writeIdx >= IMAGE_BUFFER_DEPTH)
			{
				writeIdx = 0;
				closeWriteFiles();
				ready_flag = true;
				//printf("image buffer is full\n");
				logPrintf(logLevelInfo_e,"ImageBuffer","image buffer is full!",FOREGROUND_BLUE|FOREGROUND_GREEN);
			}
		}
		ReleaseMutex(_hMutex);
		return true;
	}
	
	ReleaseMutex(_hMutex);
	return false;
}

bool ImageBuffer::setImageToStart(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	reader.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
    // skip 3 frames to match with smoothGps
    imageInfo_t image;
    getCurrentImage(&image);
    getCurrentImage(&image);
    getCurrentImage(&image);

	fseek(readFp,0,SEEK_SET);
	readIdx = 0;
	ReleaseMutex(_hMutex);
	return true;
}

bool ImageBuffer::getCurrentImage(imageInfo_t *outImage)
{
	WaitForSingleObject(_hMutex,INFINITE);
	//*outImage = &Buffer[readIdx];
	bool status = reader.read(outImage->image);

    if(status == false)
    {
        logPrintf(logLevelError_e, "ImageCollector", "Failed to get image from buffer",FOREGROUND_RED);
        ReleaseMutex(_hMutex);
        return false;
    }

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
        // skip 3 frames to match with smoothGps
        imageInfo_t image;
        reader.read(image.image);
        reader.read(image.image);
        reader.read(image.image);

		fseek(readFp,0,SEEK_SET);
	}
	ReleaseMutex(_hMutex);
	return true;
}

int ImageBuffer::getImageNumber(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
    int buffSizeTmp = bufferSize;
    ReleaseMutex(_hMutex);
    return buffSizeTmp;
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
    bool flagTmp = ready_flag;
	ReleaseMutex(_hMutex);

	return flagTmp;
}

void ImageBuffer::setReadyFlag(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	ready_flag = true;
	closeWriteFiles();
	ReleaseMutex(_hMutex);
}

void ImageBuffer::setInParamIdx(int input)
{
    WaitForSingleObject(_hMutex,INFINITE);
    inParamIdx = input;
    ReleaseMutex(_hMutex);
}

int ImageBuffer::getInParamIdx()
{
    WaitForSingleObject(_hMutex,INFINITE);
    int inParamIdxTmp = inParamIdx;
    ReleaseMutex(_hMutex);
    return inParamIdxTmp;
}

void ImageBuffer::setImageSize(int width,int height)
{
    WaitForSingleObject(_hMutex,INFINITE);
    _imageHeight = height;
    _imageWidth = width;
    ReleaseMutex(_hMutex);
}

void ImageBuffer::getImageSize(int& width,int& height)
{
    WaitForSingleObject(_hMutex,INFINITE);
    width = _imageWidth;
    height = _imageHeight;
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

bool ImageBufferAll::addImage(cv::Mat &image, point3D_t gpsInfo, point3D_t gpsInfoPre, float speed, float direction, int inParamIdx)
{
	WaitForSingleObject(_hMutex,INFINITE);
	if(saveFlag)
	{
		//imageBuffer[readIdx]->addImage(image,gpsInfo,gpsInfoPre,speed,direction, inParamIdx);
		 bool flag = imageBuffer[readIdx]->addImage(image,gpsInfo,gpsInfoPre,speed,direction,inParamIdx);
         ReleaseMutex(_hMutex);
         return flag;
    }
    ReleaseMutex(_hMutex);
    return false;
}

bool ImageBufferAll::getBuffer(ImageBuffer **buffer)
{
	WaitForSingleObject(_hMutex,INFINITE);
	saveFlag = true;
	if(imageBuffer[readIdx]->getReadyFlag())
	{
		int oldIdx = readIdx;
		*buffer = (imageBuffer[readIdx]);
		//imageBuffer[readIdx]->closeWriteFiles();
		//imageBuffer[readIdx]->openReadFiles();
		readIdx = (readIdx+1)%IMAGE_BUFFER_NUM;
		imageBuffer[readIdx]->cleanBuffer();
		imageBuffer[readIdx]->closeReadFiles();

#if(ON == RD_PAUSE_BUFFER_FULL)  //copy previous half buffer to current buffer
		
		if(imageBuffer[oldIdx]->getImageNumber() == IMAGE_BUFFER_DEPTH)
		{
			imageBuffer[oldIdx]->openReadFiles_nofilter();
			int inParamIdx = imageBuffer[oldIdx]->getInParamIdx();
			
			for(int idx = 0; idx < IMAGE_BUFFER_DEPTH; idx++)
			{
				imageInfo_t image;
				imageBuffer[oldIdx]->getCurrentImage(&image);
				if(idx >= (IMAGE_BUFFER_DEPTH/2))
				{
					imageBuffer[readIdx]->addImage(image.image, image.gpsInfo, image.gpsInfoPre, image.speed, image.direction,inParamIdx);
				}
			}
			imageBuffer[oldIdx]->setImageToStart();
			imageBuffer[oldIdx]->closeReadFiles();
		}

#endif


		bool flag = imageBuffer[oldIdx]->openReadFiles();

		saveFlag = false;
		ReleaseMutex(_hMutex);
		return flag;
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
    int frameNum = imageBuffer[readIdx]->getImageNumber();
	ReleaseMutex(_hMutex);
	return frameNum;
}

void ImageBufferAll::setReadyFlag(void)
{
	WaitForSingleObject(_hMutex,INFINITE);
	imageBuffer[readIdx]->setReadyFlag();
	ReleaseMutex(_hMutex);
}

void ImageBufferAll::setInParamIdx(int input)
{
    WaitForSingleObject(_hMutex,INFINITE);
    imageBuffer[readIdx]->setInParamIdx(input);
    ReleaseMutex(_hMutex);
}

int ImageBufferAll::getInParamIdx()
{
    WaitForSingleObject(_hMutex,INFINITE);
    int inParamIdxTmp = imageBuffer[readIdx]->getInParamIdx();
    ReleaseMutex(_hMutex);
    return inParamIdxTmp;
}

void ImageBufferAll::setImageSize(int width,int height)
{
    WaitForSingleObject(_hMutex,INFINITE);
    imageBuffer[readIdx]->setImageSize(width, height);
    ReleaseMutex(_hMutex);
}

void ImageBufferAll::getImageSize(int& width,int& height)
{
    WaitForSingleObject(_hMutex,INFINITE);
    imageBuffer[readIdx]->getImageSize(width, height);
    ReleaseMutex(_hMutex);
}

#endif