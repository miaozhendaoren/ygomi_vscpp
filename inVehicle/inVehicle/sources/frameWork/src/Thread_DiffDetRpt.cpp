/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_DiffDetRpt.cpp
* @brief get the image data from Image sensors and get current GPS information 
*             process image data and detect traffic sign and other needed inforamtion
*             call Difference Detection Unit to compare the detect information and Digital Horizon Database information
*             send differnce data to Digital Horizon Data Difference Collector(some buffer can cache?)
*             then send the difference data to Server side.
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "svm.h"
#include "Detection.h"
#include "typeDefine.h"
#include "database.h"
#include "databaseInVehicle.h" // databaseInVehicle
#include "statisticsFurInfo.h"
#include "AppInitCommon.h"
#include "NEMA_GPGGA_Proc.h"
#include "LogInfo.h"
#include "messageProcessClass.h"

using namespace ns_database;
using std::list;
using cv::Mat;

#define USE_CAMERA 0
#define USE_VIDEO 1
#define USE_IMAGE 2
#define IMAGE_SOURCE USE_CAMERA

#define IMAGE_DISPLAY 1

#define SEND_MAX_BYTE_NUM 10000
#define IMAGE_WIDTH	800.00
#define IMAGE_HEIGHT 600.00	
#define FRAME_STATISTICS_NUM 50
#define FRAME_NUM_IN_RANG	5
#define SCREEN_WIDTH_LEFT_THRESHOLD (IMAGE_WIDTH/4)
#define SCREEN_WIDTH_RIGHT_THRESHOLD (3*IMAGE_WIDTH/4)

extern ns_database::databaseInVehicle* database_gp;
extern ns_database::databaseInVehicle* database_gpFinal;

short detectImage(int* number_of_frames,VideoCapture* capturePtr,Detector* detectorPtr,TS_Structure* targetPtr,int* detectNumberPtr);
void convertImageToFurniture(TS_Structure* targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,list<furAttributesInVehicle_t>* furnListPtr);
void filterFurToReport(point3D_t currentGps,list<statisticsFurInfo_t>* furnListInBuffPtr, list<furAttributesInVehicle_t>* furnListInDetPtr,list<furAttributesInVehicle_t>* outListPtr);
void compareFurnitureList(furAttributesInVehicle_t *furDetIn,list<furAttributesInVehicle_t>* list2, list<furAttributesInVehicle_t>* listAddOut, list<furAttributesInVehicle_t>* listUpdateOut);
int findDeleteFurnitures(point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevPtr,TS_Structure* detImageptr,list<furAttributesInVehicle_t>* furnitureListInDbPtr,list<deleteFurInfo_t>* furnitureListInBuffPtr, list<furAttributesInVehicle_t>* outputListPtr);

unsigned int __stdcall Thread_DiffDetRpt(void *data)
{
	//open the camera device.
	int nRet;
	int sendLen;
	int number_of_frames;
	char sendBuff[SEND_MAX_BYTE_NUM];
	list<statisticsFurInfo_t> furnListInBuff;
	list<deleteFurInfo_t> furnListInDelBuff;

	segAttributes_t segInfoPrev;
	memset((void*)&segInfoPrev,0,sizeof(segInfoPrev));
#if IMAGE_SOURCE == USE_CAMERA	
	VideoCapture capture(0);
	if (!capture.isOpened()) 
		logPrintf(logLevelError_e, "DIFF_DETECT", "Cannot open camera!");
	capture.set(CV_CAP_PROP_FPS ,30);
	capture.set(CV_CAP_PROP_FRAME_WIDTH ,IMAGE_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT ,IMAGE_HEIGHT);
#elif IMAGE_SOURCE == USE_VIDEO
	VideoCapture capture("./resource/Loop2.MOV"); // open the video file for reading
	//seek to the end of file
	capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
	number_of_frames = capture.get(CV_CAP_PROP_POS_FRAMES);
	if ( !capture.isOpened() )  // if not success, exit program
	{
		logPrintf(logLevelError_e, "DIFF_DETECT", "Cannot open the video file!");
		return -1;
	}
	else
	{
		capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0);

		double fps = capture.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video	
	}
#endif
	while(1)
	{
		point3D_t gpsInfo;
		point3D_t gpsInfoPrev;
		Detector  detector;
		TS_Structure target;
		int			detectNum;
		short		detectRet = -1;
		char sendMsgFlag = 0;

		//get current GPS inforamtion
		gpsInfo.lat = gGpsInfo.dLatitude;
		gpsInfo.alt = gGpsInfo.altitude;
		gpsInfo.lon = gGpsInfo.dLongitude;

		gpsInfoPrev.lat = gGpsInfo.dLatitudePre;
		gpsInfoPrev.lon = gGpsInfo.dLongitudePre;
		gpsInfoPrev.alt = gGpsInfo.altitudePre;

		//get a image from camera or vedio and process image
#if ((IMAGE_SOURCE == USE_CAMERA) || (IMAGE_SOURCE == USE_VIDEO))
		detectRet = detectImage(&number_of_frames,&capture,&detector,&target,&detectNum);
#endif
		// compare the difference between image detection and database.
		if(-1 != detectRet)
		{
			short delNumber = 0;
			short updateNum = 0;
			messageProcessClass	diffMsg;
			diffRptMsg_t*			msgHeaderPtr = diffMsg.getDiffRptMsg();
			void* payloadPtr = (void*)(diffMsg.getDiffRptMsgPayload());

			list<furAttributesInVehicle_t> furInfoListInDb;
			list<furAttributesInVehicle_t>::iterator furIndex; 
			list<furAttributesInVehicle_t> furInfoListInDet;
			list<furAttributesInVehicle_t> furInfoListAddReport;
            list<furAttributesInVehicle_t> furInfoListUpdateReport;
            list<furAttributesInVehicle_t> furInfoListDeleteReport;
			list<furAttributesInVehicle_t> furListTemp;


			//convert the image data structure
			if(0 != target.totalNumber)
			{	
				convertImageToFurniture(&target,&gpsInfo,&gpsInfoPrev,&furInfoListInDet);
			}

			//filter the image
			filterFurToReport(gpsInfo,&furnListInBuff, &furInfoListInDet,&furListTemp);

			if( furListTemp.size() > 0)
			{
				list<furAttributesInVehicle_t>::iterator furListTempIdx;
				for(furListTempIdx = furListTemp.begin();furListTempIdx != furListTemp.end(); ++furListTempIdx)
				{
					//get the furniture info according to the GPS.
					database_gp->getFurnitureByGps(&(furListTempIdx->location), database_gp->_distThreshFar, furInfoListInDb);

					compareFurnitureList(&(*furListTempIdx),&furInfoListInDb,&furInfoListAddReport, &furInfoListUpdateReport);

                    furInfoListInDb.clear();
				}
			}

			list<point3D_t> pointInRangeList;
			list<list<furAttributesInVehicle_t>> allFurnitureListinDb;
			database_gp->getAllFurnitures(allFurnitureListinDb);
			if (allFurnitureListinDb.size() == 0)
			{
				furnListInDelBuff.clear();
				furInfoListDeleteReport.clear();
			}
			else
			{
				database_gp->getLookAheadFurnitures(&gpsInfo, 50, 5, furInfoListInDb, pointInRangeList);

				findDeleteFurnitures(&gpsInfo,&gpsInfoPrev,&target,&furInfoListInDb,&furnListInDelBuff, &furInfoListDeleteReport);
			}

			{
				int32 payloadSize = 0;
				int pduIdx = 0;
				int32 payloadLen = 0;

				furIndex = furInfoListAddReport.begin();

				//for(furIndex = furInfoListReport.begin(); furIndex != furInfoListReport.end()-delNumber;furIndex++)
				// detect a new furniture or update a furniture
				//std::cout << "add new furniture number:" << addNumber<< std::endl;
				while(furIndex != furInfoListAddReport.end())
				{
					diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,addDatabase_e,payloadSize);
					database_gp->convFurnitureToTlv(&(*furIndex),ns_database::memory_e,&payloadPtr,&payloadLen);
					diffMsg.setDiffRptPduFntHeader(pduIdx,0,0);
					payloadSize += payloadLen;

					char furTypeString[100];
					sprintf(furTypeString,"Furniture \"%s\" detected",ns_database::ID2Name((*furIndex).type).c_str());
					logPrintf(logLevelInfo_e, "DIFF_DETECT", furTypeString,FOREGROUND_GREEN);

					furIndex++;
					pduIdx++;
				}

				// report the update furnitures
                furIndex = furInfoListUpdateReport.begin();

				while(furIndex != furInfoListUpdateReport.end())
				{
					diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,updateDatabase_e,payloadSize);
					database_gp->convFurnitureToTlv(&(*furIndex),ns_database::memory_e,&payloadPtr,&payloadLen);
					diffMsg.setDiffRptPduFntHeader(pduIdx,furIndex->segId,furIndex->furId);
					payloadSize += payloadLen;

					char furTypeString[100];
					sprintf(furTypeString,"Furniture \"%s\" updated",ns_database::ID2Name((*furIndex).type).c_str());
					logPrintf(logLevelInfo_e, "DIFF_DETECT", furTypeString,FOREGROUND_GREEN);

					furIndex++;
					pduIdx++;
				}

				// delete furniture
                furIndex = furInfoListDeleteReport.begin();

				while(furIndex != furInfoListDeleteReport.end())
				{
					diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,deleteDatabase_e,payloadSize);
					database_gp->convFurnitureToTlv(&(*furIndex),ns_database::memory_e,&payloadPtr,&payloadLen);
					diffMsg.setDiffRptPduFntHeader(pduIdx,0,0);
					payloadSize += payloadLen;

					char furTypeString[100];
					sprintf(furTypeString,"Furniture \"%s\" not detected",ns_database::ID2Name((*furIndex).type).c_str());
					logPrintf(logLevelInfo_e, "DIFF_DETECT", furTypeString,FOREGROUND_BLUE | FOREGROUND_GREEN);

					furIndex++;
					pduIdx++;
				}

                if (pduIdx > 0)
                {
				    sendLen = sizeof(diffMsgHeader_t) + 16*pduIdx + payloadSize;
				    diffMsg.setMsgHeader((uint32*) msgHeaderPtr,0x0001,g_VehicleID,lowLevel_e,payloadSize,pduIdx);
				    diffMsg.packedDiffRptMsg((uint32*)sendBuff);

				    sendMsgFlag = 1;
                }
			}
		}
		else
		{
			uint8	   existFlagInTemp;
			uint8	   existFlagInFinal;

			segAttributes_t segInfoInTemp;
			segAttributes_t segInfoInFinal;

			database_gp->getSegmentByGps(&gpsInfo, &existFlagInTemp, &segInfoInTemp);// temp database for test
			database_gpFinal->getSegmentByGps(&gpsInfo, &existFlagInFinal, &segInfoInFinal); 

			if((((1 == segInfoPrev.segId_used) && (segInfoPrev.segId != segInfoInTemp.segId)) || (0 == segInfoPrev.segId_used)) && (1 == (existFlagInTemp ^ existFlagInFinal)))
			{
				segInfoPrev = segInfoInTemp;

				messageProcessClass	diffMsg;
				int32 payloadLen[10];

				int idx;

				//database_gp->getSegmentById(segInfo.segId,segInfo);
				void* payloadPtr = (void*)(diffMsg.getDiffRptMsgPayload());
				// get info for segment
				database_gp->convSegmentToTlv(&segInfoPrev, ns_database::memory_e, &payloadPtr, &payloadLen[0]);
				// get info for vector in a specified segment
				database_gp->getVectorsInSegTlv(segInfoPrev.segId,ns_database::memory_e,&payloadPtr,&payloadLen[1]);
				diffRptMsg_t*			msgHeaderPtr = diffMsg.getDiffRptMsg();
				// for segment info
				diffMsg.setMsgHeader((uint32*) msgHeaderPtr,0x0001,g_VehicleID,lowLevel_e,payloadLen[0] + payloadLen[1],2);
				diffMsg.setDiffRptPduMsgHeader(0,0,addDatabase_e,0);
				diffMsg.setDiffRptPduMsgHeader(1,6,addDatabase_e,payloadLen[0]);
				sendLen = payloadLen[0] + payloadLen[1];
				for(idx = 0; idx < msgHeaderPtr->msgHeader.numPDUs;idx++)
				{		
					switch(msgHeaderPtr->msgHeader.msgTypeID)
					{
					case furnitureElement_e:
					case furnitureSign_e:
					case vectorElement_e:
						sendLen = sendLen + 8;
						break;
					}
				}
				diffMsg.packedDiffRptMsg((uint32*)sendBuff);
				sendLen = sendLen + sizeof(diffMsgHeader_t) + msgHeaderPtr->msgHeader.numPDUs*8;
				sendMsgFlag = 1;
			}
			else
			{
				segInfoPrev = segInfoInTemp;
			}
		}
		//send the difference data to server side.
		if (1 == sendMsgFlag)
		{
			nRet = sendto(sockServer,(char*)sendBuff,sendLen,0,(SOCKADDR*)&serverAddr,g_SocketLen);//send response message data
			if ((nRet == SOCKET_ERROR) || (nRet == 0))
			{
				logPrintf(logLevelInfo_e, "DIFF_DETECT", "Send message to server failed!");
			}
			else
			{
				logPrintf(logLevelInfo_e, "DIFF_DETECT", "<<<< Send message to server OK");
			}
		}
	}
#if ((IMAGE_SOURCE == USE_CAMERA) && (IMAGE_SOURCE == USE_VIDEO))
	capture.release();
#endif
	return 0;
}
int findDeleteFurnitures(point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevPtr,TS_Structure* detImageptr,list<furAttributesInVehicle_t>* furnitureListInDbPtr,list<deleteFurInfo_t>* furnitureListInBuffPtr, list<furAttributesInVehicle_t>* outputListPtr)
{
	// furnitureListInDbPtr is the furniture in database, furnitureListInBuffPtr is history list.
	// find out the deleted furnitures
	int delNum = 0;

	//step 1: increase the statistic frame number
	list<deleteFurInfo_t>::iterator furnitureListInBuffIdx = furnitureListInBuffPtr->begin();
	while(furnitureListInBuffIdx !=  furnitureListInBuffPtr->end())
	{
		furnitureListInBuffIdx->frameNumber++;
		++furnitureListInBuffIdx;
	}

	// step 2: check the furniture in database, if has new furniture, add to storage list.
	list<furAttributesInVehicle_t>::iterator furnitureListInDbIdx = furnitureListInDbPtr->begin();
	// furniture in database
	while(furnitureListInDbIdx !=  furnitureListInDbPtr->end())
	{		
		bool listFlag = false;
		furnitureListInBuffIdx = furnitureListInBuffPtr->begin();
		while(furnitureListInBuffIdx !=  furnitureListInBuffPtr->end())
		{
			// same furniture in database
			if( checkTwoFurnitureSame( &(furnitureListInBuffIdx->furAttri), &(*furnitureListInDbIdx), database_gp->_distThreshNear, database_gp->_angleThresh))
			{	
				listFlag = true;
				furnitureListInBuffIdx->delNumber ++;
				break;
			}
			++furnitureListInBuffIdx;
		}
		if(	!listFlag) // add furniuture to storage list
		{
			deleteFurInfo_t temFur;
			temFur.delNumber = 1;
			temFur.detNumber = 0;
			temFur.frameNumber = 0;
			temFur.furAttri = *furnitureListInDbIdx;
			furnitureListInBuffPtr->push_back(temFur);
		}
		++furnitureListInDbIdx;
	}
	list<deleteFurInfo_t>::iterator idxTmp;
	furnitureListInBuffIdx = furnitureListInBuffPtr->begin();
	//step 3: compare the storage list with image, if furnitures has been detected in image, increase the detected number;
	while(furnitureListInBuffIdx != furnitureListInBuffPtr->end())
	{
		int imageIdx;
		int imageSize = detImageptr->totalNumber;
		bool detFlag = false;
		for(imageIdx = 0; imageIdx < imageSize;imageIdx++)
		{
			if(furnitureListInBuffIdx->furAttri.type == detImageptr->TS_type[imageIdx])
			{
				int side;
				double x_centre = detImageptr->TS_rect[imageIdx].x + detImageptr->TS_rect[imageIdx].width/2.0;
				if(x_centre < (IMAGE_WIDTH/2.0))
				{ 
					side = 2;
				}
				else
				{ 
					side = 1;
				}
				furAttributesInVehicle_t furniture;
				calcGpsRoadSide(gpsInfoPrevPtr, gpsInfoPtr, side, 5, &(furniture.location));
				if(checkGpsInRange(&(furnitureListInBuffIdx->furAttri.location),&(furniture.location),database_gp->_distThreshFar))
				{
					if((furnitureListInBuffIdx->furAttri.sideFlag_used == 1) && (furnitureListInBuffIdx->furAttri.sideFlag == side))
					{
						furnitureListInBuffIdx->detNumber++;
						break;
					}
				}
			}
		}
		++furnitureListInBuffIdx;
	}
	//step3: report the furniture need to delete
	furnitureListInBuffIdx = furnitureListInBuffPtr->begin();
	while(furnitureListInBuffIdx != furnitureListInBuffPtr->end())
	{
		// step 4: find the funiture is not in database(out of the look head rang)
		{
			bool flag = false;
			furnitureListInDbIdx = furnitureListInDbPtr->begin();
			while(furnitureListInDbIdx !=  furnitureListInDbPtr->end())
			{
				if( checkTwoFurnitureSame( &(furnitureListInBuffIdx->furAttri), &(*furnitureListInDbIdx), database_gp->_distThreshNear, database_gp->_angleThresh))
				{
					flag = true;
					break;
				}
				++furnitureListInDbIdx;
			}
			// out of look head rang
			if(!flag) 
			{
				if((furnitureListInBuffIdx->delNumber > FRAME_NUM_IN_RANG) && (furnitureListInBuffIdx->detNumber == 0) && (furnitureListInBuffIdx->frameNumber > FRAME_NUM_IN_RANG))
				{
					outputListPtr->push_back(furnitureListInBuffIdx->furAttri);
					delNum++;
				}
				idxTmp = furnitureListInBuffIdx;
				++furnitureListInBuffIdx;
				furnitureListInBuffPtr->erase(idxTmp);
			}
			else
			{
				++furnitureListInBuffIdx;
			}
		}
	}
	return delNum;
}

void compareFurnitureList(furAttributesInVehicle_t *furDetIn,list<furAttributesInVehicle_t>* list2, list<furAttributesInVehicle_t>* listAddOut, list<furAttributesInVehicle_t>* listUpdateOut)
{
	list<furAttributesInVehicle_t>::iterator idx2;

	// check the furniture info in list1, including the updated info and adding info
	// list2 is the furniture in database for current GPS.
	
	bool typeFindFlag = false;

	//compare type
	for(idx2 = list2->begin();idx2 != list2->end();idx2++)
	{
		furAttributesInVehicle_t* furPtr2 = &(*idx2);
		 // TODO: compare angle of furniture
		if(checkTwoFurnitureSameNoRange(&(*furDetIn), &(*idx2), database_gp->_angleThresh))
		// Update existing furniture
		{
            typeFindFlag = true;

			furDetIn->segId_used = 1;
			furDetIn->segId = furPtr2->segId;
			furDetIn->furId_used = 1;
			furDetIn->furId = furPtr2->furId;
			listUpdateOut->push_back(*furDetIn);
			break;
		}
     } // end for
	if(!typeFindFlag)
	// Add new furniture
	{
		listAddOut->push_back(*furDetIn);
	}     
}

short detectImage(int* number_of_frames,VideoCapture* capturePtr,Detector* detectorPtr,TS_Structure* targetPtr,int* detectNumberPtr)
{
	static int frameNumber;
	cv::Mat image;
	int ID = 0;
	{	

#if IMAGE_SOURCE == USE_CAMERA
		*capturePtr >> image;
#elif IMAGE_SOURCE == USE_VIDEO	
		int Decimation = 2;
		//count frames
		//double fps = capturePtr->get(CV_CAP_PROP_FPS); //get the frames per seconds of the video	
		*number_of_frames = *number_of_frames - Decimation;
		if (*number_of_frames >= 0)
		{
			for (int i = 0; i< Decimation; i++)
				*capturePtr >> image;
		}
		else
		{
			capturePtr->set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			*number_of_frames = capturePtr->get(CV_CAP_PROP_POS_FRAMES);
			capturePtr->isOpened();
			capturePtr->set(CV_CAP_PROP_POS_AVI_RATIO, 0);

			double fps = capturePtr->get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
			for (int i = 0; i< Decimation; i++)
				*capturePtr >> image;
		}				
		//resize(image, image, Size(IMAGE_WIDTH,IMAGE_HEIGHT));
		resize(image, image, Size(960,540));
		//540 960
		//if (!capture.read(image))break;
#elif IMAGE_SOURCE == USE_IMAGE
		char currFileName[1000];
		//sprintf_s( currFileName, 1000, ".\\yellow\\001\\%04d.jpg",ID++);		
		//sprintf_s( currFileName, 1000, ".\\blue\\001\\%04d.jpg",ID++);
		image= cv::imread(currFileName);		
#endif	
		if (!image.data) 
		{
			logPrintf(logLevelError_e, "DIFF_DETECT", "NO image data!");
			return -1;
		}
		/////////////////////////////////////////////////////////////////////////////////////////


		//double startT = static_cast<double>(cv::getTickCount());
		//*detectNumberPtr = detectorPtr->TS_Detect_withTrack(image,*detectorPtr,*targetPtr); 
		*detectNumberPtr = detectorPtr->TS_Detect_withTrack(image,*detectorPtr,*targetPtr,frameNumber++);
		//double time1 = (static_cast<double>(cv::getTickCount())-startT)/cv::getTickFrequency();

		if (*detectNumberPtr >= 2)
		{	
			//std::cout << "detectNumber = " << *detectNumberPtr << std::endl;
		}
		//std::cout << "satDetection = " << time1 << std::endl;
	}

	return 0;
}
void convertImageToFurniture(TS_Structure* targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,list<furAttributesInVehicle_t>* furnListPtr)
{
	int idx;

	TS_Structure tempTarget;
	memcpy((void*)&tempTarget,(void*)targetPtr,sizeof(tempTarget));
	//delete the same type in a same frame.
	int maxArea = tempTarget.TS_area[0];
	for(idx = 0;idx < tempTarget.totalNumber - 1; idx++)
	{
		if(tempTarget.TS_type[idx] == tempTarget.TS_type[idx+1])
		{
			double x_centre = tempTarget.TS_rect[idx].x + tempTarget.TS_rect[idx].width/2.0;
			double x_centreNext = tempTarget.TS_rect[idx+1].x + tempTarget.TS_rect[idx+1].width/2.0;
			short side;
			short sideNext;
			if(x_centre < IMAGE_WIDTH/2)
			{  side = 1;}//left
			else
			{  side = 2;}
			if(x_centreNext < IMAGE_WIDTH/2)
			{  sideNext = 1;}//left
			else
			{  sideNext = 2;}
			if(side == sideNext)
			{
				if(maxArea > tempTarget.TS_area[idx+1])
				{ targetPtr->TS_type[idx+1] = 0; }
				else
				{targetPtr->TS_type[idx] = 0; maxArea = targetPtr->TS_type[idx+1];}
			}
		}
		else
		{
			maxArea = tempTarget.TS_area[idx+1];
		}
	}
	// delete the furniture in the middle of the screen

	for(idx = 0;idx < tempTarget.totalNumber; idx++)
	{
		double x_centre = tempTarget.TS_rect[idx].x + tempTarget.TS_rect[idx].width/2.0;
		if ((x_centre > SCREEN_WIDTH_LEFT_THRESHOLD) && (x_centre < SCREEN_WIDTH_RIGHT_THRESHOLD))
		{
			targetPtr->TS_type[idx] = 0;
		}
	}

	for(idx = 0; idx < targetPtr->totalNumber; idx++)
	{
		furAttributesInVehicle_t furniture;
		memset((void*)&furniture,0,sizeof(furniture));
		if( targetPtr->TS_type[idx] != 0)
		{
			uint8 side;
			furniture.type_used = 1;
			furniture.type = targetPtr->TS_type[idx];

			furniture.reliabRating = 1;	
			furniture.reliabRating_used = 1;
			furniture.reliabRating = 1;	
			// compute angle
			furniture.angle_used = 1;
			calcNormalAngle(gpsInfoPrevP,gpsInfoPtr, &(furniture.angle));
			// compute location
			furniture.location_used = 1;
			double x_centre = targetPtr->TS_rect[idx].x + targetPtr->TS_rect[idx].width/2.0;
			if(x_centre < (IMAGE_WIDTH/2.0))
			{ 
				side = 2;
				furniture.sideFlag_used = 1;
				furniture.sideFlag = 2;
			}
			else
			{ 
				side = 1;
				furniture.sideFlag_used = 1;
				furniture.sideFlag = 1;
			}
			calcGpsRoadSide(gpsInfoPrevP, gpsInfoPtr, side, 5, &(furniture.location));

			//compute the height
			furniture.location.alt = (IMAGE_HEIGHT - (targetPtr->TS_rect[idx].y + targetPtr->TS_rect[idx].height/2))/IMAGE_HEIGHT;

			furnListPtr->push_back(furniture);
		}
	}
}
void filterFurToReport(point3D_t currentGps,list<statisticsFurInfo_t>* furnListInBuffPtr, list<furAttributesInVehicle_t>* furnListInDetPtr,list<furAttributesInVehicle_t>* outListPtr)
{
	list<statisticsFurInfo_t>::iterator furnListInBuffIdx = furnListInBuffPtr->begin();
	list<furAttributesInVehicle_t>::iterator furnListInDetIdx = furnListInDetPtr->begin();

	//step1: find out the new detected furniture and push to the history list.
	while(furnListInDetIdx != furnListInDetPtr->end())
	{
		bool findTypeFlag = false;
        furnListInBuffIdx = furnListInBuffPtr->begin();
		while(furnListInBuffIdx != furnListInBuffPtr->end())
		{
			if(checkTwoFurnitureSameNoRange(&(*furnListInDetIdx), &(furnListInBuffIdx->furAttri), database_gp->_angleThresh))
			{
				findTypeFlag = true;
				break;
			}
			++furnListInBuffIdx;
		}
		if(!findTypeFlag)
		{
			statisticsFurInfo_t countFurInfo;
			countFurInfo.firstGps = currentGps;
			countFurInfo.furAttri = *furnListInDetIdx;
			furnListInBuffPtr->push_back(countFurInfo);
		}
		++furnListInDetIdx;
	}
	//step 2: check which furniutr need to be reported
	list<statisticsFurInfo_t>::iterator idxTmp;
	furnListInBuffIdx = furnListInBuffPtr->begin();

	while(furnListInBuffIdx != furnListInBuffPtr->end())
	{
		bool findTypeFlag = false;
        furnListInDetIdx = furnListInDetPtr->begin();
		while(furnListInDetIdx != furnListInDetPtr->end())
		{
			if(checkTwoFurnitureSameNoRange(&(*furnListInDetIdx), &(furnListInBuffIdx->furAttri), database_gp->_angleThresh))
			// update furniture in the history list
			{
				furnListInBuffIdx->firstGps = currentGps;
				furnListInBuffIdx->furAttri = *furnListInDetIdx;
				findTypeFlag = true;
				break;
			}
			++furnListInDetIdx;
		}
		if(!findTypeFlag)
		{
			if(!checkGpsInRange(&currentGps,&(furnListInBuffIdx->firstGps),database_gp->_distThreshFarFar))
			// report and delete the reported furniture in history list
			{
				outListPtr->push_back(furnListInBuffIdx->furAttri);
				idxTmp = furnListInBuffIdx;
				++furnListInBuffIdx;
				furnListInBuffPtr->erase(idxTmp);
			}
			else
			{
				++furnListInBuffIdx;
			}
		}
		else
		{
			++furnListInBuffIdx;;
		}
	}
}
