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
#include <stdio.h>
#include <process.h>
#include <list>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "svm.h"
#include "Detection.h"
#include "typeDefine.h"
#include "Thread_DiffDetRpt.h"
#include "Signal_Thread_Sync.h"
#include "NEMA_GPGGA_Proc.h"
#include "statisticsFurInfo.h"
#include "LogInfo.h"

using namespace ns_database;
using namespace std;
using namespace cv;


extern ns_database::database* database_gp;
extern ns_database::database* database_gpFinal;

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
	memset((void*)&segInfoPrev,0,sizeof(segAttributes_t));
	#if IMAGE_SOURCE == USE_CAMERA	
		VideoCapture capture(1);
		if (!capture.isOpened()) 
			logPrintf(logLevelError_e, "DIFF_DETECT", "Cannot open camera!");
		capture.set(CV_CAP_PROP_FPS ,30);
		capture.set(CV_CAP_PROP_FRAME_WIDTH ,IMAGE_WIDTH);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT ,IMAGE_HEIGHT);
	#elif IMAGE_SOURCE == USE_VIDEO
		VideoCapture capture("./Resource/Loop2.MOV"); // open the video file for reading
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
			messageProcessClass	diffMsg;
			diffRptMsg_t*			msgHeaderPtr = diffMsg.getDiffRptMsg();
			void* payloadPtr = (void*)(diffMsg.getDiffRptMsgPayload());

			list<furAttributes_t> furInfoListInDb;
			list<furAttributes_t>::iterator furIndex; 
			list<furAttributes_t> furInfoListInDet;
			list<furAttributes_t> furInfoListReport;
			list<furAttributes_t> furListTemp;
			point3D_t gpsInfoTemp;
			furAttributes_t furTemp;
			
			//convert the image data structure
			if(0 != target.totalNumber)
			{	
				convertImageToFurniture(&target,&gpsInfo,&gpsInfoPrev,&furInfoListInDet);
			}

			//filter the image
			filterFurToReport(&furnListInBuff, &furInfoListInDet,&furListTemp);

			if( furListTemp.size() > 0)
			{
				furTemp = furListTemp.front();
				gpsInfoTemp.alt = furTemp.location.alt;
				gpsInfoTemp.lat = furTemp.location.lat;
				gpsInfoTemp.lon = furTemp.location.lon;
			}
			else
			{
				gpsInfoTemp.alt = gpsInfo.alt;
				gpsInfoTemp.lat = gpsInfo.lat;
				gpsInfoTemp.lon = gpsInfo.lon;
			}
				//get the furniture info according to the GPS.
				database_gp->getFurnitureByGps(&gpsInfoTemp, furInfoListInDb);
				compareFurnitureList(&furListTemp,&furInfoListInDb,&furInfoListReport);
				
				furInfoListInDb.clear();
				list<point3D_t> pointInRangeList;
				database_gp->getLookAheadFurnitures(&gpsInfo,70, furInfoListInDb, pointInRangeList);

				delNumber = findDeleteFurnitures(&target,&furInfoListInDb,&furnListInDelBuff, &furInfoListReport);
				
				if (furInfoListReport.size() > 0)
				{
					int32 payloadSize = 0;
					int pduIdx = 0;;
					int16 furNumber = furInfoListReport.size();
					int32 payloadLen = 0;
					short addNumber = furInfoListReport.size() - delNumber;

					furIndex = furInfoListReport.begin();

					//for(furIndex = furInfoListReport.begin(); furIndex != furInfoListReport.end()-delNumber;furIndex++)
					// dedect a new furniture or update a furniture
					while(addNumber--)
					{
						diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,addDatabase_e,payloadSize);
						database_gp->convFurnitureToTlv(&(*furIndex),memory_e,&payloadPtr,&payloadLen);
						diffMsg.setDiffRptPduFntHeader(pduIdx,0,0);
						payloadSize += payloadLen;

						char furTypeString[100];
						sprintf(furTypeString,"Furniture type \"%s\" detected",ID2Name((*furIndex).type).c_str());
						logPrintf(logLevelInfo_e, "DIFF_DETECT", furTypeString);

						furIndex++;
						pduIdx++;
					}
					// delete furniture 
					while(delNumber--)
					{
						diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,deleteDatabase_e,payloadSize);
						database_gp->convFurnitureToTlv(&(*furIndex),memory_e,&payloadPtr,&payloadLen);
						diffMsg.setDiffRptPduFntHeader(pduIdx,0,0);
						payloadSize += payloadLen;

						char furTypeString[100];
						sprintf(furTypeString,"Furniture type \"%s\" in DB not detected",ID2Name((*furIndex).type).c_str());
						logPrintf(logLevelInfo_e, "DIFF_DETECT", furTypeString);

						furIndex++;
						pduIdx++;
					}
					sendLen = sizeof(diffMsgHeader_t) + 16*pduIdx + payloadSize;
					diffMsg.setMsgHeader((uint32*) msgHeaderPtr,0x0001,g_VehicleID,lowLevel_e,payloadSize,furNumber);
					diffMsg.packedDiffRptMsg((uint32*)sendBuff);
				
					sendMsgFlag = 1;
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
					database_gp->convSegmentToTlv(&segInfoPrev, memory_e, &payloadPtr, &payloadLen[0]);
					// get info for vector in a specified segment
					database_gp->getVectorsInSegTlv(segInfoPrev.segId,memory_e,&payloadPtr,&payloadLen[1]);
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
					logPrintf(logLevelInfo_e, "DIFF_DETECT", "Send message to server OK");
				}
			}
	}
	#if ((IMAGE_SOURCE == USE_CAMERA) && (IMAGE_SOURCE == USE_VIDEO))
	capture.release();
	#endif
    return 0;
}
short findDeleteFurnitures(TS_Structure* detImageptr,list<furAttributes_t>* list1,list<deleteFurInfo_t>* list2, list<furAttributes_t>* list3)
{
	// list1 is the furniture in database, list2 is statistic list list 3 is report list.
	// find out the deleted furnitures
	short delNum = 0;

	//step 1: increase the statistic frame number
	if(list2->size() > 0)
	{
		list<deleteFurInfo_t>::iterator idx2 = list2->begin();
		while(idx2 !=  list2->end())
		{
			idx2->frameNumber++;
			idx2++;
		}
	}

	// step 2: check the furniture in database, if has new furniture, add to storage list.
	if(list1->size() > 0 )
	{
		list<furAttributes_t>::iterator idx1 = list1->begin();

		// furniture in database
		while(idx1 !=  list1->end())
		{		
			if(list2->size() > 0)
			{
				bool listFlag = false;
				list<deleteFurInfo_t>::iterator idx2 = list2->begin();
				while(idx2 != list2->end())
				{
					// same furniture in database
					if((idx1->type_used == 1) && (idx2->furAttri.type_used == 1) && (idx1->type == idx2->furAttri.type) && (database_gp->checkGpsInRange(&(idx1->location),&(idx2->furAttri.location),database_gp->_distThreshNear))
						&& ((idx1->sideFlag_used == 1) && (idx2->furAttri.sideFlag_used == 1) && (idx1->sideFlag == idx2->furAttri.sideFlag)))
					{	
						listFlag = 1;
						idx2->delNumber ++;
						break;
					}
					idx2++;
				}
				if(	0 == listFlag) // add furniuture to storage list
				{
					deleteFurInfo_t temFur;
					temFur.delNumber = 1;
					temFur.detNumber = 0;
					temFur.frameNumber = 0;
					memcpy((void*)&(temFur.furAttri),(void*)&(*idx1),sizeof(furAttributes_t));
					list2->push_back(temFur);
				}
			}
			else
			{
				deleteFurInfo_t temFur;
				temFur.delNumber = 1;
				temFur.detNumber = 0;
				temFur.frameNumber = 0;
				memcpy((void*)&(temFur.furAttri),(void*)&(*idx1),sizeof(furAttributes_t));
				list2->push_back(temFur);
			}
			idx1++;
		}
	}
	if(list2->size() > 0)
	{
		list<deleteFurInfo_t>::iterator idx2 = list2->begin();
		list<deleteFurInfo_t>::iterator idxTmp;
		//step 3: compare the storage list with image, if furnitures has been detected in image, increase the detected number;
		while(idx2 != list2->end())
		{
			int imageIdx;
			int imageSize = detImageptr->totalNumber;
			bool detFlag = false;
			for(imageIdx = 0; imageIdx < imageSize;imageIdx++)
			{
				if(idx2->furAttri.type == detImageptr->TS_type[imageIdx])
				{
					int side;
					if(detImageptr->TS_rect[imageIdx].x < (IMAGE_WIDTH/2))
					{ 
						side = 2;
					}
					else
					{ 
						side = 1;
					}
					if((idx2->furAttri.sideFlag_used == 1) && (idx2->furAttri.sideFlag == side))
					{
						idx2->detNumber++;
						break;
					}
				}
			}
			idx2++;
		}

		//step3: report the furniture need to delete
		idx2 = list2->begin();
		while(idx2 != list2->end())
		{
			// step 4: find the funiture is not in database(out of the look head rang)
			if(list1->size() > 0 )
			{
				bool flag = false;
				list<furAttributes_t>::iterator idx1 = list1->begin();
				while(idx1 !=  list1->end())
				{
					if((idx1->type_used == 1) && (idx2->furAttri.type_used == 1) && (idx1->type == idx2->furAttri.type) && (database_gp->checkGpsInRange(&(idx1->location),&(idx2->furAttri.location),database_gp->_distThreshNear))
						&& ((idx1->sideFlag_used == 1) && (idx2->furAttri.sideFlag_used == 1) && (idx1->sideFlag == idx2->furAttri.sideFlag)))
					{
						flag = true;
						break;
					}
					idx1++;
				}
				// out of look head rang
				if(!flag) 
				{
					if((idx2->delNumber > FRAME_NUM_IN_RANG) && (idx2->detNumber == 0) && (idx2->frameNumber > FRAME_NUM_IN_RANG))
					{
						list3->push_back(idx2->furAttri);
						delNum++;
					}
					idxTmp = idx2;
					idx2++;
					list2->erase(idxTmp);
				}
				else
				{
					idx2++;
				}
			}
			else
			{
				if((idx2->delNumber > FRAME_NUM_IN_RANG) && (idx2->detNumber == 0) && (idx2->frameNumber > FRAME_NUM_IN_RANG))
				{
					list3->push_back(idx2->furAttri);
					delNum++;
				}
				idxTmp = idx2;
				idx2++;
				list2->erase(idxTmp);
			}
		}
	}
	return delNum;
}
short compareFurnitureList(list<furAttributes_t>* list1,list<furAttributes_t>* list2, list<furAttributes_t>* list3)
{
	list<furAttributes_t>::iterator idx1;
	list<furAttributes_t>::iterator idx2;
	short delNum = 0;
	// check the furniture info in list1, including the updated info and adding info
	// list1 is the furniture detect in image, list2 is the furniture in database for current GPS.
	for(idx1 = list1->begin();idx1 != list1->end();idx1++)
	{
		furAttributes_t* furPtr1 = &(*idx1);
		bool typeFindFlag = 0;
		//compare type
		for(idx2 = list2->begin();idx2 != list2->end();idx2++)
		{
			furAttributes_t* furPtr2 = &(*idx2);
			// type is the same, comparing other attributions.
			if((furPtr1->type_used == 1) && (furPtr2->type_used == 1) && (furPtr1->type == furPtr2->type))
			{
				typeFindFlag = 1;
				if(((furPtr1->sideFlag_used == 1) && (furPtr2->sideFlag_used == 1) && (furPtr1->sideFlag != furPtr2->sideFlag))
				|| (furPtr1->sideFlag_used != furPtr2->sideFlag_used))
				{
					list3->push_back(*idx1);
					break;
				}
				// check side
				/*if(((furPtr1->side_used == 1) && (furPtr2->side_used == 1) && ((furPtr1->side[0] != furPtr2->side[0]) || (furPtr1->side[1] != furPtr2->side[1])))
					|| (furPtr1->side_used != furPtr2->side_used))
				{
					list3->push_back(*idx1);
					break;
				}*/
			   // check location
				if(((furPtr1->location_used == 1) && (furPtr2->location_used == 1) && (!database_gp->checkGpsInRange(&furPtr1->location,&furPtr2->location,database_gp->_distThreshFar)))
					|| (furPtr1->location_used != furPtr2->location_used))
				{
					list3->push_back(*idx1);
					break;
				}
				// check reliabRating
				if((furPtr2->reliabRating_used == 1) && (furPtr2->reliabRating < MAX_RALIABILITY))
				{
					list3->push_back(*idx1);
					break;
				}
				// check the side
				//if((furPtr2->sideFlag_used == 1) && (furPtr2->sideFlag_used == 1) && (furPtr2->sideFlag != furPtr1->sideFlag))
				//{
				//	list3->push_back(*idx1);
				//	break;
				//}
			}
		}
		if(0 == typeFindFlag)
		{
			list3->push_back(*idx1);
		}
	}
	return delNum;
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
void convertImageToFurniture(TS_Structure* targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,list<furAttributes_t>* furnListPtr)
{
	int idx;
	
	TS_Structure tempTarget;
	memcpy((void*)&tempTarget,(void*)targetPtr,sizeof(TS_Structure));
	//delete the same type in a same frame.
	//int maxArea = tempTarget.TS_area[0];
	//for(idx = 0;idx < tempTarget.totalNumber - 1; idx++)
	//{
	//	if(tempTarget.TS_type[idx] == tempTarget.TS_type[idx+1])
	//	{
	//		if(maxArea > tempTarget.TS_area[idx+1])
	//		{ targetPtr->TS_type[idx+1] = 0; }
	//		else
	//		{targetPtr->TS_type[idx] = 0; maxArea = targetPtr->TS_type[idx+1];}
	//	}
	//	else
	//	{
	//		maxArea = tempTarget.TS_area[idx+1];
	//	}
	//}
	// delete the furniture in the middle of the screen

	for(idx = 0;idx < tempTarget.totalNumber; idx++)
	{
		if (((tempTarget.TS_rect[idx].x + tempTarget.TS_rect[idx].width/2.0 ) > (IMAGE_WIDTH*SCREEN_WIDTH_LEFT_THRESHOLD)) && ((tempTarget.TS_rect[idx].x + tempTarget.TS_rect[idx].width/2 ) < (IMAGE_WIDTH*SCREEN_WIDTH_LEFT_THRESHOLD)))
		{
			targetPtr->TS_type[idx] = 0;
		}
	}

	for(idx = 0; idx < targetPtr->totalNumber; idx++)
	{
		furAttributes_t furniture;
		memset((void*)&furniture,0,sizeof(furAttributes_t));
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
			database_gp->calcNormalAngle(gpsInfoPrevP,gpsInfoPtr, &(furniture.angle));
			// compute location
			furniture.location_used = 1;

			if(targetPtr->TS_rect[idx].x < (IMAGE_WIDTH/2.0))
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
			database_gp->calcGpsRoadSide(gpsInfoPrevP, gpsInfoPtr, side, 5, &(furniture.location));

			//compute the height
			furniture.location.alt = (IMAGE_HEIGHT - (targetPtr->TS_rect[idx].y + targetPtr->TS_rect[idx].height/2))/IMAGE_HEIGHT;

			furnListPtr->push_back(furniture);
		}
	}
}
void filterFurToReport(list<statisticsFurInfo_t>* furnListInBuffPtr, list<furAttributes_t>* furnListInDetPtr,list<furAttributes_t>* outListPtr)
{
	list<statisticsFurInfo_t>::iterator idx1;
	list<furAttributes_t>::iterator idx2;

	// if the statistics lis is empty, copy the detected image to list.
	if((0 == furnListInBuffPtr->size()) && (0 != furnListInDetPtr->size()))
	{
		for(idx2 = furnListInDetPtr->begin();idx2 != furnListInDetPtr->end();idx2++)
		{
			furAttributes_t FurInfo = *idx2;
			statisticsFurInfo_t countFurInfo;
			countFurInfo.number = FRAME_STATISTICS_NUM;
			memcpy((void*)&(countFurInfo.furAttri),(void*)&FurInfo,sizeof(furAttributes_t));
			furnListInBuffPtr->push_back(countFurInfo);
		}
	}
	else if((0 != furnListInBuffPtr->size()) && (0 == furnListInDetPtr->size()))
	{
		for(idx1 = furnListInBuffPtr->begin();idx1 != furnListInBuffPtr->end();idx1++)
		{
			statisticsFurInfo_t countFurInfo = *idx1;
			idx1->number = idx1->number - 1;
			if(idx1->number <= 0)
			{
				outListPtr->push_back(idx1->furAttri);
			}
		}
	}
	else if((0 != furnListInBuffPtr->size()) && (0 != furnListInDetPtr->size()))// decrease 1 of the detected image' statistics number
	{
		for(idx1 = furnListInBuffPtr->begin();idx1 != furnListInBuffPtr->end();idx1++)
		{
			statisticsFurInfo_t countFurInfo = *idx1;
			bool findTypeFlag = false;
			for(idx2 = furnListInDetPtr->begin();idx2 != furnListInDetPtr->end();idx2++)
			{
				furAttributes_t furAttributes = *idx2;
				if((countFurInfo.furAttri.type_used == 1) && (furAttributes.type_used == 1) && (countFurInfo.furAttri.type == furAttributes.type)
					&& ((countFurInfo.furAttri.sideFlag_used == 1) && (furAttributes.sideFlag_used == 1) && (countFurInfo.furAttri.sideFlag == furAttributes.sideFlag)))
				{
					countFurInfo.number = FRAME_STATISTICS_NUM;
					memcpy((void*)&(idx1->furAttri),(void*)&furAttributes,sizeof(furAttributes_t));// update the exist image's latest information 
					findTypeFlag = true;
					break;
				}
			}
			if(!findTypeFlag)
			{
				idx1->number = idx1->number - 1;// find out the disapeared image to decrease one
				if(idx1->number <= 0)
				{
					outListPtr->push_back(idx1->furAttri);
				}
			}
		}
		// find out the new type in detected images and add to statistics list
		for(idx2 = furnListInDetPtr->begin();idx2 != furnListInDetPtr->end();idx2++)
		{
			furAttributes_t furAttributes = *idx2;
			statisticsFurInfo_t countFurInfo;
			bool findTypeFlag = false;
			for(idx1 = furnListInBuffPtr->begin();idx1 != furnListInBuffPtr->end();idx1++)
			{
				countFurInfo = *idx1;
				if((countFurInfo.furAttri.type_used == 1) && (furAttributes.type_used == 1) && (countFurInfo.furAttri.type == furAttributes.type)
					&& ((countFurInfo.furAttri.sideFlag_used == 1) && (furAttributes.sideFlag_used == 1) && (countFurInfo.furAttri.sideFlag == furAttributes.sideFlag)))
				{
					findTypeFlag = true;
					break;
				}
			}
			if(!findTypeFlag)// new type detected
			{
				countFurInfo.number = FRAME_STATISTICS_NUM;
				memcpy((void*)&(countFurInfo.furAttri),(void*)&furAttributes,sizeof(furAttributes_t));// update the exist image's latest information 
				furnListInBuffPtr->push_back(countFurInfo);
			}
		}
	}
	// delete furniture which the number is zero
	if(0 != furnListInBuffPtr->size())
	{
		int size = furnListInBuffPtr->size();
		idx1 = furnListInBuffPtr->begin();
		list<statisticsFurInfo_t>::iterator idxTmp;
		while(size--)
		{
			if(idx1->number <= 0)
			{	
				idxTmp = idx1;
				idx1++;
				furnListInBuffPtr->erase(idxTmp);
			}
			else
			{
				idx1++;
			}
		}
	}
}
