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
*      2015/05/08         Xin Shao        Create
*       2015/05/11          Qin Shi          Add message process
*******************************************************************************
*/
#include <stdlib.h>
#include <time.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "svm.h"
#include "Detection.h"
#include "detection_colored.h"
#include "detection_blackWhite.h"
#include "typeDefine.h"
#include "database.h"
#include "databaseInVehicle.h" // databaseInVehicle
#include "statisticsFurInfo.h"
#include "AppInitCommon.h"
#include "NEMA_GPGGA_Proc.h"
#include "LogInfo.h"
#include "messageProcessClass.h"
#include "Signal_Thread_Sync.h"
#include "saveLinePointInSafe.h"
#include "lineDetection.h"
#include "Thread_ImageSensorCollector.h"
#include "ImageBuffer.h"
#include "roadScan.h"

#include <iomanip>
#include <fstream>

using namespace ns_database;
using namespace std;
using cv::Mat;
using namespace ns_detection;
using namespace ns_roadScan;

double roadLen = 3.75;//m
double roadPixelNum = 400;

void compareFurnitureList(furAttributesInVehicle_t *furDetIn,list<furAttributesInVehicle_t>* list2, list<furAttributesInVehicle_t>* listAddOut, list<furAttributesInVehicle_t>* listUpdateOut);
void convertImageToFurniture(TS_Structure* targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,point3D_t* refGps,list<furAttributesInVehicle_t>* furnListPtr);
void filterFurToReport(point3D_t currentGps,list<statisticsFurInfo_t>* furnListInBuffPtr, list<furAttributesInVehicle_t>* furnListInDetPtr,list<furAttributesInVehicle_t>* outListPtr);

struct gpsCarInfo_t
{
    int laneId;
    int lineStyle;
    double laneWidth;//width;

    point3D_t gpsInfo;
    point3D_t gpsInfoPre;
};

void smoothLaneId(INOUT vector<int> &lineIdxVec)
{
    const int MAX_LANE_NUM = 2;
    const int CHANGE_LANE_NUM_THRESH = 2;
    const int CONTINUOUS_POINT_NUM_THRESH = 10;

    int size = lineIdxVec.size();
    int changeNumber = 0;
    int preLaneId;

    vector<int> statistic;
    statistic.assign(MAX_LANE_NUM, 0);

    vector<int> sectionNumVec;
    sectionNumVec.assign(size, 0);

    int sectionStartIdx = 0;
    int sectionNum = 1; // 1 for the first point

    // find statistics
    for(int index = 0; index < size; index++)
    {
        int curLaneId = lineIdxVec[index];
        statistic[curLaneId] ++;

        if(index > 0)
        {
            if(preLaneId != curLaneId)
            {
                changeNumber++;

                for(int idxInSec = sectionStartIdx; idxInSec < index; ++idxInSec)
                {
                    sectionNumVec[idxInSec] = sectionNum;
                }

                sectionStartIdx = index;

                sectionNum = 1;
            }else
            {
                sectionNum++;
            }
        }

        preLaneId = curLaneId;
    }

    for(int idxInSec = sectionStartIdx; idxInSec < size; ++idxInSec)
    {
        sectionNumVec[idxInSec] = sectionNum;
    }

    // smooth
    int trueLaneId;
    if(changeNumber > CHANGE_LANE_NUM_THRESH)
    {
        if(statistic[0] > statistic[1])
        {
            trueLaneId = 0;
        }else
        {
            trueLaneId = 1;
        }

        for(int index = 0; index < size; index++)
        {
            if((lineIdxVec[index] != trueLaneId) && 
                (sectionNumVec[index] < CONTINUOUS_POINT_NUM_THRESH))
            {
                lineIdxVec[index] = trueLaneId;
            }
        }
    }
}

unsigned int __stdcall Thread_DiffDetRpt(void *data)
{
    //open the camera device.
    int nRet;
    int sendLen;
    list<laneType_t> laneInfoList;

    ////////////////////////////////////////////////////////////////////////////
    int lineIndex = 0;
    double leftWidth = 0;
    double rightWidth = 0;
    //Step1:  Kalman initialization
    LineDetect  Linedetector;
    // For Vanishi Point
    KalmanFilter KF(4, 2, 0);
    Mat_<float> measurement(2,1); 
    measurement.setTo(Scalar(0));
    //Size S = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH), (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    Size S = Size(IMAGE_SENSOR_WIDTH,IMAGE_SENSOR_HEIGHT);
    Linedetector.initialVanishPointKF(KF,S);

    // For Lane Marker
    KalmanFilter LaneMarkKF(2, 2, 0);
    Mat_<float> measLandMark(2,1); 
    measLandMark.setTo(Scalar(0));
    Linedetector.iniLaneMarkKF(LaneMarkKF,S);        
    ////////////////////////////////////////////////////////////////////////////
    vector<dataEveryRow> roadPaintData;
    vector<gpsInformationAndInterval> GPSAndInterval;
    gpsInformationAndInterval gpsAndInterval;

    while(1)
    {
        ImageBuffer *buffer;
        if(!imageBuffer.getBuffer(&buffer))
        {
            Sleep(50);
            continue;
        }

        int number = buffer->getImageNumber();
        
        // Painting detection
        {
            Mat history = Mat::zeros(S.height *HH*SCALE,S.width, CV_8UC1);
            int rowIndex = 0;
            int Interval = 0;
            int IntervalTmp = 0;
            vector<point3D_t> gpsPointCar, gpsPointL, gpsPointR;
            vector<gpsCarInfo_t> gpsCarInfoVec;

            Mat roadT1, roadDraw3;

            imageInfo_t *image;

            vector<int> lineIdxVec;
            vector<double> leftWidthVec;
            vector<double> rightWidthVec;
            vector<point3D_t> gpsVec;
        
            double middlePixel;
            Point2d GPS_abs, GPS_next;

            int intrtmp = 0;

            for(int index = 0; index < number; index++)
            {
                buffer->getCurrentImage(&image);
                middlePixel = (image->image.cols) / 2;

                //Linedetector.line_Detection(image->image, LaneMarkKF, KF, measurement, S, measLandMark, &lineIndex, &leftWidth, &rightWidth);
            
                GPS_abs.x = image->gpsInfoPre.lat;
                GPS_abs.y = image->gpsInfoPre.lon;
                GPS_next.x = image->gpsInfo.lat;
                GPS_next.y = image->gpsInfo.lon;

                roadImageGen(image->image, history, &rowIndex, &GPS_abs, &GPS_next, &gpsAndInterval, &intrtmp, inParam);
                
                if (gpsAndInterval.intervalOfInterception)
                {
                    GPSAndInterval.push_back(gpsAndInterval);
                }

                gpsCarInfo_t gpsCarInfo;
                gpsCarInfo.laneId = lineIndex;
                gpsCarInfo.lineStyle = 0;
                gpsCarInfo.laneWidth = 0;
                gpsCarInfo.gpsInfo = image->gpsInfo;
                gpsCarInfo.gpsInfoPre = image->gpsInfoPre;
            
                gpsCarInfoVec.push_back(gpsCarInfo);

                lineIdxVec.push_back(lineIndex);
                leftWidthVec.push_back(leftWidth);
                rightWidthVec.push_back(rightWidth);
                gpsVec.push_back(image->gpsInfo);
            }

            char bufferTemp[32];                    
            sprintf(bufferTemp, "road_time.png");
            imwrite(bufferTemp, history );

            rowIndex = rowIndex - GPSAndInterval[GPSAndInterval.size()-1].intervalOfInterception;
            Mat historyROI = history(Rect(0,rowIndex,history.cols,history.rows-rowIndex));

            sprintf(bufferTemp, "road_time_roi.png");
            imwrite(bufferTemp, historyROI );

            // smoothLaneId
            smoothLaneId(lineIdxVec);
        
            roadImageProc2(historyROI, GPSAndInterval, roadPaintData, inParam);

            for(int index = 0; index < roadPaintData.size(); index++)
            {
                if( (0 != roadPaintData[index].Left_Middle_RelGPS.x) && (0 != roadPaintData[index].Left_Middle_RelGPS.y) && 
                    (0 != roadPaintData[index].Right_Middle_RelGPS.x) && (0 != roadPaintData[index].Right_Middle_RelGPS.y) )
                {
                    point3D_t outGpsInfoL, outGpsInfoR;
                    outGpsInfoL.alt = 0; outGpsInfoL.lat = roadPaintData[index].Left_Middle_RelGPS.x; outGpsInfoL.lon = roadPaintData[index].Left_Middle_RelGPS.y;
                    outGpsInfoR.alt = 0; outGpsInfoR.lat = roadPaintData[index].Right_Middle_RelGPS.x; outGpsInfoR.lon = roadPaintData[index].Right_Middle_RelGPS.y;
                    laneType_t laneInfo;
                    laneInfo.gpsL = outGpsInfoL;
                    laneInfo.gpsR = outGpsInfoR;
                    laneInfo.laneId = 0;
                    laneInfo.laneWidth = 3.75;
                    laneInfo.lineStyle = 0;
                    laneInfo.laneChangeFlag = 0;
                    laneInfo.linePaintFlagL = roadPaintData[index].isPaint_Left;
                    laneInfo.linePaintFlagR = roadPaintData[index].isPaint_Right;
                    laneInfoList.push_back(laneInfo);
                }
            }

            roadPaintData.clear();
            GPSAndInterval.clear();
        }

        // Furniture detection
        list<furAttributesInVehicle_t> furInfoListAddReport;
        list<furAttributesInVehicle_t> furInfoListUpdateReport;
        {
            //Detector_colored *detector = new Detector_colored();
            Detector_blackWhite *detector = new Detector_blackWhite();
            imageInfo_t *image;

            list<statisticsFurInfo_t> furnListInBuff;

            for(int index = 0; index < number; index++)
            {
                TS_Structure detectedTrafficSign;
                memset(&detectedTrafficSign,0,sizeof(detectedTrafficSign));

                buffer->getCurrentImage(&image);

                detector->trafficSignDetect(image->image, detectedTrafficSign);

                if(detectedTrafficSign.totalNumber > 0)
                {
                    //cout << "detected sign number: "<< detectedTrafficSign.totalNumber << endl;

                    list<furAttributesInVehicle_t> furInfoListInDet;
                    list<furAttributesInVehicle_t> furListTemp;

                    point3D_t refGps;
                    refGps.lon = inParam.GPSref.y;
                    refGps.lat = inParam.GPSref.x;
                    refGps.alt = 0;
                    convertImageToFurniture(&detectedTrafficSign,&image->gpsInfo,&image->gpsInfoPre,&refGps,&furInfoListInDet);

                    //filter the image
                    filterFurToReport(image->gpsInfo,&furnListInBuff, &furInfoListInDet,&furListTemp);

                    if( furListTemp.size() > 0)
                    {
                        list<furAttributesInVehicle_t>::iterator furListTempIdx;
                        for(furListTempIdx = furListTemp.begin();furListTempIdx != furListTemp.end(); ++furListTempIdx)
                        {
                            list<furAttributesInVehicle_t> furInfoListInDb;

                            //get the furniture info according to the GPS.
                            database_gp->getFurnitureByGps(&(furListTempIdx->location), database_gp->_distThreshFar, furInfoListInDb);

                            compareFurnitureList(&(*furListTempIdx),&furInfoListInDb,&furInfoListAddReport, &furInfoListUpdateReport);
                        }
                    }
                }
            }

            delete detector;
        }

        // Generate message
        {
            messageProcessClass    diffMsg;
            diffRptMsg_t* msgHeaderPtr = diffMsg.getDiffRptMsg();
            msgHeaderPtr->payload = new uint8[MAX_PAYLOAD_BYTE_NUM];
            void* payloadPtr = (void*)(msgHeaderPtr->payload);
            
            if(msgHeaderPtr->payload == NULL)
            {
                continue;
            }

            int numRoadGeoPdu;
            if(laneInfoList.size() > 0)
            {
                numRoadGeoPdu = 1;
            }else
            {
                numRoadGeoPdu = 0;
            }

            // fill the message header
            msgHeaderPtr->msgHeader.msgTypeID = DIFF_RPT_MSG;
            msgHeaderPtr->msgHeader.numPDUs = numRoadGeoPdu + furInfoListUpdateReport.size() + furInfoListAddReport.size();
            msgHeaderPtr->msgHeader.priority = middLevel_e;
            msgHeaderPtr->msgHeader.vehicleID = g_VehicleID;
            msgHeaderPtr->msgHeader.headerLen = sizeof(msgHeaderPtr->msgHeader) + msgHeaderPtr->msgHeader.numPDUs * sizeof(msgHeaderPtr->payloadHeader.pduHeader[0]);

            // Traffic signs
            int32 payloadSize = 0;
            int pduIdx = 0;
            int32 payloadLen = 0;
            int pduoffset = 0;

            list<furAttributesInVehicle_t>::iterator furIndex;
            furIndex = furInfoListAddReport.begin();

            //for(furIndex = furInfoListReport.begin(); furIndex != furInfoListReport.end()-delNumber;furIndex++)
            // detect a new furniture or update a furniture
            //std::cout << "add new furniture number:" << addNumber<< std::endl;
            while(furIndex != furInfoListAddReport.end())
            {
                diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,addDatabase_e,payloadSize);
                database_gp->convFurnitureToTlv(&(*furIndex),ns_database::memory_e,&payloadPtr,&payloadLen);
                
                msgHeaderPtr->payloadHeader.pduHeader[pduIdx].operate = addDatabase_e;
                msgHeaderPtr->payloadHeader.pduHeader[pduIdx].pduType = furnitureElement_e;
                msgHeaderPtr->payloadHeader.pduHeader[pduIdx].pduOffset = pduoffset;

                pduoffset += payloadLen; 
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

                msgHeaderPtr->payloadHeader.pduHeader[pduIdx].operate = updateDatabase_e;
                msgHeaderPtr->payloadHeader.pduHeader[pduIdx].pduType = furnitureElement_e;
                msgHeaderPtr->payloadHeader.pduHeader[pduIdx].pduOffset = pduoffset;

                pduoffset += payloadLen; 
                payloadSize += payloadLen;

                char furTypeString[100];
                sprintf(furTypeString,"Furniture \"%s\" updated",ns_database::ID2Name((*furIndex).type).c_str());
                logPrintf(logLevelInfo_e, "DIFF_DETECT", furTypeString,FOREGROUND_GREEN);

                furIndex++;
                pduIdx++;
            }

            // Road geometry
            {
                database_gp->getLaneGpsTlv(&laneInfoList,memory_e,(void**)&payloadPtr,&payloadLen);
                payloadSize += payloadLen;
                msgHeaderPtr->msgHeader.payloadLen = payloadSize;

                // fill the pdu header
                for(int idx = 0; idx < numRoadGeoPdu; ++idx)
                {
                    msgHeaderPtr->payloadHeader.pduHeader[pduIdx].operate = addDatabase_e;
                    msgHeaderPtr->payloadHeader.pduHeader[pduIdx].pduType = gpsInfo_e;
                    msgHeaderPtr->payloadHeader.pduHeader[pduIdx].pduOffset = pduoffset;
                    
                    pduoffset += payloadLen;
                    pduIdx++;
                }
            }

            // Send message
            {
                //char *sendBuff = new char[msgHeaderPtr->msgHeader.headerLen];
                //memcpy((void*)sendBuff,(void*)msgHeaderPtr,msgHeaderPtr->msgHeader.headerLen);

                int nRet = send(sockClient,(char*)msgHeaderPtr,msgHeaderPtr->msgHeader.headerLen,0);//,(SOCKADDR*)&serverAddr,g_SocketLen);//send response message data
                if ((nRet == SOCKET_ERROR) || (nRet == 0))
                {
                    int errorCode = WSAGetLastError();
                    trySetConnectSocket(true);
                    logPrintf(logLevelInfo_e, "DIFF_DETECT", "Send message header to server failed!");
                
                    delete msgHeaderPtr->payload;
                    msgHeaderPtr->payload = NULL;
                }
                else
                {
                    logPrintf(logLevelInfo_e, "DIFF_DETECT", "<<<< Send message header to server OK");

                    if((msgHeaderPtr->payload != NULL) && (msgHeaderPtr->msgHeader.payloadLen > 0))
                    {
                        int nRet = send(sockClient,(char*)msgHeaderPtr->payload,msgHeaderPtr->msgHeader.payloadLen,0);
                        if ((nRet == SOCKET_ERROR) || (nRet == 0))
                        {
                            logPrintf(logLevelInfo_e, "DIFF_DETECT", "Send message payload to server failed!");

                            delete msgHeaderPtr->payload;
                            msgHeaderPtr->payload = NULL;
                        }
                        else
                        {
                            logPrintf(logLevelInfo_e, "DIFF_DETECT", "<<<< Send message payload to server OK");
                            if(nRet >= msgHeaderPtr->msgHeader.payloadLen)
                            {
                                delete msgHeaderPtr->payload;
                                msgHeaderPtr->payload = NULL;
                            }
                        }
                    }
                }
            }
            laneInfoList.clear();
        }

        //ReleaseSemaphore(g_readySema_DiffDet, 1 ,NULL);
    }//end while(1)
    return 0;
}

void convertImageToFurniture(TS_Structure* targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,point3D_t* gpsRef,list<furAttributesInVehicle_t>* furnListPtr)
{
    int idx;

    const int SCREEN_WIDTH_LEFT_THRESHOLD = (IMAGE_SENSOR_WIDTH/2);//(IMAGE_SENSOR_WIDTH/4);
    const int SCREEN_WIDTH_RIGHT_THRESHOLD = (IMAGE_SENSOR_WIDTH/2);//(3*IMAGE_SENSOR_WIDTH/4);

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
            if(x_centre < IMAGE_SENSOR_WIDTH/2)
            {  side = 1;}//left
            else
            {  side = 2;}
            if(x_centreNext < IMAGE_SENSOR_WIDTH/2)
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
            if(x_centre < (IMAGE_SENSOR_WIDTH/2.0))
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
            calcGpsRoadSide(gpsInfoPrevP, gpsInfoPtr, gpsRef, side, 5, &(furniture.location));

            //compute the height
            furniture.location.alt = (IMAGE_SENSOR_HEIGHT - (targetPtr->TS_rect[idx].y + targetPtr->TS_rect[idx].height/2))/IMAGE_SENSOR_HEIGHT;

            furnListPtr->push_back(furniture);
        }
    }
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
        furDetIn->segId_used = 1;
        furDetIn->segId = 0;
        furDetIn->furId_used = 0;

        listAddOut->push_back(*furDetIn);
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
            if(!checkGpsInRange(&currentGps,&(furnListInBuffIdx->firstGps),database_gp->_distThreshNear))
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
