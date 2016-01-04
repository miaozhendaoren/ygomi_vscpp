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
*      2015/05/11         Qin Shi         Add message process
*******************************************************************************
*/
#include <stdlib.h>
#include <time.h>
#include <opencv2\core\core.hpp>
#include <opencv2\core\operations.hpp>
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
#include "configure.h"
#include "utils.h" // coordinateChange

#include <iomanip>
#include <fstream>
#include "TimeStamp.h"
#include <math.h>

using namespace ns_database;
using namespace ns_statistics;
using namespace std;
using cv::Mat;
using namespace ns_detection;
using namespace ns_roadScan;

double roadLen = 3.75;//m
double roadPixelNum = 400;

void compareFurnitureList(furAttributesInVehicle_t *furDetIn,list<furAttributesInVehicle_t>* list2, list<furAttributesInVehicle_t>* listAddOut, list<furAttributesInVehicle_t>* listUpdateOut);
void convertImageToFurniture(Size imageSize, ns_detection::TS_Structure &targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,point3D_t* refGps, int paramIdx, list<furWithPosition_t>* furnListPtr);
void filterFurToReport(Point2d refGps,point3D_t currentGps,list<statisticsFurInfo_t>* furnListInBuffPtr, list<furWithPosition_t>* furnListInDetPtr,list<furAttributesInVehicle_t>* outListPtr);
void checkFurWhenBufferEnd(list<statisticsFurInfo_t> *furnListInBuffPtr,list<furAttributesInVehicle_t> *outListPtr);

struct gpsCarInfo_t
{
    int laneId;
    int lineStyle;
    double laneWidth;//width;

    point3D_t gpsInfo;
    point3D_t gpsInfoPre;
};

#ifdef TRAFFIC_SIGN_TEST
FILE *fd;
#endif

void convertLandMark2furniture(landMark &mark, furAttributesInVehicle_t &furniture)
{
	//furniture.angle = landMark.angleVec;
	furniture.angle_used = 1;
	furniture.angle = (float)atan2(mark.angleVec.x, mark.angleVec.y);

	furniture.location_used = 1;
	furniture.location.alt = 0;
	furniture.location.lat = mark.centerRel.x;
	furniture.location.lon = mark.centerRel.y;

	furniture.type_used = 1;
	furniture.type      = mark.type;
	
	furniture.sideFlag_used = 1;
	furniture.sideFlag = 4;
}

unsigned int __stdcall Thread_DiffDetRpt(void *data)
{
    //open the camera device.
    int nRet;
    int sendLen;
	
	Mat history;
#if(RD_MODE != RD_VIDEO_LOAD_MODE)
	// use inParamVec[0] to allocate the memory
	history = Mat::zeros(IMAGE_BUFFER_DEPTH*HH,inParamVec[0].imageCols * inParamVec[0].imageScaleWidth, CV_8UC1);
#endif

    while(1)
    {
        ImageBuffer *buffer;
		vector<dataEveryRow> roadPaintData1;
		vector<dataEveryRow> roadPaintData2;
	
		vector<gpsInformationAndInterval> GPSAndInterval1;
		vector<gpsInformationAndInterval> GPSAndInterval2;
		Mat DrawMarker;
		Mat historyROI1;
		vector<landMark> vecLandMark;

		RD_ADD_TS(tsFunId_eThread_DifRpt,1);
        if(!imageBuffer.getBuffer(&buffer))
        {
            Sleep(500);
            continue;
        }
		RD_ADD_TS(tsFunId_eThread_DifRpt,2);
        int number = buffer->getImageNumber();
        int inParamIdx = buffer->getInParamIdx();

		// roadscan and furniture report buffer
		list<laneType_t> curLaneInfoList;
		list<laneType_t> sideLaneInfoList;
        list<furAttributesInVehicle_t> furInfoListAddReport;
        list<furAttributesInVehicle_t> furInfoListUpdateReport;
        // Painting detection

		// checke history buffer
		int imageWidth, imageHeight;
        buffer->getImageSize(imageWidth, imageHeight);
		int imageWidthTemp = imageWidth * inParamVec[inParamIdx].imageScaleWidth; // change from inParam to inParamVec[0], to use the first inParam value
		int imageHeightTemp = imageHeight * inParamVec[inParamIdx].imageScaleHeight;
		int inti_width = imageWidth * inParamVec[inParamIdx].imageScaleWidth; 
#if(RD_MODE == RD_VIDEO_LOAD_MODE)
		// use inParamVec[0] to allocate the memory
		history = Mat::zeros(number*HH,inti_width, CV_8UC1);
#else
		// change the memory size if the image columns been changed
		if (inParamVec[0].imageCols * inParamVec[0].imageScaleWidth != imageWidthTemp)
		{ resize(history,history,Size(history.rows,imageWidthTemp));}
#endif

#if(RD_ROAD_DETECT == ON)
        {
			logPrintf(logLevelInfo_e, "DIFF_DETECT", "RoadScan Start",FOREGROUND_BLUE);
            // start initialization
            gpsInformationAndInterval gpsAndInterval;

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
            
			
			Size S = Size(imageWidthTemp,imageHeightTemp);
            Linedetector.initialVanishPointKF(KF,S);

            // For Lane Marker
            KalmanFilter LaneMarkKF(2, 2, 0);
            Mat_<float> measLandMark(2,1); 
            measLandMark.setTo(Scalar(0));
            Linedetector.iniLaneMarkKF(LaneMarkKF,S);
            // end initialization

            int rowIndex = history.rows;

            vector<point3D_t> gpsPointCar, gpsPointL, gpsPointR;
            vector<gpsCarInfo_t> gpsCarInfoVec;

            Mat roadT1, roadDraw3;

            imageInfo_t image;

            vector<int> lineIdxVec;
            vector<double> leftWidthVec;
            vector<double> rightWidthVec;
            vector<point3D_t> gpsVec;
        
            double middlePixel;
            Point2d GPS_abs, GPS_next;
            Point2d GPS_stop = Point2d(0, 0);
            bool stopFlg = false;

            int intrtmp = 0;

			RD_ADD_TS(tsFunId_eThread_DifRpt,3);
			
            for(int index = 0; index < number; index++)
            {
                if(buffer->getCurrentImage(&image) == false)
                {
                    // get image from buffer failed
                    break;
                }
                middlePixel = (image.image.cols) / 2;

                //Linedetector.line_Detection(image->image, LaneMarkKF, KF, measurement, S, measLandMark, &lineIndex, &leftWidth, &rightWidth);
            
                GPS_abs.x = image.gpsInfoPre.lat;
                GPS_abs.y = image.gpsInfoPre.lon;
                GPS_next.x = image.gpsInfo.lat;
                GPS_next.y = image.gpsInfo.lon;

                // scan the current lane 
                int flag1 = roadImageGen(image.image, history, &rowIndex, 
                                        &GPS_abs, &GPS_next,HVec[inParamIdx], 
                                        inParamVec[inParamIdx].distancePerPixel,
                                        inParamVec[inParamIdx].startRowCurLane,  
                                        &gpsAndInterval, &intrtmp, 
                                        inParamVec[inParamIdx], GPS_stop, stopFlg);

                // store the current GPS information 
				if(flag1 == -1) 
				{
					printf("-----------goto 1-----------\n");
					goto CLEAN_ROADSCAN_BUFFER1;
				}

                if (gpsAndInterval.intervalOfInterception)
                {
                    GPSAndInterval1.push_back(gpsAndInterval);
                }
            }
			RD_ADD_TS(tsFunId_eThread_DifRpt,4);
            char bufferTemp[32];                    
            sprintf(bufferTemp, "road_time.png");
            imwrite(bufferTemp, history );

			if(!GPSAndInterval1.empty())
			{
				rowIndex = rowIndex - GPSAndInterval1[GPSAndInterval1.size()-1].intervalOfInterception;
				if((rowIndex < 0)||((history.rows-rowIndex)<=0))
				{
					printf("----------error: roindex = %d, historyRow = %d-----------\n",rowIndex,history.rows);
				}
				historyROI1 = history(Rect(0,rowIndex,history.cols,history.rows-rowIndex));

                DrawMarker = Mat::zeros(historyROI1.rows, historyROI1.cols, CV_8UC3);

				sprintf(bufferTemp, "road_time_roi.png");
				imwrite(bufferTemp, historyROI1 );
#if defined(_FRANKFORT_ALL_CAMERA)
#else
				roadImageProc2(historyROI1, inParamVec[inParamIdx], GPSAndInterval1, roadPaintData1, vecLandMark, DrawMarker);
#endif

				RD_ADD_TS(tsFunId_eThread_DifRpt,5);
				for(int index = 0; index < roadPaintData1.size(); index++)
				{
					if( (0 != roadPaintData1[index].Left_Middle_RelGPS.x) && (0 != roadPaintData1[index].Left_Middle_RelGPS.y) && 
						(0 != roadPaintData1[index].Right_Middle_RelGPS.x) && (0 != roadPaintData1[index].Right_Middle_RelGPS.y) )
					{
						point3D_t outGpsInfoL = { 0 }, outGpsInfoR = { 0 }, outGpsInfo= { 0 };
						outGpsInfoL.lat = roadPaintData1[index].Left_Middle_RelGPS.x; outGpsInfoL.lon = roadPaintData1[index].Left_Middle_RelGPS.y;
						outGpsInfoR.lat = roadPaintData1[index].Right_Middle_RelGPS.x; outGpsInfoR.lon = roadPaintData1[index].Right_Middle_RelGPS.y;
                        outGpsInfo.lat = roadPaintData1[index].Middle_RelGPS.x; outGpsInfo.lon = roadPaintData1[index].Middle_RelGPS.y;
						laneType_t laneInfo;
						laneInfo.gpsL = outGpsInfoL;
						laneInfo.gpsR = outGpsInfoR;
                        laneInfo.gpsTrack = outGpsInfo;
						laneInfo.laneId = 0;
						laneInfo.laneWidth = 3.75;
						laneInfo.lineStyle = 0;
						laneInfo.laneChangeFlag = 0;
						laneInfo.linePaintFlagL = roadPaintData1[index].isPaint_Left;
						laneInfo.linePaintFlagR = roadPaintData1[index].isPaint_Right;
						curLaneInfoList.push_back(laneInfo);
					}
				}
			}
            roadPaintData1.clear();
        }
        RD_ADD_TS(tsFunId_eThread_DifRpt,6);
#if ((RD_SIDEWARD_LANE_DETECT == ON) && (RD_ROAD_DETECT == ON))
     {
            // start initialization
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
            
			Size S = Size(imageWidthTemp,imageHeightTemp);
            Linedetector.initialVanishPointKF(KF,S);

            // For Lane Marker
            KalmanFilter LaneMarkKF(2, 2, 0);
            Mat_<float> measLandMark(2,1); 
            measLandMark.setTo(Scalar(0));
            Linedetector.iniLaneMarkKF(LaneMarkKF,S);
            // end initialization

            


            vector<point3D_t> gpsPointCar, gpsPointL, gpsPointR;
            vector<gpsCarInfo_t> gpsCarInfoVec;

            Mat roadT1, roadDraw3;

            imageInfo_t image;

            vector<int> lineIdxVec;
            vector<double> leftWidthVec;
            vector<double> rightWidthVec;
            vector<point3D_t> gpsVec;
        
            double middlePixel;
            Point2d GPS_abs, GPS_next;
            Point2d GPS_stop = Point2d(0, 0);
            bool stopFlg = false;
            int intrtmp = 0;
			int rowIndex = history.rows;
			// clear the history buffer
			//memset(history.data,0,history.rows*history.cols*history.channels()*history.step.buf[1]);

            // reset the image buffer
            buffer->setImageToStart();
            gpsInformationAndInterval gpsAndInterval;

            for(int index = 0; index < number; index++)
            {
                if(buffer->getCurrentImage(&image) == false)
                {
                    // get image from buffer failed
                    break;
                }
                middlePixel = (image.image.cols) / 2;
            
                GPS_abs.x = image.gpsInfoPre.lat;
                GPS_abs.y = image.gpsInfoPre.lon;
                GPS_next.x = image.gpsInfo.lat;
                GPS_next.y = image.gpsInfo.lon;
                // scan the left and right lane of the current lane
                int flag = roadImageGen(image.image, history, &rowIndex, 
                                        &GPS_abs, &GPS_next,laneHVec[inParamIdx], 
                                        inParamVec[inParamIdx].distancePerPixelY,
                                        inParamVec[inParamIdx].startRowAllLane, 
                                        &gpsAndInterval, 
                                        &intrtmp, inParamVec[inParamIdx], 
                                        GPS_stop,stopFlg);

				if(flag == -1)
				{
					printf("-----------goto 2-----------\n");
					goto CLEAN_ROADSCAN_BUFFER2;
				}

                if (gpsAndInterval.intervalOfInterception)
                {
                    GPSAndInterval2.push_back(gpsAndInterval);
                }
            }
            if(!GPSAndInterval2.empty())
			{
				rowIndex = rowIndex - GPSAndInterval2[GPSAndInterval2.size()-1].intervalOfInterception;
				Mat historyROI = history(Rect(0,rowIndex,history.cols,history.rows-rowIndex));

                char bufferTemp[32]; 
				sprintf(bufferTemp, "road_time_roi_sideward.png");
				imwrite(bufferTemp, historyROI);

                Mat outImage = Mat::zeros(historyROI.rows, historyROI.cols, CV_8UC3);
                float width;
                float meanX;

                // detect the sideward lanes of the current lane
                bool flag = currentLaneMatched(DrawMarker, historyROI1, historyROI, GPSAndInterval1, GPSAndInterval2, inParamVec[inParamIdx], outImage, &width, &meanX,invertHVec[inParamIdx], laneHVec[inParamIdx]);
                if (flag == true)
                { 
                    bool successFlag = neighborLaneDetect(outImage, width, meanX, GPSAndInterval2, inParamVec[inParamIdx], roadPaintData2);

                    if (successFlag == true)
					{
						for(int index = 0; index < roadPaintData2.size(); index++)
						{
							if( ((0 != roadPaintData2[index].Left_Middle_RelGPS.x) && (0 != roadPaintData2[index].Left_Middle_RelGPS.y)) 
								&& ((0 != roadPaintData2[index].Right_Middle_RelGPS.x) && (0 != roadPaintData2[index].Right_Middle_RelGPS.y)) )
							{
								point3D_t outGpsInfoL = { 0 }, outGpsInfoR = { 0 }, outGpsInfo= { 0 };
								outGpsInfoL.lat = roadPaintData2[index].Left_Middle_RelGPS.x; outGpsInfoL.lon = roadPaintData2[index].Left_Middle_RelGPS.y;
								outGpsInfoR.lat = roadPaintData2[index].Right_Middle_RelGPS.x; outGpsInfoR.lon = roadPaintData2[index].Right_Middle_RelGPS.y;
								laneType_t laneInfo;                  
								laneInfo.gpsL = outGpsInfoL;
								laneInfo.gpsR = outGpsInfoR;
								laneInfo.gpsTrack = outGpsInfo;
								laneInfo.laneId = 0;
								laneInfo.laneWidth = width*inParamVec[inParamIdx].distancePerPixelX*0.01;//unit:meter
								laneInfo.lineStyle = 0;
								laneInfo.laneChangeFlag = 0;
								laneInfo.linePaintFlagL = roadPaintData2[index].isPaint_Left;
								laneInfo.linePaintFlagR = roadPaintData2[index].isPaint_Right;
								sideLaneInfoList.push_back(laneInfo);
							}
						}
					}
                }
            }
            roadPaintData2.clear();
        }

CLEAN_ROADSCAN_BUFFER2:
		 GPSAndInterval2.clear();
#endif
CLEAN_ROADSCAN_BUFFER1:
		 GPSAndInterval1.clear();
		 logPrintf(logLevelInfo_e, "DIFF_DETECT", "RoadScan Stop",FOREGROUND_BLUE);
#endif

#if (RD_ROAD_SIGN_DETECT != RD_ROAD_SIGN_DETECT_OFF)
		//check if there is some traffic sign on the road scamed by roadscan
        {
            if( vecLandMark.size() > 0)
            {
                for(int roadSign = 0; roadSign < vecLandMark.size(); roadSign++)
                {
#ifdef TRAFFIC_SIGN_TEST                           
                    fprintf(fd,"====================================================================\n");	
                    fprintf(fd,"report_%d_%d = ",furListTempIdx->type,furListTempIdx->sideFlag);
                    fprintf(fd,"%.14f %.14f;\n",furListTempIdx->location.lon,furListTempIdx->location.lat);	
                    fprintf(fd,"***********************************************************************\n");
#endif
                    list<furAttributesInVehicle_t> furInfoListInDb;
					furAttributesInVehicle_t furTemp;

					convertLandMark2furniture(vecLandMark[roadSign], furTemp);
                    //get the furniture info according to the GPS.
					database_gp->getFurnitureByGps(&(furTemp.location), database_gp->_distThreshFar, furInfoListInDb);

                    compareFurnitureList(&furTemp,&furInfoListInDb,&furInfoListAddReport, &furInfoListUpdateReport);
                }

				vecLandMark.clear();
            }
        }
#endif

#if(RD_SIGN_DETECT != RD_SIGN_DETECT_OFF)
		logPrintf(logLevelInfo_e, "DIFF_DETECT", "Traffic Sign Detect Start",FOREGROUND_BLUE);
#ifdef TRAFFIC_SIGN_TEST
        fd = fopen("gpsInfo.txt","a");
        if (fd == NULL)
        {
            cout<< "open file failed!" << endl;
        }
#endif

		buffer->setImageToStart();
        {
            imageInfo_t image;

            list<statisticsFurInfo_t> furnListInBuff;
			RD_ADD_TS(tsFunId_eThread_DifRpt,7);
            for(int index = 0; index < number; index += 1) // +=2: skip one frame and process one frame
            {
                ns_detection::TS_Structure detectedTrafficSign;

                //buffer->getCurrentImage(&image); // skip one frame and process one frame
                if(buffer->getCurrentImage(&image) == false)
                {
                    // get image from buffer failed
                    break;
                }

#if (RD_LOCATION == RD_GERMAN_LEHRE)
                resize(image.image,image.image,Size(image.image.cols/2,image.image.rows/2));
#endif
                trafficSignDetectorVec[inParamIdx]->trafficSignDetect(image.image, detectedTrafficSign); // detect traffic signs
                {
                    //cout << "detected sign number: "<< detectedTrafficSign.totalNumber << endl;

                    list<furWithPosition_t> furInfoListInDet;
                    list<furAttributesInVehicle_t> furListTemp;

                    if(detectedTrafficSign.trafficSign.size() > 0)
                    {
                        point3D_t refGps;
                        refGps.lon = inParamVec[inParamIdx].GPSref.y;
                        refGps.lat = inParamVec[inParamIdx].GPSref.x;
                        refGps.alt = 0;

                        Point2d GPS_abs, GPS_next;
                        GPS_abs.x = image.gpsInfoPre.lat;
                        GPS_abs.y = image.gpsInfoPre.lon;
                        GPS_next.x = image.gpsInfo.lat;
                        GPS_next.y = image.gpsInfo.lon;

                        trafficSignDetectorVec[inParamIdx]->positionMeasure(inParamVec[inParamIdx], GPS_abs, GPS_next, image.image, detectedTrafficSign, invertHVec[inParamIdx]);
                        convertImageToFurniture(image.image.size(), detectedTrafficSign,&image.gpsInfo,&image.gpsInfoPre,&refGps, inParamIdx, &furInfoListInDet);

#ifdef TRAFFIC_SIGN_TEST

                        list<furWithPosition_t>::iterator testIdx = furInfoListInDet.begin();

                        while(testIdx != furInfoListInDet.end())
                        {
                            if(testIdx->furAttri.type != 0)
                            {
                                fprintf(fd,"A_%d_%d  = [",testIdx->furAttri.type,testIdx->furAttri.sideFlag);	
                                for(int idx = 0;idx < testIdx->position.size();++idx)
                                {
                                    fprintf(fd,"%.14f %.14f;",testIdx->position[idx].lon,testIdx->position[idx].lat);	
                                }
                                 fprintf(fd,"]; \n");	
                            }
                            ++testIdx;
                        }
#endif
                    }
                    
                    //filter the image
                    filterFurToReport(inParamVec[inParamIdx].GPSref,image.gpsInfo,&furnListInBuff, &furInfoListInDet,&furListTemp);

                    if( furListTemp.size() > 0)
                    {
                        list<furAttributesInVehicle_t>::iterator furListTempIdx;
                        for(furListTempIdx = furListTemp.begin();furListTempIdx != furListTemp.end(); ++furListTempIdx)
                        {
#ifdef TRAFFIC_SIGN_TEST                           
                            fprintf(fd,"====================================================================\n");	
                            fprintf(fd,"report_%d_%d = ",furListTempIdx->type,furListTempIdx->sideFlag);
                            fprintf(fd,"%.14f %.14f;\n",furListTempIdx->location.lon,furListTempIdx->location.lat);	
                            fprintf(fd,"***********************************************************************\n");
#endif
                            list<furAttributesInVehicle_t> furInfoListInDb;

                            //get the furniture info according to the GPS.
                            database_gp->getFurnitureByGps(&(furListTempIdx->location), database_gp->_distThreshFar, furInfoListInDb);

                            compareFurnitureList(&(*furListTempIdx),&furInfoListInDb,&furInfoListAddReport, &furInfoListUpdateReport);
                        }
                    }
                }
            }

            // check if any fur need to be reported at the end of buffer
            {
                list<furAttributesInVehicle_t> furListTemp;

                checkFurWhenBufferEnd(&furnListInBuff,&furListTemp);

                if( furListTemp.size() > 0)
                {
                    list<furAttributesInVehicle_t>::iterator furListTempIdx;
                    for(furListTempIdx = furListTemp.begin();furListTempIdx != furListTemp.end(); ++furListTempIdx)
                    {
#ifdef TRAFFIC_SIGN_TEST                           
                        fprintf(fd,"====================================================================\n");	
                        fprintf(fd,"report_%d_%d = ",furListTempIdx->type,furListTempIdx->sideFlag);
                        fprintf(fd,"%.14f %.14f;\n",furListTempIdx->location.lon,furListTempIdx->location.lat);	
                        fprintf(fd,"***********************************************************************\n");
#endif
                        list<furAttributesInVehicle_t> furInfoListInDb;

                        //get the furniture info according to the GPS.
                        database_gp->getFurnitureByGps(&(furListTempIdx->location), database_gp->_distThreshFar, furInfoListInDb);

                        compareFurnitureList(&(*furListTempIdx),&furInfoListInDb,&furInfoListAddReport, &furInfoListUpdateReport);
                    }
                }
            }

        }

#ifdef TRAFFIC_SIGN_TEST
        fclose(fd);
#endif
		logPrintf(logLevelInfo_e, "DIFF_DETECT", "Traffic Sign Detect Stop",FOREGROUND_BLUE);
#endif//(RD_SIGN_DETECT != RD_SIGN_DETECT_OFF)
		RD_ADD_TS(tsFunId_eThread_DifRpt,8);
        // Generate message
        {
            messageProcessClass    diffMsg;
            diffRptMsg_t* msgHeaderPtr = diffMsg.getDiffRptMsg();
            msgHeaderPtr->payload = new uint8[MAX_ROAD_POINT_BYTES];
            void* payloadPtr = (void*)(msgHeaderPtr->payload);
            
            if(msgHeaderPtr->payload == NULL)
            {
                continue;
            }

            int numCurRoadPdu = 0,numSideRoadPdu = 0;
            if(curLaneInfoList.size() > 0)
            {
                numCurRoadPdu = 1;
            }else
			{
				numCurRoadPdu = 0;
			}

			if(sideLaneInfoList.size() > 0)
            {
                numSideRoadPdu = 1;
            }else
            {
                numSideRoadPdu = 0;
            }

            // fill the message header
            msgHeaderPtr->msgHeader.msgTypeID = DIFF_RPT_MSG;
            msgHeaderPtr->msgHeader.numPDUs = numSideRoadPdu + numCurRoadPdu + furInfoListUpdateReport.size() + furInfoListAddReport.size();
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
                database_gp->convFurnitureToTlv(&(*furIndex),ns_database::memory_e,&payloadPtr,&payloadLen);
                
                diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,addDatabase_e,pduoffset);

                pduoffset += payloadLen; 
                payloadSize += payloadLen;

                char furTypeString[100];
                sprintf(furTypeString,"Furniture \"%s\" detected",ns_database::ID2Name((*furIndex).type).c_str());
                logPrintf(logLevelInfo_e, "DIFF_DETECT", furTypeString,FOREGROUND_GREEN);

                furIndex++;
                pduIdx++;
            }
			RD_ADD_TS(tsFunId_eThread_DifRpt,9);
            // report the update furnitures
            furIndex = furInfoListUpdateReport.begin();

            while(furIndex != furInfoListUpdateReport.end())
            {
                database_gp->convFurnitureToTlv(&(*furIndex),ns_database::memory_e,&payloadPtr,&payloadLen);

                diffMsg.setDiffRptPduMsgHeader(pduIdx,furnitureElement_e,updateDatabase_e,pduoffset);

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
#ifdef ROAD_SCAN_UT
                static int id = 0;
                if(sideLaneInfoList.size() > 0)
                {
                    FILE *fd;
                    stringstream index;
                    string temp;
                    index << id;
                    id++;
                    index >> temp;
                    string imageName = "lanesData" + temp + ".m";
                    errno_t err = fopen_s(&fd,imageName.c_str(),"w");
                    if(0 != err)
                    {
	                    fclose(fd);
                        return false;
                    }
                    list<laneType_t>::iterator laneIdx = curLaneInfoList.begin();
                    fprintf(fd,"curLane.leftLine = [");
                    for(int ii = 0; ii < curLaneInfoList.size(); ii++)
                    {
                        fprintf(fd,"%.14f %.14f; ",laneIdx->gpsL.lat,laneIdx->gpsL.lon);
                        laneIdx++;
                    }
                    fprintf(fd,"];\n");
                    laneIdx = curLaneInfoList.begin();
                    fprintf(fd,"curLane.rightLine = [");
                    for(int ii = 0; ii < curLaneInfoList.size(); ii++)
                    {
                        fprintf(fd,"%.14f %.14f; ",laneIdx->gpsR.lat,laneIdx->gpsR.lon);
                        laneIdx++;
                    }
                    fprintf(fd,"];\n");

                    laneIdx = sideLaneInfoList.begin();
                    fprintf(fd,"sideLane.leftLine = [");
                    for(int ii = 0; ii < sideLaneInfoList.size(); ii++)
                    {
                        fprintf(fd,"%.14f %.14f; ",laneIdx->gpsL.lat,laneIdx->gpsL.lon);
                        laneIdx++;

                    }
                    fprintf(fd,"];\n");
                    laneIdx = sideLaneInfoList.begin();
                    fprintf(fd,"sideLane.rightLine = [");
                    for(int ii = 0; ii < sideLaneInfoList.size(); ii++)
                    {

                        fprintf(fd,"%.14f %.14f; ",laneIdx->gpsR.lat,laneIdx->gpsR.lon);
                        laneIdx++;
                    }
                    fprintf(fd,"];\n");
                    fprintf(fd,"figure;\n");
                    fprintf(fd,"plot(curLane.leftLine(:,2),curLane.leftLine(:,1),'r');\n");
                    fprintf(fd,"hold on;\n");
                    fprintf(fd,"plot(curLane.rightLine(:,2),curLane.rightLine(:,1),'b');\n");
                    fprintf(fd,"hold on;\n");
                    fprintf(fd,"plot(sideLane.leftLine(:,2),sideLane.leftLine(:,1),'m');\n");
                    fprintf(fd,"hold on;\n");
                    fprintf(fd,"plot(sideLane.rightLine(:,2),sideLane.rightLine(:,1),'g');\n");
                    fprintf(fd,"hold off;\n");
                    fclose(fd);
                }
                
#endif

                // current lane information
                database_gp->getLaneGpsTlv(&curLaneInfoList,memory_e,(void**)&payloadPtr,&payloadLen);
                payloadSize += payloadLen;
                msgHeaderPtr->msgHeader.payloadLen = payloadSize;

                // fill the pdu header
                for(int idx = 0; idx < numCurRoadPdu; ++idx)
                {
                    diffMsg.setDiffRptPduMsgHeader(pduIdx,curLaneInfo_e,addDatabase_e,pduoffset);
                    
                    pduoffset += payloadLen;
                    pduIdx++;
                }

                // sideward lane information
                database_gp->getLaneGpsTlv(&sideLaneInfoList,memory_e,(void**)&payloadPtr,&payloadLen);
                payloadSize += payloadLen;
                msgHeaderPtr->msgHeader.payloadLen = payloadSize;

                // fill the pdu header
                for(int idx = 0; idx < numSideRoadPdu; ++idx)
                {
                    diffMsg.setDiffRptPduMsgHeader(pduIdx,sideLaneInfo_e,addDatabase_e,pduoffset);
                    
                    pduoffset += payloadLen;
                    pduIdx++;
                }

            }

			RD_ADD_TS(tsFunId_eThread_DifRpt,10);
            // Send message
            if(pduIdx > 0)
            {
                //char *sendBuff = new char[msgHeaderPtr->msgHeader.headerLen];
                //memcpy((void*)sendBuff,(void*)msgHeaderPtr,msgHeaderPtr->msgHeader.headerLen);
#if(ON == RD_REPORT_RESULT)
                int nRet = send(sockClient,(char*)msgHeaderPtr,msgHeaderPtr->msgHeader.headerLen,0);//,(SOCKADDR*)&serverAddr,g_SocketLen);//send response message data
#else
				int nRet = 1;
#endif
				if ((nRet == SOCKET_ERROR) || (nRet == 0))
                {
                    int errorCode = WSAGetLastError();
                    trySetConnectSocket(true);
                    logPrintf(logLevelInfo_e, "DIFF_DETECT", "Send message header to server failed!",FOREGROUND_RED);
                
                    delete msgHeaderPtr->payload;
                    msgHeaderPtr->payload = NULL;
                }
                else
                {
                    logPrintf(logLevelInfo_e, "DIFF_DETECT", "<<<< Send message header to server OK");

                    if((msgHeaderPtr->payload != NULL) && (msgHeaderPtr->msgHeader.payloadLen > 0))
                    {
#if (ON == RD_REPORT_RESULT)
                        int nRet = send(sockClient,(char*)msgHeaderPtr->payload,msgHeaderPtr->msgHeader.payloadLen,0);
#else
						int nRet = 1;
#endif
                        if ((nRet == SOCKET_ERROR) || (nRet == 0))
                        {
                            logPrintf(logLevelInfo_e, "DIFF_DETECT", "Send message payload to server failed!",FOREGROUND_RED);

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
            }else
            {
                delete msgHeaderPtr->payload;
            }
            curLaneInfoList.clear();
            sideLaneInfoList.clear();
        }
        //ReleaseSemaphore(g_readySema_DiffDet, 1 ,NULL);
		RD_ADD_TS(tsFunId_eThread_DifRpt,11);
    }//end while(1)

    return 0;
}

void convertImageToFurniture(Size imageSize, ns_detection::TS_Structure &targetPtr,point3D_t* gpsInfoPtr,point3D_t* gpsInfoPrevP,point3D_t* gpsRef, int paramIdx, list<furWithPosition_t>* furnListPtr)
{
    int idx;

    int SCREEN_WIDTH_LEFT_THRESHOLD = (imageSize.width/3);
    int SCREEN_WIDTH_RIGHT_THRESHOLD = (2*imageSize.width/3);

    ns_detection::TS_Structure tempTarget = targetPtr;

    //delete the same type in a same frame.
    int maxArea = tempTarget.trafficSign[0].area;

    for(idx = 0;idx < tempTarget.trafficSign.size() - 1; idx++)
    {
        if(tempTarget.trafficSign[idx].type == tempTarget.trafficSign[idx+1].type)
        {
            double x_centre = tempTarget.trafficSign[idx].rect.x + tempTarget.trafficSign[idx].rect.width/2.0;
            double x_centreNext = tempTarget.trafficSign[idx+1].rect.x + tempTarget.trafficSign[idx+1].rect.width/2.0;
            short side;
            short sideNext;
            if(x_centre < imageSize.width/2)
            {  side = 1;}//left
            else
            {  side = 2;}
            if(x_centreNext < imageSize.width/2)
            {  sideNext = 1;}//left
            else
            {  sideNext = 2;}
            if(side == sideNext)
            {
                if(maxArea > tempTarget.trafficSign[idx+1].area)
                { 
                    targetPtr.trafficSign[idx+1].type = 0;
                    targetPtr.trafficSign[idx+1].position.clear();
                }
                else
                {
                    targetPtr.trafficSign[idx].type = 0;
                    maxArea = targetPtr.trafficSign[idx+1].area;
                    targetPtr.trafficSign[idx].position.clear();
                }
            }
        }
        else
        {
            maxArea = tempTarget.trafficSign[idx+1].area;
        }
    }
    // delete the furniture in the middle of the screen

    for(idx = 0;idx < tempTarget.trafficSign.size(); idx++)
    {
        double x_centre = tempTarget.trafficSign[idx].rect.x + tempTarget.trafficSign[idx].rect.width/2.0;
        if ((x_centre > SCREEN_WIDTH_LEFT_THRESHOLD) && (x_centre < SCREEN_WIDTH_RIGHT_THRESHOLD))
        {
            targetPtr.trafficSign[idx].type = 0;
            targetPtr.trafficSign[idx].position.clear();
        }
    }

    for(idx = 0; idx < targetPtr.trafficSign.size(); idx++)
    {
        furWithPosition_t furniture;
        if( targetPtr.trafficSign[idx].type != 0)
        {
            uint8 side;
            furniture.furAttri.type_used = 1;
            furniture.furAttri.type = targetPtr.trafficSign[idx].type;

            furniture.furAttri.reliabRating_used = 1;
            furniture.furAttri.reliabRating = 1;    
            // compute angle
            furniture.furAttri.angle_used = 1;
            calcNormalAngle(gpsInfoPrevP,gpsInfoPtr, &(furniture.furAttri.angle));
            // compute location
            furniture.furAttri.location_used = 1;
            double x_centre = targetPtr.trafficSign[idx].rect.x + targetPtr.trafficSign[idx].rect.width/2.0;
            if(x_centre < (imageSize.width/2.0))
            { 
                side = 2;
                furniture.furAttri.sideFlag_used = 1;
                furniture.furAttri.sideFlag = 2;

            }
            else
            { 
                side = 1;
                furniture.furAttri.sideFlag_used = 1;
                furniture.furAttri.sideFlag = 1;
            }

            // get loop index from furniture in loop index
            uint8 loopIndex = getLoopIdxFromFurInLoopIdx(paramIdx);
            furniture.furAttri.inLoopIdx_used = 1;
            furniture.furAttri.inLoopIdx = loopIndex;
            //calcGpsRoadSide(gpsInfoPrevP, gpsInfoPtr, gpsRef, side, 5, &(furniture.location));

            // copy the latitude,longtitude and mutiple of sign's height

            vector<point3D_t> location = targetPtr.trafficSign[idx].position;
            vector<float> offset = targetPtr.trafficSign[idx].offset;

            furniture.position = (location);

            for(int locationIdx = 0; locationIdx < location.size(); ++locationIdx)
            {
                //pointRelative3D_t furRelLoc;
                //calcRelativeLocation(gpsRef, &location[locationIdx], &furRelLoc);
                //furniture.position[locationIdx].lon = furRelLoc.x;
                //furniture.position[locationIdx].lat = furRelLoc.y;

                //compute the height
                furniture.position[locationIdx].alt = ((double)(imageSize.height - (targetPtr.trafficSign[idx].rect.y + targetPtr.trafficSign[idx].rect.height/2)))/((double)imageSize.height);
            }

            furniture.offset = (offset);

            furniture.offsetNumPerFur  = targetPtr.trafficSign[idx].position.size();

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

void checkFurWhenBufferEnd(list<statisticsFurInfo_t> *furnListInBuffPtr,list<furAttributesInVehicle_t> *outListPtr)
{
    list<statisticsFurInfo_t>::iterator furnListInBuffIdx = furnListInBuffPtr->begin();
    while(furnListInBuffIdx != furnListInBuffPtr->end())
    {
        if(furnListInBuffIdx->position.size() > VAR_ACC_TIME)
        {
             Mat slopeMatrix(furnListInBuffIdx->position.size(),2,CV_64FC1);
             Mat constMatrix(furnListInBuffIdx->position.size(),1,CV_64FC1);
                        
             for(int stepIdx1 = 0;stepIdx1 < furnListInBuffIdx->position.size(); ++stepIdx1)
             {
                vector<point3D_t> positionVector = furnListInBuffIdx->position[stepIdx1];
                vector<Point2f> positionVec2d;
                // 3d point transform to 2d.
                for(int numIdx = 0; numIdx < positionVector.size();++numIdx)
                {
                    Point2f point;
                    point.x = positionVector[numIdx].lat;
                    point.y = positionVector[numIdx].lon;
                    positionVec2d.push_back(point);
                }
                // line fitting
                Vec4f     lineFit;
                fitLine(positionVec2d,lineFit,CV_DIST_L2,1,0.01,0.01);
                // Ax=b, construct the A matrix and b matrix
                slopeMatrix.at<double>(stepIdx1,0) = lineFit[1]/lineFit[0];
#ifdef TRAFFIC_SIGN_TEST
                fprintf(fd,"%d_%d_K = %.14f;\n",furnListInBuffIdx->furAttri.type,furnListInBuffIdx->furAttri.sideFlag,atan2(lineFit[1],lineFit[0]));
#endif
                slopeMatrix.at<double>(stepIdx1,1) = -1.0;
                constMatrix.at<double>(stepIdx1) = lineFit[1]/lineFit[0] * lineFit[2] - lineFit[3];
             }
             //delete the parallel lines
            bool parallelFlag = false;
            for(int rowIdx1 = 0;rowIdx1 < slopeMatrix.rows; ++rowIdx1)
            {
                int times = 0;
                double slope1 = atan(slopeMatrix.at<double>(rowIdx1,0))*180/PI;

                for(int rowIdx2 = 0; rowIdx2 < slopeMatrix.rows; ++rowIdx2)
                {                           
                    double slope2 = atan(slopeMatrix.at<double>(rowIdx2,0))*180/PI;

                    double distS = abs(slope2 - slope1);
  

                    if(distS < PARALLEL_LINE_DEGREE)
                    {
                        times++;
                    }
                }
                if((float)(times - 1) > slopeMatrix.rows*0.5)
                {
                        parallelFlag = true;
                        break;
                }
            }
            if(parallelFlag == false)
            {

                Mat slopeMat_T =  slopeMatrix.t();

                Mat HermitMat = slopeMat_T*slopeMatrix;
                Mat HermitMat_inv = HermitMat.inv(DECOMP_SVD);

                Mat location = HermitMat_inv*slopeMat_T*constMatrix;
                point3D_t  gpsReport;     
                gpsReport.lat = location.at<double>(0,0);
                gpsReport.lon = location.at<double>(1,0);

                {
                    // calculate the sign's height 
                    double minDistH = MIN_DIST;
                    float bestMuliple = 10000;
                    vector<point3D_t> positionVector = furnListInBuffIdx->position[0];
                    for(int stepIdx = 0;stepIdx < positionVector.size(); ++stepIdx)
                    {
                        double distX = positionVector[stepIdx].lat - gpsReport.lat;
                        double distY = positionVector[stepIdx].lon - gpsReport.lon;
                        double distH = sqrt(distX*distX + distY*distY);
                        if(minDistH > distH)
                        {
                            bestMuliple = furnListInBuffIdx->offset[0][stepIdx];
                            minDistH = distH;
                        }
                    }
                    if(minDistH < MIN_DIST)
                    {
                        // alt: height of traffic sign, in multiple of the height of sign surface, 
                        //      from ground to lower boundary of sign surface.
                        gpsReport.alt = bestMuliple;
                        furAttributesInVehicle_t reportFur;
                        reportFur = furnListInBuffIdx->furAttri;
                        reportFur.location = gpsReport;
                                
                        switch(reportFur.type)
                        {
                            case 27553:
                                reportFur.type = 27453;
                                break;
                            case 27554:
                                reportFur.type = 27454;
                                break;
                            case 27555:
                                reportFur.type = 27455;
                                break;
                            case 27556:
                                reportFur.type = 27456;
                                break;
                        }
                        outListPtr->push_back(reportFur);
                    }//end if(minDistH < MIN_DIST)
                } // end if(dist < DIST_THREHOLD)
            } // end if(parallelFlag == false)
        } // end if(furnListInBuffIdx->position.size() > VAR_ACC_TIME)
        ++furnListInBuffIdx;
    } // end while
}

void filterFurToReport(Point2d refGps,point3D_t currentGps,list<statisticsFurInfo_t>* furnListInBuffPtr, list<furWithPosition_t>* furnListInDetPtr,list<furAttributesInVehicle_t>* outListPtr)
{
    list<statisticsFurInfo_t>::iterator furnListInBuffIdx = furnListInBuffPtr->begin();
    list<furWithPosition_t>::iterator furnListInDetIdx = furnListInDetPtr->begin();
    list<furWithPosition_t>::iterator detIdx;
    //step1: find out the new detected furniture and push to the history list.
    while(furnListInDetIdx != furnListInDetPtr->end())
    {
        bool findTypeFlag = false;
        furnListInBuffIdx = furnListInBuffPtr->begin();
        while(furnListInBuffIdx != furnListInBuffPtr->end())
        {
            if(checkTwoFurnitureSameNoRange(&(furnListInDetIdx->furAttri), &(furnListInBuffIdx->furAttri), database_gp->_angleThresh))
            {
                findTypeFlag = true;
                break;
            }
            ++furnListInBuffIdx;
        }
        // the new furniture is not in history list, copy the information to history list
        if(!findTypeFlag)
        {
            statisticsFurInfo_t countFurInfo;
            countFurInfo.firstGps.push_back(currentGps);
            countFurInfo.furAttri = furnListInDetIdx->furAttri;
            countFurInfo.offsetNumPerFur.push_back(furnListInDetIdx->offsetNumPerFur);
            countFurInfo.offset.push_back(furnListInDetIdx->offset);
            countFurInfo.position.push_back(furnListInDetIdx->position);
            furnListInBuffPtr->push_back(countFurInfo);
            ++furnListInDetIdx;

        }
        // the furniture exists in history list, store the position
        else
        {
            
            furnListInBuffIdx->firstGps.push_back(currentGps);
               
            vector<point3D_t> posVector = furnListInDetIdx->position;
            furnListInBuffIdx->position.push_back(posVector); // store the temple relative position

            vector<float> offsetVector = furnListInDetIdx->offset;
            furnListInBuffIdx->offset.push_back(offsetVector); // store the offset array

            int stepVector = furnListInDetIdx->offsetNumPerFur;
            furnListInBuffIdx->offsetNumPerFur.push_back(stepVector); // store the offset number.

            furnListInBuffIdx->furAttri = furnListInDetIdx->furAttri; // update the furniture attributes.
            
            ++furnListInDetIdx;
        }
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
            if(checkTwoFurnitureSameNoRange(&(furnListInDetIdx->furAttri), &(furnListInBuffIdx->furAttri), database_gp->_angleThresh))
            {
                findTypeFlag = true;
                break;
            }
            ++furnListInDetIdx;
        }
        if(!findTypeFlag)
        {
            if(!checkGpsInRange(&currentGps,&(furnListInBuffIdx->firstGps.back()),database_gp->_distThreshNear))
            {
                point3D_t gpsReport;
                double minVar = MIN_DIST;
                float  minOffset = 50.0;

                if(furnListInBuffIdx->position.size() > VAR_ACC_TIME)
                {  
                    if(VAR_ALG == 1)
                    {
                    // calculate the GPS infomation according to different offset.  
                        int stepIdx;
                        int offsetIdx;

                        vector<point3D_t> meanLoc;

                        //step 1: calculate the location mean.
                       // for( stepIdx = 0; stepIdx < furnListInBuffIdx->offsetNumPerFur.size()-1; stepIdx++)
                        {
                            int stepNum =  furnListInBuffIdx->offsetNumPerFur[0];
                            vector<float> offsetVector = furnListInBuffIdx->offset[0];
                            vector<point3D_t> positionVector = furnListInBuffIdx->position[0];

                            for( offsetIdx = 0; offsetIdx < stepNum;offsetIdx++)
                            {                
                                float stepSize = offsetVector[offsetIdx];
                                int counter = 1;
                                int offsetIdx2 = 0;
                                point3D_t meanLocTemp;
                 
                                meanLocTemp = positionVector[offsetIdx];                 

                                for(int stepIdx2 = 1; stepIdx2 < furnListInBuffIdx->offsetNumPerFur.size(); stepIdx2++)
                                {
                                        int stepNum2 =  furnListInBuffIdx->offsetNumPerFur[stepIdx2];
                                        vector<float> offsetVector2 = furnListInBuffIdx->offset[stepIdx2];
                                        vector<point3D_t> positionVector2 = furnListInBuffIdx->position[stepIdx2];
                           
                                        while(offsetIdx2 < stepNum2)
                                        {
                                            if(offsetVector2[offsetIdx2] - stepSize < 0.01)
                                            {  break; }
                                            else
                                            {   offsetIdx2++; }
                                        }
 

                                        // calculate furniture's position in the two frames in same step;
                                        if(offsetIdx2 < stepNum2) 
                                        {
                                            // calculate the distance between the two frames;
                                            double distX = (positionVector2[offsetIdx2].lat - positionVector[offsetIdx].lat) * (positionVector2[offsetIdx2].lat - positionVector[offsetIdx].lat);
                                            double distY = (positionVector2[offsetIdx2].lon - positionVector[offsetIdx].lon) * (positionVector2[offsetIdx2].lon - positionVector[offsetIdx].lon);
                                            double distXY = sqrt(distX + distY); 

                                            //  distance < 50
                                            if(distXY < DIST_METER)
                                            {
                                                meanLocTemp.lat += positionVector2[offsetIdx2].lat;
                                                meanLocTemp.lon += positionVector2[offsetIdx2].lon;
                                                counter ++;
                                            }
                                        }
                                }
                                if(counter != 1)
                                {
                                    meanLocTemp.lat = meanLocTemp.lat / counter;
                                    meanLocTemp.lon = meanLocTemp.lon / counter;
                                    meanLoc.push_back(meanLocTemp);
                                }
                                else // counter == 1, only one frame position data is valid, do not to calculate the mean position.
                                {
                                    meanLocTemp.lat = 9e+100;
                                    meanLocTemp.lon = 9e+100;
                                    meanLoc.push_back(meanLocTemp);
                                }
                            }      
                        }

                        // step 2: calculate the location variance.
                        {
                            int stepNum =  furnListInBuffIdx->offsetNumPerFur[0];
                            vector<float> offsetVector = furnListInBuffIdx->offset[0];
                            vector<point3D_t> positionVector = furnListInBuffIdx->position[0];

                            for( offsetIdx = 0; offsetIdx < stepNum;offsetIdx++)
                            {                
                                float stepSize = offsetVector[offsetIdx];
                                int counter = 1;

                                int offsetIdx2 = 0;
                        
                                point3D_t locTemp; 

                                double varTemp;
                                locTemp.lat = positionVector[offsetIdx].lat - meanLoc[offsetIdx].lat;
                                locTemp.lon = positionVector[offsetIdx].lon - meanLoc[offsetIdx].lon;
                                varTemp = locTemp.lat*locTemp.lat + locTemp.lon*locTemp.lon;

                                for(int stepIdx2 = 1; stepIdx2 < furnListInBuffIdx->offsetNumPerFur.size(); stepIdx2++)
                                {
                                    int stepNum2 =  furnListInBuffIdx->offsetNumPerFur[stepIdx2];
                                    vector<float> offsetVector2 = furnListInBuffIdx->offset[stepIdx2];
                                    vector<point3D_t> positionVector2 = furnListInBuffIdx->position[stepIdx2];
                           
                                    while(offsetIdx2 < stepNum2)
                                    {
                                        if(offsetVector2[offsetIdx2] == stepSize)
                                        {  break; }
                                        else
                                        {   offsetIdx2++; }
                                    }
                            
                                    if((offsetIdx2 < stepNum2) && (meanLoc[offsetIdx].lat != 9e+100) && (meanLoc[offsetIdx].lon != 9e+100))
                                    {
                                        locTemp.lat = positionVector2[offsetIdx2].lat - meanLoc[offsetIdx].lat;
                                        locTemp.lon = positionVector2[offsetIdx2].lon - meanLoc[offsetIdx].lon;
                                        varTemp += locTemp.lat*locTemp.lat + locTemp.lon*locTemp.lon;
                                        counter ++;
                                    }
                                }

                                if((minVar > varTemp/counter) && (counter > VAR_ACC_TIME))
                                {
                                    minVar = varTemp/counter;
                                    gpsReport = meanLoc[offsetIdx];
                                    minOffset = offsetVector[offsetIdx]; 
                                }
                                //cout << "var time:" << counter << endl;
                            }
                        }
                        //cout << "best multiple index: " << minOffset << endl;
                        // report
                        if(minVar < MIN_DIST)
                        {
                            furAttributesInVehicle_t reportFur;
                            reportFur = furnListInBuffIdx->furAttri;
                            reportFur.location = gpsReport;
                            outListPtr->push_back(reportFur);
                        }
                    }        
                    else
                    {

                        Mat slopeMatrix(furnListInBuffIdx->position.size(),2,CV_64FC1);
                        Mat constMatrix(furnListInBuffIdx->position.size(),1,CV_64FC1);
                        
                        for(int stepIdx1 = 0;stepIdx1 < furnListInBuffIdx->position.size(); ++stepIdx1)
                        {
                            vector<point3D_t> positionVector = furnListInBuffIdx->position[stepIdx1];
                            vector<Point2f> positionVec2d;
                            // 3d point transform to 2d.
                            for(int numIdx = 0; numIdx < positionVector.size();++numIdx)
                            {
                                Point2f point;
                                point.x = positionVector[numIdx].lat;
                                point.y = positionVector[numIdx].lon;
                                positionVec2d.push_back(point);
                            }
                            // line fitting
                            Vec4f     lineFit;
                            fitLine(positionVec2d,lineFit,CV_DIST_L2,1,0.01,0.01);
                            // Ax=b, construct the A matrix and b matrix
                            slopeMatrix.at<double>(stepIdx1,0) = lineFit[1]/lineFit[0];
#ifdef TRAFFIC_SIGN_TEST
                            fprintf(fd,"%d_%d_K = %.14f;\n",furnListInBuffIdx->furAttri.type,furnListInBuffIdx->furAttri.sideFlag,atan2(lineFit[1],lineFit[0]));
#endif
                            slopeMatrix.at<double>(stepIdx1,1) = -1.0;
                            constMatrix.at<double>(stepIdx1) = lineFit[1]/lineFit[0] * lineFit[2] - lineFit[3];
                        }
                         //delete the parallel lines
                        bool parallelFlag = false;
                        for(int rowIdx1 = 0;rowIdx1 < slopeMatrix.rows; ++rowIdx1)
                        {
                            int times = 0;
                            double slope1 = atan(slopeMatrix.at<double>(rowIdx1,0))*180/PI;

                            for(int rowIdx2 = 0; rowIdx2 < slopeMatrix.rows; ++rowIdx2)
                            {                           
                                double slope2 = atan(slopeMatrix.at<double>(rowIdx2,0))*180/PI;

                                double distS = abs(slope2 - slope1);
  

                                if(distS < PARALLEL_LINE_DEGREE)
                                {
                                    times++;
                                }
                            }
                            if((float)(times - 1) > slopeMatrix.rows*0.5)
                            {
                                 parallelFlag = true;
                                 break;
                            }
                        }
                        if(parallelFlag == false)
                        {

                            Mat slopeMat_T =  slopeMatrix.t();

                            Mat HermitMat = slopeMat_T*slopeMatrix;
                            Mat HermitMat_inv = HermitMat.inv(DECOMP_SVD);

                            Mat location = HermitMat_inv*slopeMat_T*constMatrix;
                        
                            gpsReport.lat = location.at<double>(0,0);
                            gpsReport.lon = location.at<double>(1,0);
                            gpsReport.alt = furnListInBuffIdx->position[0][0].alt;

                            Point2d frameGps,relativeGps1;
                            frameGps.x = furnListInBuffIdx->firstGps.back().lat;
                            frameGps.y = furnListInBuffIdx->firstGps.back().lon;
                            coordinateChange(frameGps,refGps,relativeGps1);

                            double distX = relativeGps1.x - gpsReport.lat;
                            double distY = relativeGps1.y - gpsReport.lon;
                            double dist = sqrt(distX*distX + distY*distY);
#ifdef TRAFFIC_SIGN_TEST
                            fprintf(fd,"distance_%d  = %.14f;\n",furnListInBuffIdx->furAttri.type,dist);
#endif
 
                            if(dist < DIST_THREHOLD)
                            {
                                // calculate the sign's height 
                                double minDistH = MIN_DIST;
                                float bestMuliple = 10000;
                                vector<point3D_t> positionVector = furnListInBuffIdx->position[0];
                                for(int stepIdx = 0;stepIdx < positionVector.size(); ++stepIdx)
                                {
                                    double distX = positionVector[stepIdx].lat - gpsReport.lat;
                                    double distY = positionVector[stepIdx].lon - gpsReport.lon;
                                    double distH = sqrt(distX*distX + distY*distY);
                                    if(minDistH > distH)
                                    {
                                        bestMuliple = furnListInBuffIdx->offset[0][stepIdx];
                                        minDistH = distH;
                                    }
                                }
                                if(minDistH < MIN_DIST)
                                {
                                    // alt: height of traffic sign, in multiple of the height of sign surface, 
                                    //      from ground to lower boundary of sign surface.
                                    gpsReport.alt = bestMuliple;
                                    furAttributesInVehicle_t reportFur;
                                    reportFur = furnListInBuffIdx->furAttri;
                                    reportFur.location = gpsReport;
                                    outListPtr->push_back(reportFur);
                                }
                            }
                        }
                    }
                }
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
            ++furnListInBuffIdx;
        }
    }
}


