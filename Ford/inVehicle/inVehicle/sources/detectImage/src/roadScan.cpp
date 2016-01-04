/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  roadScan.cpp
* @brief Road detection source file
*
* Change Log:
*      Date                Who             What
*      2015/05/20         Bingtao Gao      Create
*******************************************************************************
*/

#include <windows.h>
#include <opencv2\opencv.hpp>
#include "roadScan.h"
#include <math.h>
#include "ImageBuffer.h"
#include "utils.h"
#include <iomanip>
#include <fstream>
#include "AppInitCommon.h" // H
#include "configure.h"

using namespace std;
using namespace cv;

namespace ns_roadScan
{

int roadImageGen(Mat &imageIn, Mat &history, int *rowIndex, Point2d *GPS_abs, Point2d *GPS_next,Mat &H, double cmPerPixel, int startRow,
       gpsInformationAndInterval *gpsAndInterval, int *intrtmp,Parameters inParam,Point2d &GPS_stop,bool &stopFlg)
{

	Point2d *GPS_ref = &inParam.GPSref;

	Mat image, image_32f, ColorImage;
	if(imageIn.empty())
	{
		return -1;
	}

	Size Rsize = Size(imageIn.cols*inParam.imageScaleWidth, imageIn.rows*inParam.imageScaleHeight);
    image = Mat(Rsize,CV_8UC1);
    resize(imageIn, imageIn,Rsize);
    int nchannel = imageIn.channels();
    if(nchannel == 3)
    {
        cvtRGB2Gray(imageIn, image);
    }
    else
    {
        image = imageIn;
    }
	medianBlur(image,image,3) ;
    ColorImage = image;

    /*
	image.convertTo(image_32f,CV_32F);
	processNoise(image_32f,image);
	image.convertTo(ColorImage,CV_8UC1);
	cvtColor(ColorImage, ColorImage, COLOR_GRAY2RGB);
    */
#ifdef ROAD_SCAN_UT
	SYSTEMTIME systime;
	GetLocalTime(&systime);
	std::string strKMLlfilename;
	char cKMLlfilename[30];
	sprintf(cKMLlfilename, "%d-%02d-%02d_%02d%02d%02d_%03d_road.png",
		systime.wYear,
		systime.wMonth,
		systime.wDay,
		systime.wHour,
		systime.wMinute,
		systime.wSecond,
		systime.wMilliseconds);
	strKMLlfilename = cKMLlfilename;
    imwrite(strKMLlfilename.c_str(),image);
    waitKey(1);
#endif

    int w = image.cols;
    int h = image.rows;

	double startLocation;
    Mat image_f;
    image.convertTo(image_f,CV_32F);	
    ///////////////////////////////////////////////////////////////////////////////////////////
    //Generate BirdView
    Mat birds_image;
    double scale = 1;
	startLocation = startRow;		
    Mat homography = H;

	Size birdSize = Size(image_f.cols*scale,image_f.rows*scale*inParam.stretchRate);
    warpPerspective(
        ColorImage,
        birds_image,
        H,
        birdSize,
        INTER_LINEAR  | WARP_INVERSE_MAP
        );
    Mat birdEye;
    birdEye = birds_image;
    
	Point2d GPS_rel, GPS_rel2;
    double d = 0;
    if (!stopFlg)
    {
        coordinateChange(*GPS_abs, *GPS_ref, GPS_rel);
        coordinateChange(*GPS_next, *GPS_ref, GPS_rel2);
        d = sqrt((GPS_rel.x-GPS_rel2.x)*(GPS_rel.x-GPS_rel2.x)+(GPS_rel.y-GPS_rel2.y)*(GPS_rel.y-GPS_rel2.y));
        if (d < 0.010)
        {
            GPS_stop = *GPS_abs;
            stopFlg = true;
            d = 0;
        }
        else
        {
            stopFlg = false;
            GPS_stop = Point2d(0,0);
        }
            
    }
    else
    {
        coordinateChange(*GPS_abs, *GPS_ref, GPS_rel);
        coordinateChange(GPS_stop, *GPS_ref, GPS_rel2);
        d = sqrt((GPS_rel.x-GPS_rel2.x)*(GPS_rel.x-GPS_rel2.x)+(GPS_rel.y-GPS_rel2.y)*(GPS_rel.y-GPS_rel2.y));
        if (d > 0.010)
        {
            coordinateChange(*GPS_abs, *GPS_ref, GPS_rel);
            coordinateChange(*GPS_next, *GPS_ref, GPS_rel2);
            d = sqrt((GPS_rel.x-GPS_rel2.x)*(GPS_rel.x-GPS_rel2.x)+(GPS_rel.y-GPS_rel2.y)*(GPS_rel.y-GPS_rel2.y));
            GPS_stop = Point2d(0,0);
            stopFlg = false;
        }
        else
        {
            stopFlg = true;
            d = 0;
        }           
    }
	
	int Interval =floor(d/(cmPerPixel/100)+0.5);
  
	//if(Interval >= 1)
	{
		*rowIndex = *rowIndex - *intrtmp;

		if ((*rowIndex)-Interval < 0)
		{
			cout<<"roadscan:extend history rows"<<endl;
			return -1;
		}
		int startLocationFloor = floor(startLocation);
		if(((startLocationFloor-Interval+1) < 0)||(startLocationFloor + 1) > birdEye.rows)
		{
			cout<<"roadscan:extend bird view rows"<<endl;
			return -1;
		}
		Mat imageROI = birdEye(Rect(0,floor(startLocation)-Interval+1,birdEye.cols,Interval));

		//Interval = ceil(Interval/inParam.downSampling);
        Interval = ceil(Interval/2.0);
		if (Interval>=1)
		{
			resize(imageROI,imageROI,Size(birdEye.cols,Interval));
		}
				
		Mat history_roi;
		history_roi = history(Rect(0,(*rowIndex)-Interval,history.cols,Interval));
		imageROI.copyTo(history_roi,imageROI);

		if(Interval>0)
		{
        	gpsAndInterval->GPS_now = *GPS_abs;
        	gpsAndInterval->GPS_next = *GPS_next;
        	gpsAndInterval->intervalOfInterception = Interval;
		}
		else
		{
			gpsAndInterval->intervalOfInterception = 0;
		}
		int xx = 0;
		int yy;
        // if the buffer is larger than 2H,show 2H height image
        if ((*rowIndex-Interval + 2*h) < history.rows)
        {
            yy = *rowIndex-Interval;
        }
        else 
        {
            yy = history.rows - 2*h;
        }
		if((yy >= 0) && (yy+2*h <= history.rows))
		{
			Rect rect = Rect(xx, yy, history.cols, 2*h);
			cv::namedWindow("Road Scan",CV_WINDOW_NORMAL);
			imshow("Road Scan",history(rect));
		}
	}

	*intrtmp = Interval;
#ifdef ROAD_SCAN_UT
	imshow( "Birds View", birdEye );
	
    imwrite("BirdView.png",birdEye);
#endif
	waitKey(1);

	return 0;
}
void roadImageProc2(Mat &longLane, Parameters &inParam, vector<gpsInformationAndInterval> &gpsAndInterval, vector<dataEveryRow> &roadPaintData, 
    vector<landMark> &vecLandMark,Mat &DrawMarker)
{

#ifdef ROAD_SCAN_UT
    double startT = static_cast<double>(cv::getTickCount());
#endif
	int W = longLane.cols;
	int H = longLane.rows;

    int W_half = W * 0.5;

	vector<int> changeLP;
	vector<dataEveryRow> middleRoadPaintData;
	vecLandMark.clear();

	for(int i = 0;i<H;i++)
	{
		dataEveryRow newData;

		newData.leftPoint = Point(-1, -1);
		newData.Left_Middle_RelGPS = Point2d(0.0, 0.0);
		newData.isPaint_Left = -1;
		newData.left_Edge_XY[0] = Point(-1, -1);
		newData.left_Edge_XY[1] = Point(-1, -1);
		newData.Left_Paint_Edge[0] = Point2d(0.0, 0.0);
		newData.Left_Paint_Edge[1] = Point2d(0.0, 0.0);
		newData.Left_Area_Pixel_Mean = -1.0;

		newData.Middle_RelGPS = Point2d(0.0, 0.0);
		newData.Middle_Area_Pixel_Mean = -1.0;

		newData.rightPoint = Point(-1, -1);
		newData.Right_Middle_RelGPS = Point2d(0.0, 0.0);
		newData.isPaint_Right = -1;
		newData.right_Edge_XY[0] = Point(-1, -1);
		newData.right_Edge_XY[1] = Point(-1, -1);
		newData.Right_Paint_Edge[0] = Point2d(0.0, 0.0);
		newData.Right_Paint_Edge[1] = Point2d(0.0, 0.0);
		newData.Right_Area_Pixel_Mean = -1.0;

		middleRoadPaintData.push_back(newData);
	}
	
    Mat PaintLine = Mat::zeros(longLane.rows,longLane.cols,CV_8UC1);

	int n_cut;
	n_cut = ceil(longLane.rows/CUT);
	for (int kk=1;kk<=n_cut;kk++)
	{
		Mat longLane_cut;
		if (kk==n_cut)
			longLane_cut = longLane(Rect(0,(kk-1)*CUT,longLane.cols,longLane.rows-(kk-1)*CUT));
		else
			longLane_cut = longLane(Rect(0,(kk-1)*CUT,longLane.cols,CUT));


		int w = longLane_cut.cols;
		int h = longLane_cut.rows;
		Mat lineLinkOut,allUnitContous(h,w,CV_8UC1);
		{
			Mat longLane_cut2;
			shadowProcess(longLane_cut, longLane_cut2);
#if((RD_ROAD_SIGN_DETECT_STOPLINE_MASK & RD_ROAD_SIGN_DETECT) )

			landMark landmark1;
			vector<Point> stopLineLoc;
			//stopLineDetection(longLane_cut2,stopLineLoc);
			for (int i=0;i<stopLineLoc.size();i++)
			{
				stopLineLoc[i].y = H - ((kk-1)*CUT + stopLineLoc[i].y);
				landmark1.center = stopLineLoc[i];
				landmark1.type = 1000;
				landmark1.flag = 4;
				vecLandMark.push_back(landmark1);
	//            circle(RGBImg,landmark1.center,0,Scalar(0,0,255),6);
			}
#endif
         
#if ((RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT) || (RD_GERMAN_MUNICH_AIRPORT_LARGE == RD_LOCATION))
	//		circleDection(longLane_cut,landMark,histThres);
			vector<landMark> arrows;
			{
			Mat longLane_cutBW;
			gray2BW(longLane_cut,longLane_cutBW);
			
#ifdef DEBUG_ROAD_SCAN
			imwrite("longLane_cutBW.png",longLane_cutBW);
#endif
			arrowDetection(longLane_cut,longLane_cutBW,arrows,longLane_cut2);
             }
#ifdef DEBUG_ROAD_SCAN
			imwrite("longLane_cut2.png",longLane_cut2);
#endif
			for (int i=0;i<arrows.size();i++)
			{
				arrows[i].center.y = H - ((kk-1)*CUT + arrows[i].center.y );
				arrows[i].hight = arrows[i].hight * inParam.distancePerPixel/100;
				arrows[i].width = arrows[i].width * inParam.distancePerPixel/100;
				vecLandMark.push_back(arrows[i]);
			}
        
	#endif
        
	///////shadow and no shadow ,process respectively
			{
//			Mat allUnitKapa(h,w,CV_8UC1);
//			allUnitKapa.convertTo(allUnitKapa,CV_8UC1,1024); 
//			Mat allKappBin;
//			blockCalRidge(longLane_cut2,inParam,allUnitKapa,allUnitContous,allKappBin);
			blockCalRidge(longLane_cut2,inParam,allUnitContous);
			}

			linkInterval(allUnitContous,allUnitContous);
			threshold( allUnitContous, allUnitContous,5, 255,0 );

	#ifdef DEBUG_ROAD_SCAN
			imwrite("allUnitContous2.png",allUnitContous);
	#endif
		}

		linkPaintLine(allUnitContous,lineLinkOut);

		//step3: find the left and right lane.
		int w1 = allUnitContous.cols;
		int h1 = allUnitContous.rows;

        Mat roadDraw3 = Mat::zeros(lineLinkOut.size(),CV_8UC3);
		cvtColor(lineLinkOut, roadDraw3, COLOR_GRAY2RGB);
        //cvtColor(roadLanes, roadLanes, COLOR_GRAY2RGB);

		for(int i = h1-1; i >= 0; i--)//h-1
		{
			int changeLV = lineLinkOut.at<uchar>(i,W_half);
			if(changeLV!=0)
				{
					changeLP.push_back((H-1)-(i+(kk-1)*CUT));
				}

				//find first white
				vector<int> marker;
				for(int j = 0; j < w1; j++)
				{
					int data = lineLinkOut.at<uchar>(i,j);
					if  (data != 0) 
					{
						marker.push_back(j);
					}
				}
				//for each line, for the left lane and right lane
				//choose the one most close to the center.
				int size = marker.size();
				if (size > 0)
				{
					int left = -1,right = w1;
					for (int k = 0; k < size ; k++)
					{
						if (( marker[k] < w1/2.0) && (marker[k] > left))
						{
							left = marker[k];
						}

						if (( marker[k] >= w1/2.0) && (marker[k] < right))
						{
						  	right = marker[k];	
						}
					}
					if (left>-1 ) 
					{
						int dataMiddle = allUnitContous.at<uchar>(i,left);
						if(dataMiddle!=0)
						{
							
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].leftPoint.x = left;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].leftPoint.y = i+(kk-1)*CUT;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].isPaint_Left = 1;
							circle(roadDraw3, Point(left,i),0,CV_RGB(255, 0, 0),4);
						}
					}

					if (right< w1) 
					{
						int dataMiddle = allUnitContous.at<uchar>(i,right);
						if(dataMiddle!=0)
						{
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].rightPoint.x = right;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].rightPoint.y = i+(kk-1)*CUT;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].isPaint_Right = 1;
                            circle(roadDraw3, Point(right,i),0,CV_RGB(0, 0, 255),4);
						}
					}
				}			
		}
		
		if (kk==n_cut)
		{
			Mat drawroi = DrawMarker(Rect(0,(kk-1)*CUT,longLane.cols,longLane.rows-(kk-1)*CUT));
			roadDraw3.copyTo(drawroi,roadDraw3);
		
#ifdef DEBUG_ROAD_SCAN 
			imwrite( "road MarkerAll.png", DrawMarker );
#endif
			

			Mat paintlineroi = PaintLine(Rect(0,(kk-1)*CUT,longLane.cols,longLane.rows-(kk-1)*CUT));
			allUnitContous.copyTo(paintlineroi,allUnitContous);
			//imwrite( "road PaintLine.png", PaintLine );
		}
		else
		{
			Mat drawroi = DrawMarker(Rect(0,(kk-1)*CUT,longLane.cols,CUT));
			roadDraw3.copyTo(drawroi,roadDraw3);
			//imwrite( "road Marker12.png", DrawMarker);

			Mat paintlineroi = PaintLine(Rect(0,(kk-1)*CUT,longLane.cols,CUT));
			allUnitContous.copyTo(paintlineroi,allUnitContous);
			//imwrite( "road Marker12.png", DrawMarker);
		}
		
	}

	//fullfil the data struct
	int startPoint = 0;

	for(int i=0;i<gpsAndInterval.size();i++)
	{
		int endPoint = startPoint+gpsAndInterval[i].intervalOfInterception;

		if(endPoint > H)
			break;

		for(int j=startPoint;j<endPoint;j++)
		{
			//current interval
			getInformationOfEveryLine(gpsAndInterval[i],DrawMarker,longLane,j,startPoint,middleRoadPaintData[j], inParam);
		}

		startPoint = endPoint;

	}

	//change line
#if 1
	sort(changeLP.begin(),changeLP.end());

	int thresholdCL = 30;
	vector<int> changelinePoint;

	for(int i = 0; i<changeLP.size(); i++)
	{
		if((changeLP[i]+thresholdCL)<H)
		{
			if(middleRoadPaintData[changeLP[i]+thresholdCL].leftPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]+thresholdCL].leftPoint.x-W_half)<50)
				{
					changelinePoint.push_back(changeLP[i]);
					continue;
				}
			}

			if(middleRoadPaintData[changeLP[i]+thresholdCL].rightPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]+thresholdCL].rightPoint.x-W_half)<50)
				{
					changelinePoint.push_back(changeLP[i]);
					continue;
				}
			}
		}

		if((changeLP[i]-thresholdCL)>=0)
		{
			if(middleRoadPaintData[changeLP[i]-thresholdCL].leftPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]-thresholdCL].leftPoint.x-W_half)<50)
				{
					changelinePoint.push_back(changeLP[i]);
					continue;
				}
			}

			if(middleRoadPaintData[changeLP[i]-thresholdCL].rightPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]-thresholdCL].rightPoint.x-W_half)<50)
				{
					changelinePoint.push_back(changeLP[i]);
					continue;
				}
			}
		}
	}

	for (int idx = 0; idx < changelinePoint.size(); idx++)
	{
		//line change
		int sP = changelinePoint[idx]-100;
		int eP = changelinePoint[idx]+100;

        if(sP < 0)
		{
			sP = 0;
		}

        if(eP > H)
        {
            eP = H;
        }

        if(inParam.discardRoadDataAfterLaneChange == 1)
		{
		    eP = H;
        }
		

		for(int i = sP; i < eP; i++)
		{
			middleRoadPaintData[i].leftPoint = Point(-1, -1);
            
            if(inParam.discardRoadDataAfterLaneChange == 1)
            {
                middleRoadPaintData[i].Left_Middle_RelGPS = Point2d(0.0, 0.0);
                middleRoadPaintData[i].isPaint_Left = -1;
            }
            else
            {
                middleRoadPaintData[i].isPaint_Left = 2;
            }
			
			middleRoadPaintData[i].left_Edge_XY[0] = Point(-1, -1);
			middleRoadPaintData[i].left_Edge_XY[1] = Point(-1, -1);
			middleRoadPaintData[i].Left_Paint_Edge[0] = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Left_Paint_Edge[1] = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Left_Area_Pixel_Mean = -1.0;

//			middleRoadPaintData[i].Middle_RelGPS = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Middle_Area_Pixel_Mean = -1.0;

			middleRoadPaintData[i].rightPoint = Point(-1, -1);

            if(inParam.discardRoadDataAfterLaneChange == 1)
            {
                middleRoadPaintData[i].Right_Middle_RelGPS = Point2d(0.0, 0.0);
                middleRoadPaintData[i].isPaint_Right = -1;
            }
            else
            {
                middleRoadPaintData[i].isPaint_Right = 2;
            }

			middleRoadPaintData[i].right_Edge_XY[0] = Point(-1, -1);
			middleRoadPaintData[i].right_Edge_XY[1] = Point(-1, -1);
			middleRoadPaintData[i].Right_Paint_Edge[0] = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Right_Paint_Edge[1] = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Right_Area_Pixel_Mean = -1.0;
		}
	}
#endif

	//roadPaintData - data struct
	for(int index = 0; index<H; index++)
	{
		if(index%inParam.downSampling == 0)
		{
			roadPaintData.push_back(middleRoadPaintData[index]);
		}
	}
	
	if (vecLandMark.size())
	{
		for (int i=0;i<vecLandMark.size();i++)
		{
            calLandMarkRelGPS(gpsAndInterval,vecLandMark[i].center,longLane,inParam,vecLandMark[i].centerRel,vecLandMark[i].angleVec);         
		}
    }


#ifdef ROAD_SCAN_UT
    double time = (static_cast<double>(cv::getTickCount() - startT))/cv::getTickFrequency();
    cout<<"time:"<<time<<endl;
#endif

}
bool readParamRoadScan(char* paramFileName, Parameters& inParam)
{
	ifstream readParam(paramFileName, ios::_Nocreate);

	if(!readParam)
	{
		cout<<"read parameters error"<<endl;
		return false;
	}

	//get parameters
	string str;

	while(getline(readParam,str))
	{
		if(str == "centerPoint:")
		{
			readParam>>inParam.centerPoint.x>>inParam.centerPoint.y;
		}else if(str == "lengthRate:")
		{
			readParam>>inParam.lengthRate;
		}else if(str == "distanceOfSlantLeft:")
		{
			readParam>>inParam.distanceOfSlantLeft;
		}else if(str == "distanceOfSlantRight:")
		{
			readParam>>inParam.distanceOfSlantRight;
		}else if(str == "distanceOfUpMove:")
		{
			readParam>>inParam.distanceOfUpMove;
		}else if(str == "distanceOfLeft:")
		{
			readParam>>inParam.distanceOfLeft;
		}else if(str == "distanceOfRight:")
		{
			readParam>>inParam.distanceOfRight;
		}else if(str == "ridgeThreshold:")
        {
            readParam>>inParam.ridgeThreshold;
        }else if(str == "stretchRate:")
		{
			readParam>>inParam.stretchRate;
		}else if(str == "downSampling:")
		{
			readParam>>inParam.downSampling;
		}else if(str == "distancePerPixel:")
		{
			readParam>>inParam.distancePerPixel;
		}
        else if(str == "distancePerPixelX:")
		{
			readParam>>inParam.distancePerPixelX;
        }
        else if(str == "distancePerPixelY:")
		{
			readParam>>inParam.distancePerPixelY;
        }else if(str == "GPSref:")
		{
			readParam>>inParam.GPSref.x>>inParam.GPSref.y;
		}else if(str == "imageScaleHeight:")
		{
			readParam>>inParam.imageScaleHeight;
		}else if(str == "imageScaleWidth:")
		{
			readParam>>inParam.imageScaleWidth;
		}else if(str == "imageRows:")
		{
			readParam>>inParam.imageRows;
		}else if(str == "imageCols:")
		{
			readParam>>inParam.imageCols;
        }else if(str == "discardRoadDataAfterLaneChange:")
		{
			readParam>>inParam.discardRoadDataAfterLaneChange;
        }else if(str == "offsetDistance:")
        {
            readParam>>inParam.offsetDist;
        }else if(str == "laneScaleX:")
        {
            readParam>>inParam.laneScaleX;
        }else if(str == "laneScaleY:")
        {
            readParam>>inParam.laneScaleY;
        }else if(str == "startRowCurLane:")
        {
            readParam>>inParam.startRowCurLane;
        }else if(str == "startRowAllLane:")
        {
            readParam>>inParam.startRowAllLane;
        }else if(str == "rowDownMoveNumber:")
        {
            readParam>>inParam.rowDownMoveNumber;
        }
        else
        {
            //cout<<"Undefined parameter"<<endl;
        }
	}

	return true;
}

bool currentLaneMatched(Mat &roadDetected, Mat &historyROI1, Mat &historyROI2, 
                        vector<gpsInformationAndInterval> &GPSAndInterval1 ,
                        vector<gpsInformationAndInterval> &GPSAndInterval2,
                        Parameters& inParam,Mat &outImage,float *width,float *meanX,Mat &invertH, Mat &laneH)
{
    if (GPSAndInterval1.empty() || GPSAndInterval2.empty())
    {
        return false;
    }

    bool successFlag = false;
    int offsetRow = inParam.startRowCurLane * inParam.laneScaleY + inParam.rowDownMoveNumber - inParam.startRowAllLane ;  
    int rowNum = historyROI2.rows + offsetRow;
    int colNum = historyROI2.cols;
    Mat imageTransform = Mat::zeros(rowNum, colNum, CV_8UC3);

    int rowIndex1 = historyROI1.rows - GPSAndInterval1[0].intervalOfInterception;
    int rowIndex2 = imageTransform.rows - GPSAndInterval2[0].intervalOfInterception;

    Mat matrixH = invertH*laneH;

    int minRow = min(GPSAndInterval1.size(),GPSAndInterval2.size());


    for(int ii = 0; ii < minRow; ii++)
    {            
        // detect current lane
        Mat roi = roadDetected(Rect(0,rowIndex1,roadDetected.cols,GPSAndInterval1[ii].intervalOfInterception));
                
        Size Rsize = Size(inParam.imageCols*inParam.imageScaleWidth, inParam.imageRows);

        Mat oriImage = Mat::zeros(Rsize,CV_8UC3);
        Mat roi1 = oriImage(Rect(0, inParam.startRowCurLane - roi.rows,Rsize.width,roi.rows));
        roi.copyTo(roi1,roi);


        Mat image_f;
        oriImage.convertTo(image_f,CV_32FC3);
	
        Mat roi2 = imageTransform(Rect(0,rowIndex2, historyROI2.cols,GPSAndInterval2[ii].intervalOfInterception));

        Mat roi3 ;

        Size birdSize = Size(image_f.cols,image_f.rows);//Size(roi2.cols,roi2.rows);
        // bired view
        warpPerspective(
            oriImage,
            roi3,
            matrixH,
            birdSize,
            INTER_LINEAR| WARP_INVERSE_MAP
        );
        int row2 = inParam.startRowCurLane * inParam.laneScaleY + inParam.rowDownMoveNumber;
        Mat roi4 = roi3(Rect(0,row2 - GPSAndInterval2[ii].intervalOfInterception,roi3.cols,GPSAndInterval2[ii].intervalOfInterception));
  
        roi4.convertTo(roi4,CV_8UC3);
        roi4.copyTo(roi2,roi4);

        if (ii < minRow -1 )
        {
            rowIndex1 -=  GPSAndInterval1[ii+1].intervalOfInterception;
            rowIndex2 -=  GPSAndInterval2[ii+1].intervalOfInterception;
        }
        if (rowIndex1 < 0) 
        { rowIndex1 = 0;}
    }

#ifdef ROAD_SCAN_UT
    imwrite("thransformed.png",imageTransform);
#endif            
       
    //Mat lineLinkOut1,lineLinkOut2;
    //cvtColor(imageTransform,lineLinkOut1,COLOR_RGB2GRAY);

    //linkPaintLine(lineLinkOut1,lineLinkOut2);

 
    Mat history3;
    cvtColor(historyROI2, history3, COLOR_GRAY2RGB);

    // loop up for left and right lines of current lane
    vector<Point> curLeftLine;
    vector<Point> curRightLine;

    for(int rowIdx = 0; rowIdx < historyROI2.rows; rowIdx++ )
    {
        for(int colIdx = 0; colIdx < historyROI2.cols; colIdx++)
        {
            Vec3b colorC3 = imageTransform.at<Vec3b>(rowIdx+offsetRow*(1-inParam.laneScaleX),colIdx);//FIXME:0.5 current lane cmPerpixles/all lane cmPerpixels
                
            if ((colorC3[2] > 0) && (colorC3[1] == 0) && (colorC3[0] == 0))
            { 
                circle(history3, Point(colIdx,rowIdx),0,CV_RGB(255, 0, 0),4);
                curLeftLine.push_back(Point(colIdx,rowIdx));
            }
            else if((colorC3[0] > 0) && (colorC3[1] == 0) && (colorC3[2] == 0))
            {
                circle(history3, Point(colIdx,rowIdx),0,CV_RGB(0, 0, 255),4);
                curRightLine.push_back(Point(colIdx,rowIdx));
            }
        }
    }
#ifdef ROAD_SCAN_UT
    imwrite("current_lane.png",history3);
#endif
    history3.copyTo(outImage);

    // choose 10 pointers to calculate the lane width
    float widthTemp = 0.0;
    int   meanXTemp = 0;

    if((curLeftLine.size() >= 40) && (curRightLine.size() >= 40))
    {

        int leftNum = curLeftLine.size()/5;
        int rightNum = curRightLine.size()/5;
        int number = 0;
        for(int ii = 0; ii < 5; ii++)
        {
            vector<Point> leftPoint,rightPoint;
            leftPoint.push_back(curLeftLine[ii*leftNum]);
            leftPoint.push_back(curLeftLine[ii*leftNum+leftNum - 1]);

            rightPoint.push_back(curRightLine[ii*rightNum]);
            rightPoint.push_back(curRightLine[ii*rightNum+rightNum - 1]);

            if((abs(leftPoint[0].x - rightPoint[0].x) < (int)laneWidthThreshold)  || (abs(leftPoint[1].x - rightPoint[1].x) < (int)laneWidthThreshold))//FIXME: 60 the lane maximum width
            {
                continue;
            }
            number++;
            // calculate the current lane width
            Vec4f lineL,lineR;
            fitLine(leftPoint,lineL,CV_DIST_L2 ,0,0.01,0.01);
            fitLine(rightPoint,lineR,CV_DIST_L2 ,0,0.01,0.01);

            //float slopeL = lineL[1]/(lineL[0] + 0.00005);
           // float slopeR = lineR[1]/(lineR[0] + 0.00005);

            int y = rightPoint[1].y;
            int xL = (y - lineL[3])*(lineL[0])/(lineL[1]+0.0000000001) + lineL[2];
            int xR = (y - lineR[3])*(lineR[0])/(lineR[1]+0.0000000001) + lineR[2];
            int absX = abs(xL - xR);
            meanXTemp += (xL + absX*0.5);
            widthTemp += absX;
        }
        if (number > 0)
        {
            *meanX = meanXTemp/number;
            *width = widthTemp/number;
        }
        else
        {
            *meanX = 0;
            *width = 0;
        }
        if((*width >= laneWidthThreshold) && (*width < history3.cols/3) && (*meanX >= laneWidthThreshold/2) && (*meanX + 1.5*(*width) < history3.cols)) // FIXME:calcualte the width successfully
        {
            successFlag = true;
        }
    }
    else if(((curLeftLine.size() >= 2) && (curLeftLine.size() < 40)) || ((curRightLine.size() >= 2) && (curRightLine.size() < 40)))
    {
        int minNum = min(curLeftLine.size(),curRightLine.size());
        Vec4f lineL,lineR;
        fitLine(curLeftLine,lineL,CV_DIST_L2 ,1,0.01,0.01);
        fitLine(curRightLine,lineR,CV_DIST_L2 ,1,0.01,0.01);

        //float slopeL = lineL[1]/(lineL[0] + 0.00005);
        //float slopeR = lineR[1]/(lineR[0] + 0.00005);

        for(int ii = 0; ii < minNum; ii++)
        {
            int y = curLeftLine[ii].y;
            int xL = (y - lineL[3])*(lineL[0])/(lineL[1]+0.0000000001) + lineL[2];
            int xR = (y - lineR[3])*(lineR[0])/(lineR[1]+0.0000000001) + lineR[2];
            int absX = abs(xL - xR);
            meanXTemp += (xL + absX*0.5);
            widthTemp += absX;
        }
        if (minNum > 0)
        {
            *meanX = meanXTemp/minNum;
            *width = widthTemp/minNum;
        }
        if((*width >= laneWidthThreshold) && (*width < history3.cols/3) && (*meanX >= laneWidthThreshold/2) && (*meanX + 1.5*(*width) < history3.cols)) // calcualte the width successfully
        {
            successFlag = true;
        }
    }
    return successFlag;
}
bool neighborLaneDetect(Mat &allLanes,float width,float meanX,
                                 vector<gpsInformationAndInterval> &GPSAndInterval,
                                 Parameters& inParam,vector<dataEveryRow> &roadPaintData)
{
    // initialize the vector point buffer
     vector<dataEveryRow> middleRoadPaintData;
	for(int i = 0;i < allLanes.rows;i++)
	{
		dataEveryRow newData;

		newData.leftPoint = Point(-1, -1);
		newData.Left_Middle_RelGPS = Point2d(0.0, 0.0);
		newData.isPaint_Left = -1;
		newData.left_Edge_XY[0] = Point(-1, -1);
		newData.left_Edge_XY[1] = Point(-1, -1);
		newData.Left_Paint_Edge[0] = Point2d(0.0, 0.0);
		newData.Left_Paint_Edge[1] = Point2d(0.0, 0.0);
		newData.Left_Area_Pixel_Mean = -1.0;

		newData.Middle_RelGPS = Point2d(0.0, 0.0);
		newData.Middle_Area_Pixel_Mean = -1.0;

		newData.rightPoint = Point(-1, -1);
		newData.Right_Middle_RelGPS = Point2d(0.0, 0.0);
		newData.isPaint_Right = -1;
		newData.right_Edge_XY[0] = Point(-1, -1);
		newData.right_Edge_XY[1] = Point(-1, -1);
		newData.Right_Paint_Edge[0] = Point2d(0.0, 0.0);
		newData.Right_Paint_Edge[1] = Point2d(0.0, 0.0);
		newData.Right_Area_Pixel_Mean = -1.0;

		middleRoadPaintData.push_back(newData);
	}   
    
    // remove the left black area
    Mat leftGray;
    cvtColor(allLanes, leftGray, COLOR_RGB2GRAY);
    int blackWidthL = 0;
    int blackWidthR = leftGray.cols;
    for(int rowIdx = 0; rowIdx < leftGray.rows; rowIdx++)
    {
        for(int colIdx = 0; colIdx < leftGray.cols; colIdx++)
        {
            if (leftGray.at<uchar>(rowIdx,colIdx) > 0)
            { 
                blackWidthL = colIdx + 2;
                break;
            }
        }
    }
    // remove the right black area
    for(int rowIdx = 0; rowIdx < leftGray.rows; rowIdx++)
   {
        for(int colIdx = leftGray.cols - 1; colIdx >= 0; colIdx--)
        {
            if (leftGray.at<uchar>(rowIdx,colIdx) > 0)
            { 
                blackWidthR = colIdx - 2;
                break;
            }
        }
    }
    Mat allImageRgb = allLanes(Rect(blackWidthL, 0, blackWidthR-blackWidthL, allLanes.rows));

#if 1
    // extract the left lane and right lane image
    int widthInt = (int)(meanX - blackWidthL);
    int colNumL = widthInt;
    int  startColR = widthInt;
    int startColL = 0;
    int colNumR = 1.5*width;
    
   
    Mat allImageGray;
    cvtColor(allImageRgb, allImageGray, COLOR_RGB2GRAY);
	//for test
    if ((startColR < 0) || (startColR + colNumR >= allImageGray.cols) || (startColL < 0) || (startColL + colNumL >= allImageGray.cols))
    {
        printf("error ROI: wide = %f, mean = %f,startColR = %d,startColL = %d,colNumR = %d,colNumL = %d",width,meanX,startColR,startColL,colNumR,colNumL);
		return false;
    }
    //end


    Mat leftLaneImage = allImageGray(Rect(startColL,0,colNumL,allImageGray.rows));
    Mat rightLaneImage = allImageGray(Rect(startColR,0,colNumR,allImageGray.rows));

#ifdef ROAD_SCAN_UT    
    imwrite("testL.png",leftLaneImage);
    imwrite("testR.png",rightLaneImage);
#endif

    // detect left lane
    vector<dataEveryRow> roadPaintDataL;
    vector<landMark> vecLandMarkL;
    Mat DrawMarkerL = Mat::zeros(leftLaneImage.rows, leftLaneImage.cols, CV_8UC3);  
    roadImageProc2(leftLaneImage, inParam, GPSAndInterval, roadPaintDataL, vecLandMarkL, DrawMarkerL);

#ifdef ROAD_SCAN_UT    
    imshow("testL",DrawMarkerL);
    waitKey(1);

#endif
    // detect right lane
    vector<dataEveryRow> roadPaintDataR;
    vector<landMark> vecLandMarkR;
    Mat DrawMarkerR = Mat::zeros(rightLaneImage.rows, rightLaneImage.cols, CV_8UC3); 
 //   rightLaneFlag = true;
    roadImageProc2(rightLaneImage, inParam, GPSAndInterval, roadPaintDataR, vecLandMarkR, DrawMarkerR);
 //   rightLaneFlag = false;
#ifdef ROAD_SCAN_UT    
    imshow("testR",DrawMarkerR);
    waitKey(1);
#endif

    // loop up for left and right lines of current lane
    vector<Point> curLeftLine;

    for(int rowIdx = 0; rowIdx < DrawMarkerL.rows; rowIdx++ )
    {
        for(int colIdx = 0; colIdx < DrawMarkerL.cols*0.5; colIdx++)
        {
            Vec3b colorC3 = DrawMarkerL.at<Vec3b>(rowIdx,colIdx);
                
            if ((colorC3[2] > 0) && (colorC3[1] == 0) && (colorC3[0] == 0))
            {               
                middleRoadPaintData[rowIdx].leftPoint.x = colIdx+startColL+blackWidthL;
                middleRoadPaintData[rowIdx].leftPoint.y = rowIdx;
                middleRoadPaintData[rowIdx].isPaint_Left = 1;
                circle(allLanes, Point(colIdx+startColL+blackWidthL,rowIdx),0,CV_RGB(255, 0, 255),4); // left lane
                break;
            }
        }
    }

    for(int rowIdx = 0; rowIdx < DrawMarkerR.rows; rowIdx++ )
    {
        for(int colIdx = DrawMarkerR.cols*0.5; colIdx < DrawMarkerR.cols; colIdx++)
        {
            Vec3b colorC3 = DrawMarkerR.at<Vec3b>(rowIdx,colIdx);
                
            if ((colorC3[0] > 0) && (colorC3[1] == 0) && (colorC3[2] == 0))
            { 
                middleRoadPaintData[rowIdx].rightPoint.x = colIdx+startColR+blackWidthL;
                middleRoadPaintData[rowIdx].rightPoint.y = rowIdx;
                middleRoadPaintData[rowIdx].isPaint_Right = 1;
                circle(allLanes, Point(colIdx+startColR+blackWidthL,rowIdx),0,CV_RGB(0, 255, 255),4); // right lane
                break;
            }
        }
    }

	//store the left and right lane information of current lane to report
	int startPoint = 0;
    int centerOfSertch = allLanes.cols * 0.5 - 1;
	for(int ii = 0;ii < GPSAndInterval.size();ii++)
	{
		int endPoint = startPoint + GPSAndInterval[ii].intervalOfInterception;

        // calculate the relative GPS 
        Point2d GPS_1,GPS_2;
        coordinateChange(GPSAndInterval[ii].GPS_now,inParam.GPSref,GPS_1);
        coordinateChange(GPSAndInterval[ii].GPS_next,inParam.GPSref,GPS_2);

        // calculate the rotation angle of the vehicle
        double angle = atan2(GPS_2.y-GPS_1.y, GPS_1.x-GPS_1.x);
        double t = PI*1.5 + angle;

		if(endPoint > allImageRgb.rows)
			break;

		for(int jj = startPoint;jj < endPoint;jj++)
		{
           
             Point2d pointerL = Point2d(0.0, jj-startPoint);
   
            if(middleRoadPaintData[jj].leftPoint.x != -1)
            {
                pointerL = Point2d(middleRoadPaintData[jj].leftPoint.x-centerOfSertch, jj-startPoint);
            }

            // change the vehicle coordinate to relative GPS
            // coordinate system 2
            double x1 = -1.0 * pointerL.x * inParam.distancePerPixelX / 100;
            double y1 = pointerL.y * inParam.distancePerPixelY / 100;

            // coordinate system 1
            middleRoadPaintData[jj].Left_Middle_RelGPS.x = x1 * cos(t) - y1 * sin(t) + GPS_1.x;
            middleRoadPaintData[jj].Left_Middle_RelGPS.y = x1 * sin(t) + y1 * cos(t) + GPS_1.y;

            Point2d pointerR = Point2d(0.0, jj-startPoint);
            if(middleRoadPaintData[jj].rightPoint.x != -1)
            {
                pointerR = Point2d(middleRoadPaintData[jj].rightPoint.x-centerOfSertch, jj-startPoint);
            }
            // change the vehicle coordinate to relative GPS
	        double x2 = -1.0 * pointerR.x * inParam.distancePerPixelX / 100;
	        double y2 = pointerR.y * inParam.distancePerPixelY / 100;

	        // coordinate system 1
	        middleRoadPaintData[jj].Right_Middle_RelGPS.x = x2 * cos(t) - y2 * sin(t) + GPS_1.x;
	        middleRoadPaintData[jj].Right_Middle_RelGPS.y = x2 * sin(t) + y2 * cos(t) + GPS_1.y;
		}

		startPoint = endPoint;

	}
    //allImageRgb.copyTo(detectedAllLanes);
    
    // down sample

	for(int index = 0; index < allImageRgb.rows; index++)
	{
		if(index%inParam.downSampling == 0)
		{
			roadPaintData.push_back(middleRoadPaintData[index]);
		}
	}

#ifdef ROAD_SCAN_UT
    imwrite("leftLane.png",DrawMarkerL);
    imwrite("rightLane.png",DrawMarkerR);

    static uint32 pIndex = 0;
    stringstream index;
    string temp;
    index << pIndex;
    pIndex++;
    index >> temp;
    string imageName = "allLanes" + temp + ".png";
    imwrite(imageName,allLanes);
#endif
#else
    // extract the left lane and right lane image
    int widthInt = (int)(meanX - blackWidthL);
    int colNumL = widthInt - 0.65*width;
    int  startColR = widthInt + 0.65*width;
    int startColL = 0;
    int colNumR = 2*width;

    Mat allImageGray;
    cvtColor(allImageRgb, allImageGray, COLOR_RGB2GRAY);
    Mat leftLaneImage = allImageGray(Rect(startColL,0,colNumL,allImageGray.rows));
    Mat rightLaneImage = allImageGray(Rect(startColR,0,colNumR,allImageGray.rows));

    Mat laneImage = Mat::zeros(leftLaneImage.rows+rightLaneImage.rows,leftLaneImage.cols+rightLaneImage.cols,CV_8UC1);

    Mat roiLeft = laneImage(Rect(0,0,leftLaneImage.cols,leftLaneImage.rows));
    Mat roiRight = laneImage(Rect(leftLaneImage.cols,0,rightLaneImage.cols,rightLaneImage.rows));

    leftLaneImage.copyTo(roiLeft);
    rightLaneImage.copyTo(roiRight);


#ifdef ROAD_SCAN_UT    
    imwrite("mergedLanes.png",laneImage);
    waitKey(1);
#endif


    vector<dataEveryRow> roadPaintData;
    vector<landMark> vecLandMark;
    Mat DrawMarker = Mat::zeros(laneImage.rows, laneImage.cols, CV_8UC3);  
    roadImageProc2(laneImage, inParam, GPSAndInterval, roadPaintData, vecLandMark, DrawMarker);
  // loop up for left and right lines of current lane
    vector<Point> curLeftLine;
    for(int rowIdx = 0; rowIdx < DrawMarker.rows; rowIdx++ )
    {
        for(int colIdx = 0; colIdx < DrawMarker.cols; colIdx++)
        {
            Vec3b colorC3 = DrawMarker.at<Vec3b>(rowIdx,colIdx);
                
            if (colorC3[2] > 0)
            { 
                circle(allImageRgb, Point(colIdx+startColL,rowIdx),0,CV_RGB(255, 0, 255),4); // left lane
            }
            else if (colorC3[0] > 0)
            {
                circle(allImageRgb, Point(colIdx+startColR-colNumL,rowIdx),0,CV_RGB(0, 255, 255),4); // right lane
            }
        }
    }
#ifdef ROAD_SCAN_UT    
    imshow("allLanes",allImageRgb);
    waitKey(1);
#endif
    imwrite("allLanes.png",allImageRgb);
#endif
	return true;

}


}
