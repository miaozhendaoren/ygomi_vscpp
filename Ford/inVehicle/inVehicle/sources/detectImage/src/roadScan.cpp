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

int roadImageGen(Mat imageIn, Mat &history, int *rowIndex, Point2d *GPS_abs, Point2d *GPS_next, 
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
    imshow("iniImage",image);
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
	startLocation = 710;		
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
        if (d < 0.10)
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
        if (d > 0.10)
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
	
	int Interval =floor(d/(inParam.distancePerPixel/100)+0.5);
  
	//if(dist>=1)
	{
		*rowIndex = *rowIndex - *intrtmp;

		if ((*rowIndex)-Interval <=0)
			(*rowIndex)=history.rows;
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
		int yy = (*rowIndex);
		yy = min(yy,history.rows-2*h-1);
				
		Rect rect = Rect(xx, yy, history.cols, h*2);
		cv::namedWindow("Road Scan",CV_WINDOW_NORMAL);
		imshow("Road Scan",history(rect));			
	}

	*intrtmp = Interval;
#ifdef ROAD_SCAN_UT
	imshow( "Birds View", birdEye );
#endif
	waitKey(1);

	return 0;
}
void roadImageProc2(Mat longLane, Parameters &inParam, vector<gpsInformationAndInterval> &gpsAndInterval, vector<dataEveryRow> &roadPaintData, 
    vector<landMark> &vecLandMark)
{

//    Mat RGBImg;
//   cvtColor(longLane,RGBImg,COLOR_GRAY2RGB);
#ifdef ROAD_SCAN_UT
    double startT = static_cast<double>(cv::getTickCount());
#endif
	int W = longLane.cols;
	int H = longLane.rows;

    int W_half = W * 0.5;

	vector<int> changeLP;
	vector<dataEveryRow> middleRoadPaintData;

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
	
    Mat DrawMarker = Mat::zeros(longLane.rows,longLane.cols,CV_8UC3);
    Mat PaintLine = Mat::zeros(longLane.rows,longLane.cols,CV_8UC1);
	Mat longLane_cut;

	int n_cut;
	n_cut = ceil(longLane.rows/CUT);
	for (int kk=1;kk<=n_cut;kk++)
	{
		if (kk==n_cut)
			longLane_cut = longLane(Rect(0,(kk-1)*CUT,longLane.cols,longLane.rows-(kk-1)*CUT));
		else
			longLane_cut = longLane(Rect(0,(kk-1)*CUT,longLane.cols,CUT));

        Mat longLane_cut2;

        shadowProcess(longLane_cut, longLane_cut2);
		int w = longLane_cut.cols;
		int h = longLane_cut.rows;

#if((RD_ROAD_SIGN_DETECT_STOPLINE_MASK & RD_ROAD_SIGN_DETECT) )

        landMark landmark1;
        vector<Point> stopLineLoc;
        stopLineDetection(longLane_cut2,stopLineLoc);
        for (int i=0;i<stopLineLoc.size();i++)
        {
            stopLineLoc[i].y =((kk-1)*CUT + stopLineLoc[i].y);
            landmark1.center = stopLineLoc[i];
            landmark1.type = 1000;
            landmark1.flag = 4;
            vecLandMark.push_back(landmark1);
//            circle(RGBImg,landmark1.center,0,Scalar(0,0,255),6);
        }
#endif
         
#ifdef DEBUG_ROAD_SCAN
//         imwrite("RGBImg.png",RGBImg);
#endif
	
#if((RD_ROAD_SIGN_DETECT_ARROW_MASK & RD_ROAD_SIGN_DETECT) )
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
//		circleDection(longLane_cut,landMark,histThres);
        vector<landMark> arrows;
        Mat longLane_cutBW;
        gray2BW(longLane_cut,longLane_cutBW);
#ifdef DEBUG_ROAD_SCAN
        imwrite("longLane_cutBW.png",longLane_cutBW);
#endif
		arrowDetection(longLane_cut,longLane_cutBW,arrows,longLane_cut2);

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
#endif
        
///////shadow and no shadow ,process respectively
		Mat allUnitKapa(h,w,CV_8UC1);
		allUnitKapa.convertTo(allUnitKapa,CV_8UC1,1024); 
	    Mat allUnitContous(h,w,CV_8UC1);
		Mat allKappBin;

        blockCalRidge(longLane_cut2,inParam,allUnitKapa,allUnitContous,allKappBin);

		linkInterval(allUnitContous,allUnitContous);
		threshold( allUnitContous, allUnitContous,5, 255,0 );

#ifdef DEBUG_ROAD_SCAN
		imwrite("allUnitContous2.png",allUnitContous);
#endif
       
        Mat lineLinkOut;
		linkPaintLine(allUnitContous,lineLinkOut);

		//step3: find the left and right lane.
		int w1 = allUnitContous.cols;
		int h1 = allUnitContous.rows;

		Mat roadDraw3;	
		cvtColor(lineLinkOut, roadDraw3, COLOR_GRAY2RGB);
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
        }else
        {
            //cout<<"Undefined parameter"<<endl;
        }
	}

	return true;
}

}
