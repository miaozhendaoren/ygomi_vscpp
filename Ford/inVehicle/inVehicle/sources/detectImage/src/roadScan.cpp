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

int roadImageGen(Mat imageIn, Mat &history, int *rowIndex, Point2d *GPS_abs, Point2d *GPS_next, gpsInformationAndInterval *gpsAndInterval, int *intrtmp,Parameters inParam)
{
    //laneInfo currlane;												//XYBLOCK
    ///////////////////////////////////////////////////////////////////////////////////////////
	//Point2d	ref = Point2d(42.29982494686108,-83.21984649411516);

	Point2d *GPS_ref = &inParam.GPSref;

	Mat image, image_32f, ColorImage;
	if(imageIn.empty())
	{
		return -1;
	}

	Size Rsize = Size(imageIn.cols*inParam.imageScaleWidth, imageIn.rows*inParam.imageScaleHeight);
	//Size Rsize = Size(640,400);//for honda data
    
//	image = Mat(Rsize,CV_8UC1);
    cvtColor(imageIn, image, COLOR_RGB2GRAY);
//	resize(imageIn, image,Rsize);

	medianBlur(image,image,3) ;

    ColorImage = image;
//    cvtColor(image, image, COLOR_RGB2GRAY);
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
    Mat image_f,Kapa;
    image.convertTo(image_f,CV_32F);	

	Point curr = Point(0,0);
	//Size S = Size(640, 480);

//	Mat MAP = Mat::zeros(600, 1000, CV_8UC3);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //Step5: Generate BirdView
    Mat birds_image;

    double scale = 1;

	//startLocation = (objPts[2].y-objPts[0].y)/(imgPts[2].y-imgPts[0].y)/inParam.lengthRate*h+(inParam.stretchRate-1)*h-10;
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
    Mat RImage;

	//imwrite("birds_image.png",birds_image);

    Mat birdEye;
    birdEye = birds_image;
//    cvtColor(birds_image, birdEye, COLOR_RGB2GRAY);
//    birdEye.convertTo(birdEye,CV_8UC1,1);
    
	Point2d GPS_rel, GPS_rel2;

	coordinateChange(*GPS_abs, *GPS_ref, GPS_rel);
	coordinateChange(*GPS_next, *GPS_ref, GPS_rel2);

	double d = sqrt((GPS_rel.x-GPS_rel2.x)*(GPS_rel.x-GPS_rel2.x)+(GPS_rel.y-GPS_rel2.y)*(GPS_rel.y-GPS_rel2.y));
	int Interval =floor(d/(inParam.distancePerPixel/100)+0.5);
   
    //double roadScanStretchRate = 1;
	//if(dist>=1)
	{
		*rowIndex = *rowIndex - *intrtmp;

		if ((*rowIndex)-Interval <=0)
			(*rowIndex)=history.rows;

		//Mat imageROI = birdEye(Rect(0,Interval/2,birdEye.cols,Interval));
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

		//should not line when judge the paint of bird view
		//line(birdEye, Point(0,floor(startLocation)), Point(w,floor(startLocation)), Scalar(255), 1, CV_AA);

		int xx = 0;
		int yy = (*rowIndex);
		yy = min(yy,HH*SCALE*h-2*h-1);
				
		Rect rect = Rect(xx, yy, history.cols, h*2);
		cv::namedWindow("Road Scan",CV_WINDOW_NORMAL);
		imshow("Road Scan",history(rect));			
	}

	*intrtmp = Interval;

	//(*rowIndex) = (*rowIndex)-Interval;
#ifdef ROAD_SCAN_UT
	imshow( "Birds View", birdEye );
#endif
	waitKey(1);

	return 0;
}


void roadImageProc2(Mat longLane, vector<gpsInformationAndInterval> &gpsAndInterval, vector<dataEveryRow> &roadPaintData, Parameters& inParam)
{

#define T1  30
#define T2  200

#ifdef ROAD_SCAN_UT
    double startT = static_cast<double>(cv::getTickCount());
#endif
//    Mat longLaneRGB = longLane;
//    Mat longLaneHSV;    
    //cvtColor(longLaneRGB,longLaneHSV,COLOR_BGR2HSV_FULL);
    //longLaneRGB.convertTo(longLaneHSV,COLOR_BGR2HSV);
    //cvtColor(longLane,longLane,COLOR_RGB2GRAY);

//    Mat longLaneGray;
//    cvtRGB2Gray(longLaneRGB, longLaneGray);

    shadowProcess(longLane, longLane);
    
#ifdef ROAD_SCAN_UT
    imwrite("longLaneGray.png",longLaneGray);
    imwrite("longLane.png",longLane);
#endif
	vector<landMark> landMark;
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

//		cout<<"n="<<n_cut<<"kk="<<kk<<endl;
		if (kk==n_cut)
			longLane_cut = longLane(Rect(0,(kk-1)*CUT,longLane.cols,longLane.rows-(kk-1)*CUT));
		else
			longLane_cut = longLane(Rect(0,(kk-1)*CUT,longLane.cols,CUT));

		int w = longLane_cut.cols;
		int h = longLane_cut.rows;

		Mat longLaneLeft = longLane_cut(Rect(0,0,longLane_cut.cols/2,longLane_cut.rows));
		Mat longLaneRight = longLane_cut(Rect(longLane_cut.cols/2,0,longLane_cut.cols/2,longLane_cut.rows));
		int size = 256;
		Mat dstImage1(size,size,CV_8UC1,Scalar(0));
		Mat dstImage2(size,size,CV_8UC1,Scalar(0));
		int hisLeftTh,hisRightTh;
		calThread(longLaneLeft,hisLeftTh,dstImage1);
		calThread(longLaneRight,hisRightTh,dstImage2);

		int histThres=0;
		if (hisLeftTh==0||hisRightTh==0)
			histThres = hisRightTh+hisLeftTh;
		else
			histThres = (hisLeftTh + hisRightTh)/2;
		histThres = max(hisLeftTh,hisRightTh);
	
#if 0
//		circleDection(longLane_cut,landMark,histThres);	
		landMarkDetection(longLane_cut,landMark,histThres+20);
		//arrowDetection(longLane,landMark);
		//stopLineDetection(longLane_cut,landMark,histThres);
#endif
        
///////shadow and no shadow ,process respectively
		Mat allUnitKapa(h,w,CV_8UC1);
		allUnitKapa.convertTo(allUnitKapa,CV_8UC1,1024); 
	    Mat allUnitContous(h,w,CV_8UC1);
		Mat allKappBin;

        blockCalRidge(longLane_cut,inParam,allUnitKapa,allUnitContous,allKappBin);

		linkInterval(allUnitContous,allUnitContous);
		threshold( allUnitContous, allUnitContous,5, 255,0 );

#ifdef DEBUG_ROAD_SCAN
		imwrite("allUnitContous2.png",allUnitContous);
#endif
       
        Mat lineLinkOut;
		linkPaintLine(allUnitContous,landMark,lineLinkOut);

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
//                        int hue = longLaneHSV.at<Vec3b>(i,left)[0];
//                        int v   = longLaneHSV.at<Vec3b>(i,left)[2];
						int dataMiddle = allUnitContous.at<uchar>(i,left);
						if(dataMiddle!=0)
						{
							
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].leftPoint.x = left;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].leftPoint.y = i+(kk-1)*CUT;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].isPaint_Left = 1;

//							calPaintEdgePos(kapa2, i, left, middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].left_Edge_XY[0], middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].left_Edge_XY[1]);                           
							circle(roadDraw3, Point(left,i),0,CV_RGB(255, 0, 0),4);
						}
					}

					if (right< w1) 
					{
//                        int hue = longLaneHSV.at<Vec3b>(i,right)[0];
						int dataMiddle = allUnitContous.at<uchar>(i,right);
						if(dataMiddle!=0)
						{
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].rightPoint.x = right;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].rightPoint.y = i+(kk-1)*CUT;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].isPaint_Right = 1;

//							calPaintEdgePos(kapa2, i, right, middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].right_Edge_XY[0], middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].right_Edge_XY[1]);

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
	int changelinePoint = -1;

	for(int i = 0; i<changeLP.size(); i++)
	{
		if((changeLP[i]+thresholdCL)<H)
		{
			if(middleRoadPaintData[changeLP[i]+thresholdCL].leftPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]+thresholdCL].leftPoint.x-W_half)<50)
				{
					changelinePoint = changeLP[i];
					break;
				}
			}

			if(middleRoadPaintData[changeLP[i]+thresholdCL].rightPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]+thresholdCL].rightPoint.x-W_half)<50)
				{
					changelinePoint = changeLP[i];
					break;
				}
			}
		}

		if((changeLP[i]-thresholdCL)>=0)
		{
			if(middleRoadPaintData[changeLP[i]-thresholdCL].leftPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]-thresholdCL].leftPoint.x-W_half)<50)
				{
					changelinePoint = changeLP[i];
					break;
				}
			}

			if(middleRoadPaintData[changeLP[i]-thresholdCL].rightPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]-thresholdCL].rightPoint.x-W_half)<50)
				{
					changelinePoint = changeLP[i];
					break;
				}
			}
		}
	}

	if(changelinePoint!=-1)
	{
		//line change
		int sP = changelinePoint-300;
		int eP = changelinePoint+300;

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
	
	if (landMark.size())
	{
		for (int i=0;i<landMark.size();i++)
		{
			findGPSInterval(gpsAndInterval,landMark[i].landMarkWeight,longLane,inParam,landMark[i].landMarkWeightRel);
		//	cout<<landMark[i].landMarkWeight.x<<" ,"<<landMark[i].landMarkWeight.y<<endl;
		//	cout<<landMark[i].landMarkWeightRel.x<<" ,"<<landMark[i].landMarkWeightRel.y<<endl;

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
