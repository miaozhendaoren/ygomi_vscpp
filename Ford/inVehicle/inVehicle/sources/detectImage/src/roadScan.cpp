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
//#include "roadScan.h"
#include <math.h>
#include "ImageBuffer.h"
#include "utils.h"
#include <iomanip>
#include <fstream>
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
		return 0;
	}

	if(imageIn.cols != 640)
	{
		Size Rsize = Size(imageIn.cols*0.5,imageIn.rows*0.5);
		//Size Rsize = Size(640,400);//for honda data
		image = Mat(Rsize,CV_8UC3);
		resize(imageIn, image,Rsize);
	}
	else
	{
		image = imageIn;
	}		

	medianBlur(image,image,3) ;

    ColorImage = image;
    cvtColor(image, image, COLOR_RGB2GRAY);
    /*
	image.convertTo(image_32f,CV_32F);
	processNoise(image_32f,image);
	image.convertTo(ColorImage,CV_8UC1);
	cvtColor(ColorImage, ColorImage, COLOR_GRAY2RGB);
    */
    imshow("iniImage",image);			

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
    Mat H;
    if(1)
    {
        Mat temp_frame = image_f.clone();

        Point A = Point(0,image.rows-1);
        Point B = Point(image.cols-1 ,image.rows-1);

        Point2f objPts[4], imgPts[4];

		imgPts[0].x=(inParam.centerPoint.x+A.x)/2-inParam.distanceOfSlantLeft;
		imgPts[0].y=(inParam.centerPoint.y+A.y)/2;

		imgPts[1].x= (inParam.centerPoint.x+B.x)/2+inParam.distanceOfSlantRight;
		imgPts[1].y= (inParam.centerPoint.y+B.y)/2;

		imgPts[2].x= A.x;
		imgPts[2].y= A.y;

		imgPts[3].x= B.x;
		imgPts[3].y= B.y;

        double scale = 1;
        double xoff = 2*w/10;
        double hh = scale*(A.y -imgPts[0].y)/2;

        objPts[0].x = 6*scale*A.x/10+xoff-inParam.distanceOfLeft;
		objPts[0].y = A.y*scale+inParam.distanceOfUpMove;

        objPts[1].x = 6*scale*B.x/10+xoff+inParam.distanceOfRight;
		objPts[1].y = A.y*scale+inParam.distanceOfUpMove;

        objPts[2].x = 6*scale*A.x/10+xoff-inParam.distanceOfLeft;
		objPts[2].y = scale*A.y+hh*(inParam.lengthRate+1)+inParam.distanceOfUpMove;

        objPts[3].x = 6*scale*B.x/10+xoff+inParam.distanceOfRight;
		objPts[3].y = scale*B.y+hh*(inParam.lengthRate+1)+inParam.distanceOfUpMove;

		//startLocation = (objPts[2].y-objPts[0].y)/(imgPts[2].y-imgPts[0].y)/inParam.lengthRate*h+(inParam.stretchRate-1)*h-10;
		startLocation = 710;
        H = getPerspectiveTransform( objPts, imgPts);			

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
    }

    Mat birdEye;

    cvtColor(birds_image, birdEye, COLOR_RGB2GRAY);
    birdEye.convertTo(birdEye,CV_8UC1,1);
    
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
		Mat imageROI = birdEye(Rect(0,floor(startLocation)-Interval+1,birdEye.cols,Interval));

		//Interval = ceil(Interval/inParam.downSampling);

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
				
		Rect rect = Rect(xx, yy, IMAGE_SENSOR_WIDTH, h*2);
		imshow("Road Scan",history(rect));			
	}

	*intrtmp = Interval;

//	(*rowIndex) = (*rowIndex)-Interval;

	imshow( "Birds View", birdEye );
	waitKey(1);
}


void roadImageProc2(Mat longLane, vector<gpsInformationAndInterval> &gpsAndInterval, vector<dataEveryRow> &roadPaintData, Parameters& inParam)
{

#define T1  30
#define T2  200
	//Mat longLane1;
	//longLane.copyTo(longLane1);
	//Mat I2(longLane.rows,longLane.cols,CV_8UC1);
	//shadowProcess(longLane, I2);

	//bool isShadow = judgeShadow(longLane);
	//

	int W = longLane.cols;
	int H = longLane.rows;

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
	
	Mat longLane_cut;
	int n_cut;
	n_cut = ceil(longLane.rows/CUT);
	Mat DrawMarker = Mat::zeros(longLane.rows,longLane.cols,CV_8UC3);
	Mat PaintLine = Mat::zeros(longLane.rows,longLane.cols,CV_8UC1);

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
		//	imshow("hst1",dstImage1);
		//	imshow("hst2",dstImage2);
		//	waitKey(1);
		//	hisLeftTh = 0;
		//	hisRightTh = 0;
		int hisTH=0;//=0,for VW and Ford
		cout<<"hisTH="<<hisTH<<"left="<<hisLeftTh<<"right="<<hisRightTh<<endl;

		int histThres=0;
		if (hisLeftTh==0||hisRightTh==0)
			histThres = hisRightTh+hisLeftTh;
		else
			histThres = (hisLeftTh + hisRightTh)/2;
		histThres = max(hisLeftTh,hisRightTh);
		cout<<histThres<<endl;
		Mat TH,leftTH,rightTH;
		threshold(longLane_cut,TH,histThres,255,0);
		threshold(longLaneLeft,leftTH,hisLeftTh,255,0);
		threshold(longLaneRight,rightTH,hisRightTh,255,0);
		hisLeftTh = 20;
		hisRightTh = 20;

		//imwrite("thres.png",TH);
		int ridgePar = calRidgePar(TH);
		int ridgeLeftPar = calRidgePar(leftTH);
		int ridgeRightPar = calRidgePar(rightTH);
		if (ridgeLeftPar<9)
		{
			ridgeLeftPar=9;
		}
		if (ridgeRightPar<9)
		{
			ridgeRightPar=9;
		}
//		cout<<"ridgePar="<<ridgePar<<"ridgeLeftPar="<<ridgeLeftPar<<"ridgeRightPar="<<ridgeRightPar<<endl;
		vector<laneMarker> lanemarker;
		//	laneMarker lanemarker;

	
		//	circleDection(longLane_cut,lanemarker,histThres);	
		numberDetection(longLane_cut,lanemarker,histThres+20);
		//arrowDetection(longLane,lanemarker);
		//stopLineDetection(longLane_cut,lanemarker,histThres);

///////shadow and no shadow ,process respectively
		int unitH=700;
		int unitNum = ceil(1.0*h/unitH);
		Mat allUnitKapa(h,w,CV_8UC1);
		allUnitKapa.convertTo(allUnitKapa,CV_8UC1,1024); 
	    Mat allUnitContous(h,w,CV_8UC1);
		Mat allKappBin;
		for (int uniti=1;uniti<=unitNum;uniti++)
		{
			//cout<<unitNum<<","<<uniti<<endl;
			Mat unit;
			if (uniti==unitNum)
				unit = longLane_cut(Rect(0,(uniti-1)*unitH,longLane_cut.cols,longLane_cut.rows-(uniti-1)*unitH));
			else
				unit = longLane_cut(Rect(0,(uniti-1)*unitH,longLane_cut.cols,unitH));
			bool isShadow = judgeShadow(unit);
			if (isShadow)
			{
				shadowProcess(unit, unit);
			}			
			Mat unitlongLaneLeft = unit(Rect(0,0,unit.cols/2,unit.rows));
			Mat unitlongLaneRight = unit(Rect(unit.cols/2,0,unit.cols/2,unit.rows));
			int size = 256;
			Mat dstImage1(size,size,CV_8UC1,Scalar(0));
			Mat dstImage2(size,size,CV_8UC1,Scalar(0));
			int unithisLeftTh,unithisRightTh;
			calThread(unitlongLaneLeft,unithisLeftTh,dstImage1);
			calThread(unitlongLaneRight,unithisRightTh,dstImage2);

			Mat unitTH,unitleftTH,unitrightTH;
			threshold(longLaneLeft,unitleftTH,unithisLeftTh,255,0);
			threshold(unitlongLaneRight,unitrightTH,unithisRightTh,255,0);
			int unitridgeLeftPar = calRidgePar(unitleftTH);
			int unitridgeRightPar = calRidgePar(unitrightTH);

			//cout<<"ridgePar="<<ridgePar<<"unitridgeLeftPar="
			//	<<unitridgeLeftPar<<"unitridgeRightPar="<<unitridgeRightPar<<endl;

			int unithistThres=0;
			if (unithisLeftTh==0||unithisRightTh==0)
				unithistThres = unithisRightTh+unithisLeftTh;
			else
				unithistThres = (unithisLeftTh + unithisRightTh)/2;
			unithistThres = max(unithisLeftTh,unithisRightTh);



			Mat unitimageLeft_32f,unitkapaLeft,unitimageRight_32f,unitkapaRight;
			Mat unitLeft = unit(Rect(0,0,unit.cols/2,unit.rows));
			Mat unitRight = unit(Rect(unit.cols/2,0,unit.cols/2,unit.rows));
			unitLeft.convertTo(unitimageLeft_32f,CV_32F);
			unitRight.convertTo(unitimageRight_32f,CV_32F);

			if (unitridgeLeftPar<9)
			{
				unitridgeLeftPar=9;
			}
			if (unitridgeRightPar<9)
			{
				unitridgeRightPar=9;
			}
			ridgeDetect(unitimageLeft_32f,unitkapaLeft,unitridgeLeftPar/3.0,unitridgeLeftPar/3.0);
			ridgeDetect(unitimageRight_32f,unitkapaRight,unitridgeRightPar/3.0,unitridgeRightPar/3.0);
			unitkapaLeft.convertTo(unitkapaLeft,CV_8UC1,1024); 
			unitkapaRight.convertTo(unitkapaRight,CV_8UC1,1024); 

			Mat unitKapa(unit.size(),CV_8UC1);
			Mat unitkapaRoiLeft = unitKapa(Rect(0,0,unit.cols/2,unit.rows));
			Mat unitkapaRoiRight = unitKapa(Rect(unit.cols/2,0,unit.cols/2,unit.rows));
			unitkapaLeft.copyTo(unitkapaRoiLeft);
			unitkapaRight.copyTo(unitkapaRoiRight);


			/*Mat unitImage_f,unitKapa;
			unit.convertTo(unitImage_f,CV_32F);	
			ridgeDetect(unitImage_f,unitKapa, ridgePar/3.0, ridgePar/3.0);*/
			//unitKapa.convertTo(unitKapa,CV_8UC1,1024); 

			//imwrite( "unitKapa.png", unitKapa );
			Mat unitKapaBin;
			threshold( unitKapa, unitKapaBin,5, 255,0 );

			vector<vector<Point> > unitcontours;
			vector<Vec4i> unithierarchy;
			findContours(unitKapaBin, unitcontours, unithierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

			// draw contours:   
		//	unitKapa = Mat::zeros(unitKapa.size(), CV_8UC1);
			Mat unitTSline  = Mat::zeros(unitKapa.size(), CV_8UC1);
			vector<SLineInfo> vecSlineinfo;//save information of every short line

			for (unsigned int i = 0; i < unitcontours.size(); i++)
			{

				//pixel mean value of contour
				double meanvalue=0;;
				SLineInfo sline;
				Vec4f line;
				fitLine(unitcontours[i],line,CV_DIST_L2 ,0,0.01,0.01);//linear fit for short line contour
				sline.vx = line[0];
				sline.vy = line[1];

				double C = (line[3]*line[0]-line[2]*line[1]);
				vector<double> Dist;
				vector<Point> pointss = unitcontours[i];
				sline.upoint = pointss[0];
				sline.dpoint = pointss[pointss.size()-1];
				sline.WeiPoint = Point(0,0);//line center of gravity

				for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
				{
					if (pointss[j].y<sline.upoint.y)
					{
						sline.upoint = pointss[j];
					}
					if (pointss[j].y>sline.dpoint.y)
					{
						sline.dpoint = pointss[j];
					}
					meanvalue += unit.at<uchar>(pointss[j].y,pointss[j].x);
					sline.WeiPoint.x+=pointss[j].x;
					sline.WeiPoint.y+=pointss[j].y;
					double dst = abs(line[1]*pointss[j].x - line[0]*pointss[j].y +C);
					//			cout<<dst<<endl;
					Dist.push_back(dst);
				}
				meanvalue = meanvalue/pointss.size();
				double dst = sqrt(double((sline.dpoint.y - sline.upoint.y)*(sline.dpoint.y - sline.upoint.y)
					+(sline.dpoint.x - sline.upoint.x)*(sline.dpoint.x - sline.upoint.x)));
				int number = Dist.size();
				double sum =0;
				for (int ii=0;ii<number;ii++)
					sum+=Dist[ii];
				double e=sum/number; 
				double s=0;

				for (int ii=0;ii<number;ii++) 
					s+=(Dist[ii]-e)*(Dist[ii]-e);

				s=sqrt(s/number);

				if (s>3)
				{
					if (dst<300)
					{
						continue;
					}		
				}
				sline.WeiPoint = Point(sline.WeiPoint.x/unitcontours[i].size(),sline.WeiPoint.y/unitcontours[i].size());
				sline.vx = line[0];
				sline.vy = line[1];
				double kk = atan(line[1]/(line[0]+0.000001));
				double angle  = abs(180*kk/PI);
				//	if (sline.WeiPoint.x>w/3&&sline.WeiPoint.x<2*w/3)
				{
					//		continue;
				}
				if (angle<80)
				{
					continue;
				}
				double kk2 = atan((sline.upoint.y - sline.WeiPoint.y)/(sline.upoint.x - sline.WeiPoint.x+0.000000001));//首和重心点斜率；
				double angle2  = abs(180*kk2/PI);
				if (angle2<80)
				{
        			continue;
				}
		//		if (isShadow)
				{
					unithisLeftTh = 20;
					unithisRightTh = 20;
				}
				if (sline.upoint.x<w/2&&meanvalue>unithisLeftTh&&dst>50)
				{
					drawContours(unitTSline, unitcontours, i, Scalar(255), 1, 8, unithierarchy, 0);
				}

				if (sline.upoint.x>=w/2&&meanvalue>unithisRightTh&&dst>50)
				{
					drawContours(unitTSline, unitcontours, i, Scalar(255), 1, 8, unithierarchy, 0);
				}
			}


			if (uniti==unitNum)
			{
				Mat kaparoi = allUnitKapa(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,allUnitKapa.rows-(uniti-1)*unitH));
				unitKapa.copyTo(kaparoi);			
				//imwrite( "allUnitKapa.png", allUnitKapa );
				
				threshold( allUnitKapa, allKappBin,5, 255,0 );
				//imwrite("allKappBin.png",allKappBin);

				Mat contoursroi = allUnitContous(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,allUnitKapa.rows-(uniti-1)*unitH));
				unitTSline.copyTo(contoursroi);
				//imwrite("allUnitContous.png",allUnitContous);

			}
			else
			{
				Mat kaparoi = allUnitKapa(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,unitH));
				unitKapa.copyTo(kaparoi);

				Mat contoursroi = allUnitContous(Rect(0,(uniti-1)*unitH,allUnitKapa.cols,unitH));
				unitTSline.copyTo(contoursroi);
			}
		}
		
		

		Mat image_f,Kapa,T,kapa2;
		allKappBin.copyTo(T);
	
		longLane_cut.convertTo(image_f,CV_32F);	
		ridgeDetect(image_f,kapa2, 2, 2);
		kapa2.convertTo(kapa2,CV_8UC1,1024);  
		//imwrite( "road_kapa2.png", kapa2);		
		threshold( kapa2, kapa2, 5, 255,0 );

	/*	Mat imageLeft_32f,kapaLeft,imageRight_32f,kapaRight;
		longLaneLeft.convertTo(imageLeft_32f,CV_32F);
		longLaneRight.convertTo(imageRight_32f,CV_32F);
	
		ridgeDetect(imageLeft_32f,kapaLeft,ridgeLeftPar/3.0,ridgeLeftPar/3.0);
		ridgeDetect(imageRight_32f,kapaRight,ridgeRightPar/3.0,ridgeRightPar/3.0);
		kapaLeft.convertTo(kapaLeft,CV_8UC1,1024); 
		kapaRight.convertTo(kapaRight,CV_8UC1,1024); 

		Mat kapaALL(longLane_cut.size(),CV_8UC1);
		Mat kapaRoiLeft = kapaALL(Rect(0,0,longLane_cut.cols/2,longLane_cut.rows));
		Mat kapaRoiRight = kapaALL(Rect(longLane_cut.cols/2,0,longLane_cut.cols/2,longLane_cut.rows));
		kapaLeft.copyTo(kapaRoiLeft);
		kapaRight.copyTo(kapaRoiRight);
		imwrite("kapaAll.png",kapaALL);
		threshold( kapaALL, T,5, 255,0 );*/
	

	//	ridgeDetect(image_f,Kapa, ridgePar/3.0, ridgePar/3.0);
	//	Kapa.convertTo(Kapa,CV_8UC1,1024); 

		//if HD 20
	//	threshold( Kapa, T,5, 255,0 );
		//if FD
		//if (ridgePar>20)
		//{
		//	threshold( Kapa, T,5, 255,0 );
		//}
		//else
		//	threshold( Kapa, T,20, 255,0 );
		////if VW 5
		//threshold( Kapa, T,5, 255,0 );		
	//	imwrite( "road_kapa.png", Kapa );
		//imwrite( "road_T.png", T );

		Mat definePaint;
		T.copyTo(definePaint,T);
		//imwrite("definePaint.png",definePaint);

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(T, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

		// draw contours:   

		T = Mat::zeros(T.size(), CV_8UC1);
		Mat TSline  = Mat::zeros(T.size(), CV_8UC1);
		Mat TSlinMark = Mat::zeros(T.size(), CV_8UC3);
		Mat TLline  =Mat::zeros(T.size(), CV_8UC1);
		vector<SLineInfo> vecSlineinfo;//save information of every short line

		for (unsigned int i = 0; i < contours.size(); i++)
		{

			//pixel mean value of contour
			double meanvalue=0;;
			SLineInfo sline;
			Vec4f line;
			fitLine(contours[i],line,CV_DIST_L2 ,0,0.01,0.01);//linear fit for short line contour
			sline.vx = line[0];
			sline.vy = line[1];

			double C = (line[3]*line[0]-line[2]*line[1]);
			vector<double> Dist;
			vector<Point> pointss = contours[i];
			sline.upoint = pointss[0];
			sline.dpoint = pointss[pointss.size()-1];
			sline.WeiPoint = Point(0,0);//line center of gravity
			
			for (int j=0;j<pointss.size();j++)//calculate up and down points of contours
			{
				if (pointss[j].y<sline.upoint.y)
				{
					sline.upoint = pointss[j];
				}
				if (pointss[j].y>sline.dpoint.y)
				{
					sline.dpoint = pointss[j];
				}
				meanvalue += longLane_cut.at<uchar>(pointss[j].y,pointss[j].x);
				sline.WeiPoint.x+=pointss[j].x;
				sline.WeiPoint.y+=pointss[j].y;
				double dst = abs(line[1]*pointss[j].x - line[0]*pointss[j].y +C);
	//			cout<<dst<<endl;
				Dist.push_back(dst);
			}
			sline.WeiPoint = Point(sline.WeiPoint.x/contours[i].size(),sline.WeiPoint.y/contours[i].size());
			bool isdelete = false;
			for(int j=0;j<lanemarker.size();j++)
			{
				if (sline.WeiPoint.x>lanemarker[j].lPoint.x&&sline.WeiPoint.x<lanemarker[j].rPoint.x)
				{
					isdelete = true;
					break;
				}
				
			}
			if (isdelete==true)
			{
				continue;
			}

			meanvalue = meanvalue/pointss.size();
			double dst = sqrt(double((sline.dpoint.y - sline.upoint.y)*(sline.dpoint.y - sline.upoint.y)
				+(sline.dpoint.x - sline.upoint.x)*(sline.dpoint.x - sline.upoint.x)));
			
			int number = Dist.size();
			double sum =0;
			for (int ii=0;ii<number;ii++)
				sum+=Dist[ii];
			double e=sum/number; 
			double s=0;

			for (int ii=0;ii<number;ii++) 
				s+=(Dist[ii]-e)*(Dist[ii]-e);

			s=sqrt(s/number);
			
			if (s>3)
			{
				if (dst<300)
				{
					continue;
				}		
			}
			
			sline.vx = line[0];
			sline.vy = line[1];
			double kk = atan(line[1]/(line[0]+0.000001));
			double angle  = abs(180*kk/PI);
		//	if (sline.WeiPoint.x>w/3&&sline.WeiPoint.x<2*w/3)
			{
		//		continue;
			}

			if (angle<80)
			{
	    		continue;
			}
			double kk2 = atan((sline.upoint.y - sline.WeiPoint.y)/(sline.upoint.x - sline.WeiPoint.x+0.000000001));//首和重心点斜率；
			double angle2  = abs(180*kk2/PI);
			if (angle2<80)
			{
	    		continue;
			}
			
			if (sline.upoint.x<w/2&&meanvalue>hisLeftTh&&dst>100)
			{
				drawContours(TSline, contours, i, Scalar(255), 1, 8, hierarchy, 0);
				drawContours(TSlinMark, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 0);
				vecSlineinfo.push_back(sline);
				circle(TSlinMark,sline.WeiPoint,0,Scalar(255,0,0),4);
				circle(TSlinMark,sline.upoint,0,Scalar(0,255,0),4);
				circle(TSlinMark,sline.dpoint,0,Scalar(0,0,255),4);
			}

			if (sline.upoint.x>=w/2&&meanvalue>hisRightTh&&dst>100)
			{
				drawContours(TSline, contours, i, Scalar(255), 1, 8, hierarchy, 0);
				drawContours(TSlinMark, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 0);
				vecSlineinfo.push_back(sline);
				circle(TSlinMark,sline.WeiPoint,0,Scalar(255,0,0),4);
				circle(TSlinMark,sline.upoint,0,Scalar(0,255,0),4);
				circle(TSlinMark,sline.dpoint,0,Scalar(0,0,255),4);
			}
		//	cout<<s<<endl;
			
		//	if (meanvalue>hisTH&&dst>50)
		////	if (dst>50)
		//	{
		//		drawContours(TSline, contours, i, Scalar(255), 1, 8, hierarchy, 0);
		//	}
		}

		//imwrite( "TSlinMark.png", TSlinMark );
		//imwrite( "TSline.png", TSline );

		/////one up point only links to one down points
		vector<PairPoints> vecPairPoints;//save up and down point pair;
		int *objptr = new int[vecSlineinfo.size()];
		int *srcptr = new int[vecSlineinfo.size()];
		for(int i=0;i<vecSlineinfo.size();i++)
		{
			objptr[i]=0;
			srcptr[i]=0;
		}

		for(int i = 0;i<vecSlineinfo.size();i++)
		{
		///	int ddddd = vecSlineinfo[i].WeiPoint.y;
			for (int j = i+1;j<vecSlineinfo.size();j++)
			{			
				//double dsty = vecSlineinfo[i].WeiPoint.y-vecSlineinfo[j].WeiPoint.y;
				double dsty = vecSlineinfo[i].upoint.y-vecSlineinfo[j].dpoint.y;
				double dstx = vecSlineinfo[i].upoint.x-vecSlineinfo[j].dpoint.x;
				Point2d diffw = Point2d(vecSlineinfo[i].WeiPoint.x-vecSlineinfo[j].WeiPoint.x,
					vecSlineinfo[i].WeiPoint.y-vecSlineinfo[j].WeiPoint.y);
				double cosvalue = abs(vecSlineinfo[i].vx*vecSlineinfo[j].vx+vecSlineinfo[i].vy*vecSlineinfo[j].vy);

				double cosvalue2 = abs(dstx*vecSlineinfo[j].vx+dsty*vecSlineinfo[j].vy)/
					sqrt((dstx*dstx+dsty*dsty)*(vecSlineinfo[j].vx*vecSlineinfo[j].vx+vecSlineinfo[j].vy*vecSlineinfo[j].vy));

				double cosvalue3 = abs(dstx*vecSlineinfo[j].vx+dsty*vecSlineinfo[j].vy)/
					sqrt((dstx*dstx+dsty*dsty)*(vecSlineinfo[j].vx*vecSlineinfo[j].vx+vecSlineinfo[j].vy*vecSlineinfo[j].vy));
				int data = longLane_cut.at<uchar>(vecSlineinfo[i].WeiPoint.y,vecSlineinfo[i].WeiPoint.x);

				double tanangle = atan(abs(diffw.y/(diffw.x+0.0000000001)))*180/PI;

				//if (abs(dsty)>1500||tanangle<75||cosvalue<0.8||abs(dstx)>20||dsty<0)//Threshold condition
				if (abs(dsty)>3000||cosvalue3<0.98||cosvalue2<0.98||cosvalue<0.98||dsty<0||abs(dstx)>longLane_cut.cols/3)//连接的阈值条件
				{
					continue;
				}
				objptr[j] +=1; 
				srcptr[i] +=1;

				PairPoints str_pairpoints;
				str_pairpoints.up = vecSlineinfo[i].upoint;
				str_pairpoints.down = vecSlineinfo[j].dpoint;
				str_pairpoints.objindex =j;//object line index；

				str_pairpoints.srcindex =i;//source line index；

				vecPairPoints.push_back(str_pairpoints);
				//	line(TTT,vecSlineinfo[i].upoint,vecSlineinfo[j].dpoint,Scalar(255,255,0),1);
			//	break;//has bug


			}
		}
		//if more than one up points link to the same down point
		for (int i = 0;i<vecSlineinfo.size();i++)
		{
			if (objptr[i]>1)
			{
				int num = objptr[i];
				Point *TemUPoints = new Point[num];
				Point TemDPoints;
				int n=0;
				int *index = new int[num];
				for (int i1 = 0;i1<vecPairPoints.size();i1++)
				{
					if (vecPairPoints[i1].objindex==i)
					{
						TemUPoints[n] = vecPairPoints[i1].up;
						index[n] = i1;
						n++;
						TemDPoints = vecPairPoints[i1].down;						
					}
				}
				int valuemin = abs(TemUPoints[0].y-TemDPoints.y);
				int reindext  = 0;
				reindext = index[0];
				int tempreindext = index[0];

				for (int i2=1;i2<n;i2++)
				{	
					double dx = vecPairPoints[index[i2]].up.x-vecPairPoints[index[i2]].down.x;
					double dy = vecPairPoints[index[i2]].up.y-vecPairPoints[index[i2]].down.y;
					double cosvalue = abs(vecSlineinfo[i].vx*dx + vecSlineinfo[i].vy*dy)/
						sqrt((dx*dx+dy*dy)*(vecSlineinfo[i].vx*vecSlineinfo[i].vx+vecSlineinfo[i].vy*vecSlineinfo[i].vy));
					double tanangle = atan(abs(dy/(dx+0.0000000001)))*180/PI;
					double cosangle = acos(cosvalue)*180/PI;
					if (cosvalue<0.99)
					{
						reindext = index[i2];
						vecPairPoints[reindext].up=Point(0,0);
						vecPairPoints[reindext].down=Point(0,0);
						continue;
					}
					if (valuemin<abs(TemUPoints[i2].y-TemDPoints.y))
					{
						reindext = index[i2];
						vecPairPoints[reindext].up=Point(0,0);
						vecPairPoints[reindext].down=Point(0,0);					
					}
					else
					{
						valuemin=abs(TemUPoints[i2].y-TemDPoints.y);						
						vecPairPoints[tempreindext].up=Point(0,0);
						vecPairPoints[tempreindext].down=Point(0,0);
						tempreindext = index[i2];
						reindext = index[i2];
					}
					
				}
				delete TemUPoints;
				delete index;
			}
			
		}

//		if one up point links to more than one down points
		for (int i = 0;i<vecSlineinfo.size();i++)
		{
			if (srcptr[i]>1)
			{
				int num = srcptr[i];
				Point *TemDPoints = new Point[num];
				Point TemUPoints;
				int n=0;
				int *index = new int[num];
				for (int i1 = 0;i1<vecPairPoints.size();i1++)
				{

					if (vecPairPoints[i1].srcindex==i)
					{
						if (vecPairPoints[i1].up.x)
						{
							TemDPoints[n] = vecPairPoints[i1].down;
							index[n] = i1;
							n++;
							TemUPoints = vecPairPoints[i1].up;		
						}										
					}
				}
				int valuemin = abs(TemDPoints[0].y-TemUPoints.y);
				int reindext  = 0;
				reindext = index[0];
				int tempreindext = index[0];
				Point minvalupoint,minvaldpoint;

				for (int i2=1;i2<n;i2++)
				{					
					if (valuemin<abs(TemDPoints[i2].y-TemUPoints.y))
					{
						reindext = index[i2];
						vecPairPoints[reindext].up=Point(0,0);
						vecPairPoints[reindext].down=Point(0,0);					
					}
					else
					{
						valuemin=abs(TemDPoints[i2].y-TemUPoints.y);						
						vecPairPoints[tempreindext].up=Point(0,0);
						vecPairPoints[tempreindext].down=Point(0,0);
						tempreindext = index[i2];
						reindext = index[i2];
					}
				}
				delete TemDPoints;
				delete index;
			}
		}

		cvtColor(longLane,longLane,COLOR_GRAY2RGB);
		for (int i = 0;i<vecPairPoints.size();i++)
		{
			if (vecPairPoints[i].up!=Point(0,0)&&(vecPairPoints[i].up.y>vecPairPoints[i].down.y))
			{
				line(TSline,vecPairPoints[i].up,vecPairPoints[i].down,Scalar(255,255,0),1);
				line(longLane,vecPairPoints[i].up,vecPairPoints[i].down,Scalar(255,0,0),5);
			}

		}
		//imwrite("TSline_link.png",TSline);
		//imwrite("longLane.png",longLane);

		Mat Tline_link_out = Mat::zeros(T.size(), CV_8UC1);
		threshold( TSline, TSline, 20, 255,0 );
		//imwrite("TSline_link2.png",TSline);

		vector<vector<Point>> contours4;
		vector<Vec4i> hierarchy4;

		findContours(TSline, contours4, hierarchy4, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		for (unsigned int i = 0; i < contours4.size(); i++)
		{
			vector<Point> pointss = contours4[i];
			Point upoint = pointss[0];
			Point dpoint = pointss[pointss.size()-1];
			for (int j=0;j<pointss.size();j++)
			{
				if (pointss[j].y<upoint.y)
				{
					upoint = pointss[j];
				}
				if (pointss[j].y>dpoint.y)
				{
					dpoint = pointss[j];
				}
			}
			int dst = abs(dpoint.y - upoint.y);
			if (dst>400)//delete the hight less than 300
			{				
				drawContours(Tline_link_out, contours4, i, Scalar(255), 1, 8, hierarchy4, 0);
			}
		}
		//imwrite("Tline_link_out.png",Tline_link_out);

		//step3: find the left and right lane.
		int w1 = T.cols;
		int h1 = T.rows;

		Mat roadDraw3;	
		cvtColor(Tline_link_out, roadDraw3, COLOR_GRAY2RGB);
		for(int i = h1-1; i >= 0; i--)//h-1
		{
			int changeLV = Tline_link_out.at<uchar>(i,320);
			if(changeLV!=0)
				{
					changeLP.push_back((H-1)-(i+(kk-1)*CUT));
				}

				//find first white
				vector<int> marker;
				for(int j = 0; j < w1; j++)
				{
					int data = Tline_link_out.at<uchar>(i,j);
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
						int dataMiddle = definePaint.at<uchar>(i,left);
						//int dataMiddle = TH.at<uchar>(i,left);
						if(dataMiddle==255)
						{
							
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].leftPoint.x = left;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].leftPoint.y = i+(kk-1)*CUT;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].isPaint_Left = 1;

							calPaintEdgePos(kapa2, i, left, middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].left_Edge_XY[0], middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].left_Edge_XY[1]);

							circle(roadDraw3, Point(left,i),0,CV_RGB(255, 0, 0),4);
						}
					}

					if (right< w1) 
					{
						int dataMiddle = definePaint.at<uchar>(i,right);
					//	int dataMiddle = TH.at<uchar>(i,left);
						if(dataMiddle==255)
						{
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].rightPoint.x = right;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].rightPoint.y = i+(kk-1)*CUT;
							middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].isPaint_Right = 1;

							calPaintEdgePos(kapa2, i, right, middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].right_Edge_XY[0], middleRoadPaintData[(H-1)-(i+(kk-1)*CUT)].right_Edge_XY[1]);

                            circle(roadDraw3, Point(right,i),0,CV_RGB(0, 0, 255),4);
						}
					}
				}			
		}
		
		if (kk==n_cut)
		{
			Mat drawroi = DrawMarker(Rect(0,(kk-1)*CUT,longLane.cols,longLane.rows-(kk-1)*CUT));
			roadDraw3.copyTo(drawroi,roadDraw3);
			//imwrite( "road MarkerAll.png", DrawMarker );

			Mat paintlineroi = PaintLine(Rect(0,(kk-1)*CUT,longLane.cols,longLane.rows-(kk-1)*CUT));
			T.copyTo(paintlineroi,T);
			//imwrite( "road PaintLine.png", PaintLine );
		}
		else
		{
			Mat drawroi = DrawMarker(Rect(0,(kk-1)*CUT,longLane.cols,CUT));
			roadDraw3.copyTo(drawroi,roadDraw3);
			//imwrite( "road Marker12.png", DrawMarker);

			Mat paintlineroi = PaintLine(Rect(0,(kk-1)*CUT,longLane.cols,CUT));
			T.copyTo(paintlineroi,T);
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
#if 0
	sort(changeLP.begin(),changeLP.end());

	int thresholdCL = 30;
	int changelinePoint = -1;

	for(int i = 0; i<changeLP.size(); i++)
	{
		if((changeLP[i]+thresholdCL)<H)
		{
			if(middleRoadPaintData[changeLP[i]+thresholdCL].leftPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]+thresholdCL].leftPoint.x-320)<50)
				{
					changelinePoint = changeLP[i];
					break;
				}
			}

			if(middleRoadPaintData[changeLP[i]+thresholdCL].rightPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]+thresholdCL].rightPoint.x-320)<50)
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
				if(abs(middleRoadPaintData[changeLP[i]-thresholdCL].leftPoint.x-320)<50)
				{
					changelinePoint = changeLP[i];
					break;
				}
			}

			if(middleRoadPaintData[changeLP[i]-thresholdCL].rightPoint.x!=-1)
			{
				if(abs(middleRoadPaintData[changeLP[i]-thresholdCL].rightPoint.x-320)<50)
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
		int sP = changelinePoint-150;
		if(sP<0)
		{
			sP = 0;
		}

		for(int i = sP; i<H; i++)
		{
			middleRoadPaintData[i].leftPoint = Point(-1, -1);
			middleRoadPaintData[i].Left_Middle_RelGPS = Point2d(0.0, 0.0);
			middleRoadPaintData[i].isPaint_Left = false;
			middleRoadPaintData[i].left_Edge_XY[0] = Point(-1, -1);
			middleRoadPaintData[i].left_Edge_XY[1] = Point(-1, -1);
			middleRoadPaintData[i].Left_Paint_Edge[0] = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Left_Paint_Edge[1] = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Left_Area_Pixel_Mean = -1.0;

			middleRoadPaintData[i].Middle_RelGPS = Point2d(0.0, 0.0);
			middleRoadPaintData[i].Middle_Area_Pixel_Mean = -1.0;

			middleRoadPaintData[i].rightPoint = Point(-1, -1);
			middleRoadPaintData[i].Right_Middle_RelGPS = Point2d(0.0, 0.0);
			middleRoadPaintData[i].isPaint_Right = false;
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

	//outLeft.close();
	//outRight.close();

	//Point2d GPS_start = Point2d(42.296853333333331,-83.213250000000002);
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
		}
		if(str == "lengthRate:")
		{
			readParam>>inParam.lengthRate;
		}
		if(str == "distanceOfSlantLeft:")
		{
			readParam>>inParam.distanceOfSlantLeft;
		}
		if(str == "distanceOfSlantRight:")
		{
			readParam>>inParam.distanceOfSlantRight;
		}
		if(str == "distanceOfUpMove:")
		{
			readParam>>inParam.distanceOfUpMove;
		}
		if(str == "distanceOfLeft:")
		{
			readParam>>inParam.distanceOfLeft;
		}
		if(str == "distanceOfRight:")
		{
			readParam>>inParam.distanceOfRight;
		}
		if(str == "stretchRate:")
		{
			readParam>>inParam.stretchRate;
		}
		if(str == "downSampling:")
		{
			readParam>>inParam.downSampling;
		}
		if(str == "distancePerPixel:")
		{
			readParam>>inParam.distancePerPixel;
		}
		if(str == "GPSref:")
		{
			readParam>>inParam.GPSref.x>>inParam.GPSref.y;
		}
		if(str == "ridgeThresh:")
		{
			readParam>>inParam.ridgeThresh;
		}
	}

	return true;
}

}
