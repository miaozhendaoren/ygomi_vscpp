#include <stdio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2\opencv.hpp>
#include <math.h>
#include <algorithm>
#include "utils.h"
#include "roadScan.h"
#include <iomanip>
#include <fstream>

using namespace std;
using namespace cv;

//#define IMAGE_SENSOR_WIDTH 640

int main(void)
{
	Parameters inParam;
	bool readStatus = readParamRoadScan("../../../../Ford/inVehicle/inVehicle/config/US.txt", inParam);

	if(!readStatus)
	{
		cout<<"read parameters error"<<endl;
		return -1;
	}

	int videoIndex = 0;

	int locNum[2], holeNum[2];

	//for(int kk = 0; kk<5; kk++)	//England
	//for(int kk = 0; kk<4; kk++)	//US
	{
		VideoCapture capture;
		FILE* gpsFile;

#if 0
		if (videoIndex%5 == 0)
		{
			MAP = Mat::zeros(600, 1000, CV_8UC3);
			capture.open("F:/roadDB/traffic signs recognition/our data/cam_20150713105036.mp4");
			gpsFile = fopen("F:/roadDB/traffic signs recognition/our data/list_20150713105036.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}
		else if (videoIndex%5 == 1)
		{
			capture.open("F:/roadDB/traffic signs recognition/our data/cam_20150713104153.mp4");
			gpsFile = fopen("F:/roadDB/traffic signs recognition/our data/list_20150713104153.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}
		else if (videoIndex%5 == 2)
		{
			capture.open("F:/roadDB/traffic signs recognition/our data/cam_20150713103609.mp4");
			gpsFile = fopen("F:/roadDB/traffic signs recognition/our data/list_20150713103609.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}
		else if (videoIndex%5 == 3)
		{
			capture.open("F:/roadDB/traffic signs recognition/our data/cam_20150713103327.mp4");
			gpsFile = fopen("F:/roadDB/traffic signs recognition/our data/list_20150713103327.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}
		else if (videoIndex%5 == 4)
		{
			capture.open("F:/roadDB/traffic signs recognition/our data/cam_20150713102750.mp4");
			gpsFile = fopen("F:/roadDB/traffic signs recognition/our data/list_20150713102750.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}
#else

		if (videoIndex%4 == 0)
		{
			//MAP = Mat::zeros(600, 1000, CV_8UC3);
			capture.open("F:/roadDB/Ford/NewcoData/MKS360_20130722_003_Uncompressed.avi");
			gpsFile = fopen("F:/roadDB/Ford/NewcoData/gps_003.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);


		}
		else if (videoIndex%4 == 1)
		{
			capture.open("F:/roadDB/Ford/NewcoData/MKS360_20130722_004_Uncompressed.avi");
			gpsFile = fopen("F:/roadDB/Ford/NewcoData/gps_004.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}
		else if (videoIndex%4 == 2)
		{
			capture.open("F:/roadDB/Ford/NewcoData/MKS360_20130722_005_Uncompressed.avi");
			gpsFile = fopen("F:/roadDB/Ford/NewcoData/gps_005.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}
		else if (videoIndex%4 == 3)
		{
			capture.open("F:/roadDB/Ford/NewcoData/MKS360_20130722_006_Uncompressed.avi");
			gpsFile = fopen("F:/roadDB/Ford/NewcoData/gps_006.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
		}

#endif
		int number_of_frames = capture.get(CV_CAP_PROP_POS_FRAMES);

		if ( !capture.isOpened() )  // if not success, exit program
		{
			cout<<"error" <<endl;
			return -1;
		}
		else
		{
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
			double fps = capture.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video	
		}

		////////////////////////////////////////////////////////////////////////////
		//Step1:  Kalman initialization	
	
		//ofstream widthAndGPS(txtname);
		Size S = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH), (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));	

		if(S.width!=640)
		{
			//S.height = 400;
			//S.width = 640;

			S.height *= 0.5;
			S.width *= 0.5;
		}
        
        vector<dataEveryRow> roadPaintData;
	    vector<gpsInformationAndInterval> GPSAndInterval;
        
		////////////////////////////////////////////////////////////////////////////
		Mat history = Mat::zeros(S.height *HH*SCALE,S.width, CV_8UC1);

		Point2d *GPS_Points;
		//GPS_Points = new Point2d [S.height *HH*SCALE];
		
		int rowIndex = 0;
		int IntervalTmp = 0;
		int Interval = 0;
		int GPStmp = 0;

		Point2d GPS_abs, GPS_ref, GPS_next;
		//ref = Point2d(initialPointOfGPS[0], initialPointOfGPS[1]);

		if(!feof(gpsFile))
		{
			fscanf(gpsFile,"%lf,%lf\n",&GPS_abs.x,&GPS_abs.y);
		}

		Point curr = Point(0,0);
		Point last = Point(0,0);
        
        gpsInformationAndInterval gpsAndInterval;
		Mat image;
		int intrtmp=0;

		//capture.set(CV_CAP_PROP_POS_FRAMES, 2000);

		for(int index = 0; index < 200; index++)//number_of_frames
        {
			capture >> image;
            if(!feof(gpsFile))
			{
				fscanf(gpsFile,"%lf,%lf\n",&GPS_next.x,&GPS_next.y);
			}

        	//Method 2:
        	roadImageGen(image, history, &rowIndex, &GPS_abs, &GPS_next, &gpsAndInterval, &intrtmp, inParam);
			if (gpsAndInterval.intervalOfInterception)
			{
				 GPSAndInterval.push_back(gpsAndInterval);
			}
          
            GPS_abs = GPS_next;
        }
        
        fclose(gpsFile);

		if(!GPSAndInterval.empty())
		{
			rowIndex -= GPSAndInterval[GPSAndInterval.size()-1].intervalOfInterception;
		}

		Mat historyROI = history(Rect(0,rowIndex,history.cols,history.rows-rowIndex));

		imwrite("historyroi.png",historyROI);
		//roadImageProc2("road_time0.png");
		
		roadImageProc2(historyROI, GPSAndInterval, roadPaintData, inParam);
    
		int H = historyROI.rows;
		
		//ofstream outFile0("GPS_0.txt");
		//ofstream outFile1("GPS_1.txt");
		//ofstream outFile2("GPS_2.txt");
		//for(int i = 0; i<roadPaintData.size(); i++)
		//{
		//	//double distance = sqrt(pow(roadPaintData[i].leftGPS.x-roadPaintData[i].rightGPS.x,2.0)+pow(roadPaintData[i].leftGPS.y-roadPaintData[i].rightGPS.y,2.0));
		//	outFile0<<setprecision(20)<<roadPaintData[i].Left_Middle_RelGPS.x<<" "<<roadPaintData[i].Left_Middle_RelGPS.y<<" "<<roadPaintData[i].Right_Middle_RelGPS.x<<" "<<roadPaintData[i].Right_Middle_RelGPS.y<<
		//		" "<<roadPaintData[i].isPaint_Left<<" "<<roadPaintData[i].isPaint_Right<<endl;

		//	outFile1<<setprecision(20)<<roadPaintData[i].Left_Paint_Edge[0].x<<" "<<roadPaintData[i].Left_Paint_Edge[0].y<<" "<<roadPaintData[i].Left_Paint_Edge[1].x<<" "<<roadPaintData[i].Left_Paint_Edge[1].y<<
		//		" "<<roadPaintData[i].isPaint_Left<<" "<<roadPaintData[i].isPaint_Right<<endl;

		//	outFile2<<setprecision(20)<<roadPaintData[i].Right_Paint_Edge[0].x<<" "<<roadPaintData[i].Right_Paint_Edge[0].y<<" "<<roadPaintData[i].Right_Paint_Edge[1].x<<" "<<roadPaintData[i].Right_Paint_Edge[1].y<<
		//		" "<<roadPaintData[i].isPaint_Left<<" "<<roadPaintData[i].isPaint_Right<<endl;
		//}

		//outFile0.close();
		//outFile1.close();
		//outFile2.close();

		ofstream dataStruct("dataStruct.txt");
		dataStruct<<setprecision(20)<<inParam.GPSref.x<<" "<<inParam.GPSref.y<<endl;

		//for(int i = 0; i<roadPaintData.size(); i++)
		//{
		//	dataStruct<<setprecision(20)<<roadPaintData[i].Left_Middle_RelGPS.x<<" "<<roadPaintData[i].Left_Middle_RelGPS.y<<" "<<roadPaintData[i].isPaint_Left<<" "
		//		<<roadPaintData[i].Left_Paint_Edge[0].x<<" "<<roadPaintData[i].Left_Paint_Edge[0].y<<" "
		//		<<roadPaintData[i].Left_Paint_Edge[1].x<<" "<<roadPaintData[i].Left_Paint_Edge[1].y<<" "
		//		<<roadPaintData[i].Left_Area_Pixel_Mean<<" "
		//		<<roadPaintData[i].Middle_RelGPS.x<<" "<<roadPaintData[i].Middle_RelGPS.y<<" "<<roadPaintData[i].Middle_Area_Pixel_Mean<<" "
		//		<<roadPaintData[i].Right_Middle_RelGPS.x<<" "<<roadPaintData[i].Right_Middle_RelGPS.y<<" "<<roadPaintData[i].isPaint_Right<<" "
		//		<<roadPaintData[i].Right_Paint_Edge[0].x<<" "<<roadPaintData[i].Right_Paint_Edge[0].y<<" "
		//		<<roadPaintData[i].Right_Paint_Edge[1].x<<" "<<roadPaintData[i].Right_Paint_Edge[1].y<<" "
		//		<<roadPaintData[i].Right_Area_Pixel_Mean<<endl;
		//}

		//ofstream outFileGPS("GPSInterval.txt");
		//for(int i=0;i<GPSAndInterval.size();i++)
		//{
		//	outFileGPS<<setprecision(20)<<GPSAndInterval[i].GPS_now.x<<" "<<GPSAndInterval[i].GPS_now.y<<" "<<GPSAndInterval[i].GPS_next.x<<" "<<GPSAndInterval[i].GPS_next.y<<
		//		" "<<GPSAndInterval[i].intervalOfInterception<<endl;
		//}
		//outFileGPS.close();

		roadPaintData.clear();
		GPSAndInterval.clear();

		cout<<"GPS output finish"<<endl;

		videoIndex++;
		
		//system("pause");
		//waitKey(-1);
	}
}