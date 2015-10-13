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

#define VW (0)
#define Ford (1)
#define Honda (2)
#define England (3)
#define Other (4)
#define Honda2 (5)
#define Airport (6)
#define Airport2 (7)
#define VW2 (8)

using namespace std;
using namespace cv;
using namespace ns_roadScan;

cv::Mat H, invertH;

int main(void)
{
	Parameters inParam;
	//	DE_Airport2 -- US_Detroit -- DE_Lehre2
	bool readStatus = readParamRoadScan("../../../../Ford/inVehicle/inVehicle/config/DE_Airport.txt", inParam);

	// Calculate H and H inverse for road scan and traffic sign detection
	ns_roadScan::calHAndInvertH(inParam, H, invertH);

	if( !readStatus )
	{
		cout<<"read parameters error"<<endl;
		return -1;
	}

	int ChooseVideo = Airport;
	int videoIndex = 0;

	int locNum[2], holeNum[2];

	for(int kk = 0; kk < 6; kk++)	//	Airport2
	{
		VideoCapture capture;
		FILE* gpsFile;

		if(ChooseVideo == Airport2)
		{
			if (videoIndex == 0)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811111511.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811111511.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 1)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811112010.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811112010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 2)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811112510.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811112510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 3)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811113010.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811113010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 4)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811113510.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811113510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex == 5)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811114010.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811114010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex == 6)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811114510.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811114510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 7)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811115010.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811115010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 8)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811115510.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811115510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 9)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811120010.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811120010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex == 10)
			{
				capture.open("F:/roadDB/Airport 2/cam_20150811120510.mp4");
				gpsFile = fopen("F:/roadDB/Airport 2/gps/list_20150811120510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
		}
		else if(ChooseVideo == Airport)
		{
			if (videoIndex == 0)
			{
				capture.open("F:/roadDB/Airport/cam_20150806120920.mp4");
				gpsFile = fopen("F:/roadDB/Airport/gps/list_20150806120920.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
		}
		else if(ChooseVideo == Ford)
		{
			if (videoIndex == 0)
			{
				capture.open("F:/roadDB/Ford/NewcoData/MKS360_20130722_003_Uncompressed.avi");
				gpsFile = fopen("F:/roadDB/Ford/NewcoData/gps_003.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
		}
		else if(ChooseVideo == VW2)
		{
			if (videoIndex == 0)
			{
				capture.open("C:/Users/ypren/Documents/newco_demo/Demo/Ford/inVehicle/inVehicle/resource/Germany/Lehre2/reverse/cap_20150722110100_cut/cam_20150722110100.mp4");
				gpsFile = fopen("C:/Users/ypren/Documents/newco_demo/Demo/Ford/inVehicle/inVehicle/resource/Germany/Lehre2/reverse/cap_20150722110100_cut/list_20150722110100.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
		}

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

		Size S = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH), (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));	

		S.height *= inParam.imageScaleHeight;
		S.width *= inParam.imageScaleWidth;
        
        vector<dataEveryRow> roadPaintData;
		vector<dataEveryRow> roadPaintDataALL;
	    vector<gpsInformationAndInterval> GPSAndInterval;
        
		////////////////////////////////////////////////////////////////////////////
		Mat history = Mat::zeros(S.height *HH*SCALE,S.width, CV_8UC1);
		
		int rowIndex = 0;
		int IntervalTmp = 0;
		int Interval = 0;
		int GPStmp = 0;

		Point2d GPS_next;

        gpsInformationAndInterval gpsAndInterval;
		Mat image;
		int intrtmp = 0;
		int frames = 350;
		vector<Point2d> gps_points;
		
		while(!feof(gpsFile))
		{
			fscanf(gpsFile,"%lf,%lf\n",&GPS_next.x,&GPS_next.y);
			gps_points.push_back(GPS_next);
		}
	
		for (int n = 0; n < 150; n++)
		{
			if ((n*frames + 1) > number_of_frames)
			{
				break;
			}

			capture.set(CV_CAP_PROP_POS_FRAMES, n*frames + 1);

			for(int index = 0; index < frames; index++)//number_of_frames
			{
				capture >> image;
				
				if (image.data && ((n*frames + index + 1) < gps_points.size()))
				{
					roadImageGen(image, history, &rowIndex, &gps_points[n*frames+index], &gps_points[n*frames+index+1], &gpsAndInterval, &intrtmp, inParam);
					
					if (gpsAndInterval.intervalOfInterception)
					{
						GPSAndInterval.push_back(gpsAndInterval);
					}
					
					if(((index == (frames - 1)) || ((n*frames + index + 1) == gps_points.size())) && ( !GPSAndInterval.empty()))
					{
						rowIndex -= GPSAndInterval[GPSAndInterval.size()-1].intervalOfInterception;
					}
				}
				else
				{
					break;
				}
			}
			
			Mat historyROI = history(Rect(0, rowIndex, history.cols, history.rows - rowIndex));
			imwrite("historyroi.png", historyROI);
			
			rowIndex = 0;
			intrtmp = 0;
			
			roadImageProc2(historyROI, GPSAndInterval, roadPaintData, inParam);
			
			history = Mat::zeros(S.height*HH*SCALE, S.width, CV_8UC1);
			
			int H = historyROI.rows;
			
			for(int i = 0; i < roadPaintData.size(); i++)
			{
				roadPaintDataALL.push_back(roadPaintData[i]);
			}
			
			roadPaintData.clear();
			GPSAndInterval.clear();
		}

		char texname[32];

		if (ChooseVideo == VW)
		{
			sprintf(texname, "dataStruct_%d.txt", videoIndex);
		}
		else if (ChooseVideo == Ford)
		{
			sprintf(texname, "dataStruct_%d.txt", videoIndex);
		}
		else if ((ChooseVideo == Honda) || (ChooseVideo == Honda2))
		{
			sprintf(texname, "dataStruct_%d.txt", videoIndex);
		}
		else if (ChooseVideo == Other)
		{
			sprintf(texname, "dataStruct_%d.txt", videoIndex);
		}
		else if (ChooseVideo == Airport)
		{
			sprintf(texname, "dataStruct_%d.txt", videoIndex);
		}
		else if (ChooseVideo == Airport2)
		{
			sprintf(texname, "dataStruct_%d.txt", videoIndex);
		}

		ofstream dataStruct(texname);
		dataStruct<<setprecision(20)<<inParam.GPSref.x<<" "<<inParam.GPSref.y<<endl;


		for(int i = 0; i<roadPaintDataALL.size(); i++)
		{		
			dataStruct<<setprecision(20)<<roadPaintDataALL[i].Left_Middle_RelGPS.x<<" "<<roadPaintDataALL[i].Left_Middle_RelGPS.y<<" "<<roadPaintDataALL[i].isPaint_Left<<" "
				<<roadPaintDataALL[i].Left_Paint_Edge[0].x<<" "<<roadPaintDataALL[i].Left_Paint_Edge[0].y<<" "
				<<roadPaintDataALL[i].Left_Paint_Edge[1].x<<" "<<roadPaintDataALL[i].Left_Paint_Edge[1].y<<" "
				<<roadPaintDataALL[i].Left_Area_Pixel_Mean<<" "
				<<roadPaintDataALL[i].Middle_RelGPS.x<<" "<<roadPaintDataALL[i].Middle_RelGPS.y<<" "<<roadPaintDataALL[i].Middle_Area_Pixel_Mean<<" "
				<<roadPaintDataALL[i].Right_Middle_RelGPS.x<<" "<<roadPaintDataALL[i].Right_Middle_RelGPS.y<<" "<<roadPaintDataALL[i].isPaint_Right<<" "
				<<roadPaintDataALL[i].Right_Paint_Edge[0].x<<" "<<roadPaintDataALL[i].Right_Paint_Edge[0].y<<" "
				<<roadPaintDataALL[i].Right_Paint_Edge[1].x<<" "<<roadPaintDataALL[i].Right_Paint_Edge[1].y<<" "
				<<roadPaintDataALL[i].Right_Area_Pixel_Mean<<endl;
		}

		////output real middle real GPS
		//vector<Point2d> actualGPS;
		//for(int index = 0; index < roadPaintDataALL.size(); index++)
		//{
		//	Point2d middleRealGPS = Point2d(0.0, 0.0);

		//	//calculate real GPS

		//	if(roadPaintDataALL[index].isPaint_Right == 1)
		//	{
		//		calActualGPSFromRef(roadPaintDataALL[index].Right_Middle_RelGPS, inParam.GPSref, middleRealGPS);
		//	}
		//	else
		//	{
		//		calActualGPSFromRef(roadPaintDataALL[index].Middle_RelGPS, inParam.GPSref, middleRealGPS);
		//	}

		//	actualGPS.push_back(middleRealGPS);
		//}

		//ofstream realGPS("realGPS.txt");
		//for(int index = 0; index < actualGPS.size(); index++)
		//{
		//	realGPS<<setprecision(20)<<actualGPS[index].y<<","<<actualGPS[index].x<<","<<0<<endl;
		//}
		//realGPS.close();
		////end output

		cout<<"output finish"<<endl;
		dataStruct.close();
		roadPaintDataALL.clear();

		videoIndex++;
	}
}