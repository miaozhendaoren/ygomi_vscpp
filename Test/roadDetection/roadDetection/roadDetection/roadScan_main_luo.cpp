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

#define VW      0
#define Ford    1
#define Honda   2
#define England 3
#define Other   4
#define Honda2  5
#define Airport  6
#define Airport2  7

using namespace std;
using namespace cv;
using namespace ns_roadScan;
cv::Mat H, invertH;

//#define IMAGE_SENSOR_WIDTH 640

int main(void)
{
	//ifstream readParam("England.txt",ios::_Nocreate);
	//ifstream readParam("VW.txt",ios::_Nocreate);DE_Lehre
	//ifstream readParam("HD.txt",ios::_Nocreate);DE_Airport2
	//ifstream readParam("US.txt",ios::_Nocreate);US_Detroit
	Parameters inParam;
	bool readStatus = readParamRoadScan("../../../../Ford/inVehicle/inVehicle/config/DE_Airport2.txt", inParam);
    ns_roadScan::calHAndInvertH(inParam, H, invertH);
	if(!readStatus)
	{
		cout<<"read parameters error"<<endl;
		return -1;
	}

	int ChooseVideo = Airport2;

	int videoIndex = 3;
	
	/*string gpsname;
	string videoname;
	ifstream fingps("F:/Airport2/gpslist.txt");
	ifstream finvideo("F:/Airport2/videolist.txt");*/

	int locNum[2], holeNum[2];

//	for(int kk = 0; kk<20; kk++)	//VW
//	for(int kk = 0; kk<19; kk++)	//US
//	for(int kk = 0; kk<3; kk++)	//Honda
	for(int kk = 0; kk<11; kk++)//Airport2
//	for(int kk = 0; kk<3; kk++)
	
	{
		VideoCapture capture;
		FILE* gpsFile;

		if (ChooseVideo==Airport2)
		{	
			/*getline(fingps,gpsname);									
			gpsname = "F:/Airport2/"+gpsname;
			const char* c_s = gpsname.c_str();
			gpsFile = fopen(c_s,"r");
			getline(finvideo,videoname);									
			videoname = "F:/Airport2/"+videoname;
			capture.open(videoname);
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);	*/

			if (videoIndex%11 == 0)
			{
				capture.open("F:/Airport2/cam_20150811111511.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811111511.txt","r");

				//capture.open("F:/fengyang/b.avi");
				//gpsFile = fopen("F:/fengyang/b.txt","r");


				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 1)
			{
				capture.open("F:/Airport2/cam_20150811112010.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811112010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 2)
			{
				capture.open("F:/Airport2/cam_20150811112510.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811112510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 3)
			{
				capture.open("F:/Airport2/cam_20150811113010.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811113010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 4)
			{
				capture.open("F:/Airport2/cam_20150811113510.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811113510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex%11 == 5)
			{
				capture.open("F:/Airport2/cam_20150811114010.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811114010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex%11 == 6)
			{
				capture.open("F:/Airport2/cam_20150811114510.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811114510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 7)
			{
				capture.open("F:/Airport2/cam_20150811115010.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811115010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 8)
			{
				capture.open("F:/Airport2/cam_20150811115510.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811115510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 9)
			{
				capture.open("F:/Airport2/cam_20150811120010.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811120010.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%11 == 10)
			{
				capture.open("F:/Airport2/cam_20150811120510.mp4");
				gpsFile = fopen("F:/Airport2/list_20150811120510.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

		}

		if (ChooseVideo == Airport)
		{
			if (videoIndex%3 == 0)
			{
				capture.open("F:/Airport/cam_20150806113420.mp4");
				gpsFile = fopen("F:/Airport/list_20150806113420.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			if (videoIndex%3 == 1)
			{
				capture.open("F:/Airport/cam_20150806113920.mp4");
				gpsFile = fopen("F:/Airport/list_20150806113920.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			if (videoIndex%3 == 2)
			{
				capture.open("F:/Airport/cam_20150806115920.mp4");
				gpsFile = fopen("F:/Airport/list_20150806115920.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
		}

		else if (ChooseVideo==VW)
		{
            if (videoIndex%20 == 0)
            {
                capture.open("F:/VW_Data_Selected/statler/statler-7200-8821.mp4");
                gpsFile = fopen("F:/VW_Data_Selected/statler/statler-7200-8821.txt","r");
                capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
            }
            else if (videoIndex%20 == 14)
            {
                capture.open("F:/VW_Data_Selected/Wanda/part5/5.mp4");
                gpsFile = fopen("F:/VW_Data_Selected/Wanda/part5/gps5.txt","r");
                capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
            }
            else if (videoIndex%20 == 19)
            {
                capture.open("F:/VW_Data_Selected/wayne/wayne-10546-11861.mp4");
                gpsFile = fopen("F:/VW_Data_Selected/wayne/wayne-10546-11861.txt","r");
                capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
            }
            else if (videoIndex%20 == 10)
            {
                capture.open("F:/VW_Data_Selected/Wanda/part1/1.mp4");
                gpsFile = fopen("F:/VW_Data_Selected/Wanda/part1/gps1.txt","r");
                capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
            }

			/*if (videoIndex%20 == 0)
				{
					capture.open("F:/VW_Data_Selected/statler/statler-7200-8821.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/statler/statler-7200-8821.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 1)
				{
					capture.open("F:/VW_Data_Selected/statler/statler-9823-10055.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/statler/statler-9823-10055.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 2)
				{
					capture.open("F:/VW_Data_Selected/statler/statler-10808-11692.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/statler/statler-10808-11692.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 3)
				{
					capture.open("F:/VW_Data_Selected/statler/statler-14601-15101.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/statler/statler-14601-15101.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 4)
				{
					capture.open("F:/VW_Data_Selected/statler/statler-15439-15766.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/statler/statler-15439-15766.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}

				else if (videoIndex%20 == 5)
				{
					capture.open("F:/VW_Data_Selected/Waldorf/part1/1.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Waldorf/part1/gps1.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}

				else if (videoIndex%20 == 6)
				{
					capture.open("F:/VW_Data_Selected/Waldorf/part2/2.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Waldorf/part2/gps2.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 7)
				{
					capture.open("F:/VW_Data_Selected/Waldorf/part3/3.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Waldorf/part3/gps3.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 8)
				{
					capture.open("F:/VW_Data_Selected/Waldorf/part4/4.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Waldorf/part4/gps4.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 9)
				{
					capture.open("F:/VW_Data_Selected/Waldorf/part5/5.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Waldorf/part5/gps5.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}

				else if (videoIndex%20 == 10)
				{
					capture.open("F:/VW_Data_Selected/Wanda/part1/1.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Wanda/part1/gps1.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}

				else if (videoIndex%20 == 11)
				{
					capture.open("F:/VW_Data_Selected/Wanda/part2/2.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Wanda/part2/gps2.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 12)
				{
					capture.open("F:/VW_Data_Selected/Wanda/part3/3.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Wanda/part3/gps3.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 13)
				{
					capture.open("F:/VW_Data_Selected/Wanda/part4/4.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Wanda/part4/gps4.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 14)
				{
					capture.open("F:/VW_Data_Selected/Wanda/part5/5.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/Wanda/part5/gps5.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}

				else if (videoIndex%20 == 15)
				{
					capture.open("F:/VW_Data_Selected/wayne/wayne-561-1907.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/wayne/wayne-561-1907.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}

				else if (videoIndex%20 == 16)
				{
					capture.open("F:/VW_Data_Selected/wayne/wayne-2259-3038.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/wayne/wayne-2259-3038.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 17)
				{
					capture.open("F:/VW_Data_Selected/wayne/wayne-3376-3858.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/wayne/wayne-3376-3858.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 18)
				{
					capture.open("F:/VW_Data_Selected/wayne/wayne-7590-8074.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/wayne/wayne-7590-8074.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%20 == 19)
				{
					capture.open("F:/VW_Data_Selected/wayne/wayne-10546-11861.mp4");
					gpsFile = fopen("F:/VW_Data_Selected/wayne/wayne-10546-11861.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}*/
		}
		else if(ChooseVideo==Ford)
		{
			if (videoIndex%19 == 0)
				{
					capture.open("F:/carvideo/MKS360_20130722_001_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_001.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);

				}
				else if (videoIndex%19 == 1)
				{
					capture.open("F:/carvideo/MKS360_20130722_002_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_002.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%19 == 2)
				{
					capture.open("F:/carvideo/MKS360_20130722_003_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_003.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
				else if (videoIndex%19 == 3)
				{
					capture.open("F:/carvideo/MKS360_20130722_004_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_004.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                else if (videoIndex%19 == 4)
				{
					capture.open("F:/carvideo/MKS360_20130722_005_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_005.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                else if (videoIndex%19 == 5)
				{
					capture.open("F:/carvideo/MKS360_20130722_006_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_006.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                else if (videoIndex%19 == 6)
				{
					capture.open("F:/carvideo/MKS360_20130722_007_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_007.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                else if (videoIndex%19 == 7)
				{
					capture.open("F:/carvideo/MKS360_20130722_008_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_008.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                else if (videoIndex%19 == 8)
				{
					capture.open("F:/carvideo/MKS360_20130722_009_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_009.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                else if (videoIndex%19 == 9)
				{
					capture.open("F:/carvideo/MKS360_20130722_0010_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_0010.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                else if (videoIndex%19 == 10)
				{
					capture.open("F:/carvideo/MKS360_20130722_011_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_011.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}

                else if (videoIndex%19 == 11)
                {
                    continue;
                }

                 else if (videoIndex%19 == 12)
				{
					capture.open("F:/carvideo/MKS360_20130722_013_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_013.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				} 
                 else if (videoIndex%19 == 13)
				{
					capture.open("F:/carvideo/MKS360_20130722_014_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_014.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				} 
                 else if (videoIndex%19 == 14)
				{
					capture.open("F:/carvideo/MKS360_20130722_015_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_015.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				} 
                 else if (videoIndex%19 == 15)
				{
					capture.open("F:/carvideo/MKS360_20130722_016_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_016.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				} 
                 else if (videoIndex%19 == 16)
				{
					capture.open("F:/carvideo/MKS360_20130722_017_Uncompressed.avi");
					gpsFile = fopen("F:/carvideo/gps_017.txt","r");
					capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
				}
                 else if (videoIndex%19 == 17)
                 {
                     capture.open("F:/carvideo/MKS360_20130722_018_Uncompressed.avi");
                     gpsFile = fopen("F:/carvideo/gps_018.txt","r");
                     capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
                 }
                 else if (videoIndex%19 == 18)
                 {
                     capture.open("F:/carvideo/MKS360_20130722_019_Uncompressed.avi");
                     gpsFile = fopen("F:/carvideo/gps_019.txt","r");
                     capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
                 }

		}
		else if(ChooseVideo == Honda)
		{
			if (videoIndex%10 == 0)
			{
				capture.open("F:/HondaDataProcessed/selected/1_fps51.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/1_fps51.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 1)
			{
				capture.open("F:/HondaDataProcessed/selected/2_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/2_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 2)
			{
				capture.open("F:/HondaDataProcessed/selected/3_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/3_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 3)
			{
				capture.open("F:/HondaDataProcessed/selected/4_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/4_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 4)
			{
				capture.open("F:/HondaDataProcessed/selected/5_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/5_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex%10 == 5)
			{
				capture.open("F:/HondaDataProcessed/selected/6_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/6_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex%10 == 6)
			{
				capture.open("F:/HondaDataProcessed/selected/7_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/7_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 7)
			{
				capture.open("F:/HondaDataProcessed/selected/8_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/8_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 8)
			{
				capture.open("F:/HondaDataProcessed/selected/9_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/9_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 9)
			{
				capture.open("F:/HondaDataProcessed/selected/10_fps55.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/selected/gps/10_fps55.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

		}
		else if(ChooseVideo == Honda2)
		{
			if (videoIndex%10 == 0)
			{
				capture.open("F:/HondaDataProcessed/20150803/1.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/1.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 1)
			{
				capture.open("F:/HondaDataProcessed/20150803/2.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/2.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 2)
			{
				capture.open("F:/HondaDataProcessed/20150803/3.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/3.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 3)
			{
				capture.open("F:/HondaDataProcessed/20150803/4.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/4.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 4)
			{
				capture.open("F:/HondaDataProcessed/20150803/5.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/5.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex%10 == 5)
			{
				capture.open("F:/HondaDataProcessed/20150803/6.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/6.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

			else if (videoIndex%10 == 6)
			{
				capture.open("F:/HondaDataProcessed/20150803/7.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/7.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 7)
			{
				capture.open("F:/HondaDataProcessed/20150803/8.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/8.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 8)
			{
				capture.open("F:/HondaDataProcessed/20150803/9.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/9.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}
			else if (videoIndex%10 == 9)
			{
				capture.open("F:/HondaDataProcessed/20150803/10.mp4");
				gpsFile = fopen("F:/HondaDataProcessed/20150803/10.txt","r");
				capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
			}

		}
		else if (ChooseVideo==Other)
		{
			capture.open("F:/HondaDataProcessed/Honda_frames_2988_fps_53.1237.avi");
			gpsFile = fopen("F:/HondaDataProcessed/gps_no_offset.txt","r");
			capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);
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

		////////////////////////////////////////////////////////////////////////////
		//Step1:  Kalman initialization	
	
		//ofstream widthAndGPS(txtname);
		Size S = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH), (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));	

		if(S.width!=640)
		{
			//S.height = 400;
			//S.width = 640;
			S.height /= 2;
			S.width /= 2;
			//S.height *= 0.5;
			//S.width *= 0.5;
		}
        
        vector<dataEveryRow> roadPaintData;
		vector<dataEveryRow> roadPaintDataALL;
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
		float aa;
		//ref = Point2d(initialPointOfGPS[0], initialPointOfGPS[1]);

        gpsInformationAndInterval gpsAndInterval;
		Mat image;
		int intrtmp=0;
		int frames = 350;
		vector<Point2d> gps_points;
		
		while(!feof(gpsFile))
		{
			fscanf(gpsFile,"%lf,%lf\n",&GPS_next.x,&GPS_next.y);
			gps_points.push_back(GPS_next);
		}
	//	capture.set(CV_CAP_PROP_POS_FRAMES, 3100);
	//	number_of_frames=1300;

	
		for (int n=14;n<150;n++)
		{

            cout<<"video="<<videoIndex<<endl;
			cout<<"interval="<<n<<endl;
			if (n*frames+1>number_of_frames)
			{
				cout<<"endl";
				break;
				
			}
			capture.set(CV_CAP_PROP_POS_FRAMES, n*frames+1);
			for(int index = 0; index < frames; index++)//number_of_frames
			{
		//		cout<<index<<endl;
				capture >> image;
				if (image.data&& n*frames+index+1<gps_points.size())
				{
					roadImageGen(image, history, &rowIndex, &gps_points[n*frames+index], &gps_points[n*frames+index+1], &gpsAndInterval, &intrtmp, inParam);
					if (gpsAndInterval.intervalOfInterception)
					{
						GPSAndInterval.push_back(gpsAndInterval);
					}
					if(index==frames-1||n*frames+index+1==gps_points.size())
					{
						rowIndex -= GPSAndInterval[GPSAndInterval.size()-1].intervalOfInterception;
					}
				}
				else
					break;
			}
			Mat historyROI = history(Rect(0,rowIndex,history.cols,history.rows-rowIndex));
			imwrite("historyroi.png",historyROI);
			

		rowIndex=0;
		intrtmp=0;
		roadImageProc2(historyROI, GPSAndInterval, roadPaintData, inParam);
		history = Mat::zeros(S.height *HH*SCALE,S.width, CV_8UC1);
    
		int H = historyROI.rows;
		
		/*ofstream outFile0("GPS_0.txt");
		for(int i = 0; i<roadPaintData.size(); i++)
		{
			double distance = sqrt(pow(roadPaintData[i].Left_Middle_RelGPS.x-roadPaintData[i].Right_Middle_RelGPS.x,2.0)+pow(roadPaintData[i].Left_Middle_RelGPS.y-roadPaintData[i].Right_Middle_RelGPS.y,2.0));
	
			outFile0<<setprecision(20)<<distance<<endl;
		}
		outFile0.close();*/


		

		for(int i = 0; i<roadPaintData.size(); i++)
		{
			roadPaintDataALL.push_back(roadPaintData[i]);
		}

		roadPaintData.clear();
		GPSAndInterval.clear();	
		}

		char texname[32];
		if (ChooseVideo== VW)
		{
			sprintf(texname,"dataStruct_%d.txt",videoIndex%20);
		}
		else if (ChooseVideo==Ford)
		{
			sprintf(texname,"dataStruct_%d.txt",videoIndex%4);
		}
		else if (ChooseVideo==Honda||ChooseVideo==Honda2)
		{
			sprintf(texname,"dataStruct_%d.txt",videoIndex%10);
		}
		else if (ChooseVideo == Other)
		{
			sprintf(texname,"dataStruct_%d.txt",videoIndex%1);
		}
		else if (ChooseVideo == Airport)
		{
			sprintf(texname,"dataStruct_%d.txt",videoIndex%3);
		}
		else if (ChooseVideo == Airport2)
		{
			sprintf(texname,"dataStruct_%d.txt",videoIndex%11);
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

			double left_paint = sqrt(pow(roadPaintDataALL[i].Left_Paint_Edge[0].x-roadPaintDataALL[i].Left_Paint_Edge[1].x,2.0)
				+pow(roadPaintDataALL[i].Left_Paint_Edge[0].y-roadPaintDataALL[i].Left_Paint_Edge[1].y,2.0));
			double right_paint = sqrt(pow(roadPaintDataALL[i].Right_Paint_Edge[0].x-roadPaintDataALL[i].Right_Paint_Edge[1].x,2.0)
				+pow(roadPaintDataALL[i].Right_Paint_Edge[0].y-roadPaintDataALL[i].Right_Paint_Edge[1].y,2.0));
		
		}

		cout<<"output finish"<<endl;
		dataStruct.close();
		roadPaintDataALL.clear();


		videoIndex++;
		
		//system("pause");
		//waitKey(-1);
	}
}