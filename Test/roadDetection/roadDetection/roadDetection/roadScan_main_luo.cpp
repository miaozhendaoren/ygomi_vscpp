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
#define Honda3  8

using namespace std;
using namespace cv;
using namespace ns_roadScan;
cv::Mat H, invertH;

char aviNames[50][100];
char gpsNames[50][100];

//#define IMAGE_SENSOR_WIDTH 640


int main(void)
{
    //ifstream readParam("England.txt",ios::_Nocreate);
    //ifstream readParam("VW.txt",ios::_Nocreate);DE_Lehre
    //ifstream readParam("HD.txt",ios::_Nocreate);DE_Airport2
    //ifstream readParam("US.txt",ios::_Nocreate);US_Detroit
    //US_Palo_Alto
    Parameters inParam;
    int chooseVideo = Airport2;
    bool readStatus;
    FILE* fp;

    if (chooseVideo == Honda)
    {
        readStatus = readParamRoadScan("../../../../Ford/inVehicle/inVehicle/config/US_Palo_Alto.txt", inParam);
        fp = fopen("HomdaaviGpsFiles.txt", "r");
    }
    else if (chooseVideo == Airport2)
    {
        readStatus = readParamRoadScan("../../../../Ford/inVehicle/inVehicle/config/DE_Airport2.txt", inParam);
        fp = fopen("AirportaviGpsFiles.txt", "r");
    }
    else if (chooseVideo == Ford)
    {
        readStatus = readParamRoadScan("../../../../Ford/inVehicle/inVehicle/config/US_Detroit.txt", inParam);    
        fp = fopen("FordaviGpsFiles.txt", "r");
    }   
    else
        ;
    ns_roadScan::calHAndInvertH(inParam, H, invertH);
    int numFiles;
    int idxFile = 0;
    int readIdx = 0;
    while(!feof(fp))
    {
        if((readIdx&0x1) == 0)
        {
            fscanf(fp,"%s",aviNames[readIdx>>1]);
        }else
        {
            fscanf(fp,"%s",gpsNames[readIdx>>1]);
        }
        readIdx++;
    }
    numFiles = (readIdx>>1);
    fclose(fp);

    for (int videoIndex = 0;videoIndex<numFiles;videoIndex++)
    {
        VideoCapture capture;
        string video = aviNames[videoIndex];
        printf("select video %s\n",aviNames[videoIndex]);
        FILE* gpsFile = fopen(gpsNames[videoIndex],"r");    
        capture.open(aviNames[videoIndex]);

        int number_of_frames = capture.get(CV_CAP_PROP_FRAME_COUNT);

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

        Size S = Size(inParam.imageCols,inParam.imageRows);

        vector<dataEveryRow> roadPaintData;
        vector<dataEveryRow> roadPaintDataAll;
        vector<gpsInformationAndInterval> GPSAndInterval;
        vector<landMark> vecLandMark;
        vector<landMark> vecLandMarkAll;

        ////////////////////////////////////////////////////////////////////////////
        Mat history = Mat::zeros(S.height *HH*SCALE,S.width*inParam.imageScaleWidth, CV_8UC1);

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
        int frames = 300;
        vector<Point2d> gps_points;
        while(!feof(gpsFile))
        {
            fscanf(gpsFile,"%lf,%lf\n",&GPS_next.x,&GPS_next.y);
            gps_points.push_back(GPS_next);
        }
        fseek(gpsFile, 0, SEEK_SET);
        for (int n=1;n<150;n++)
        {

            cout<<"video="<<videoIndex<<endl;
            cout<<"interval="<<n<<endl;
            if (n*frames+1>number_of_frames)
            {
                cout<<"endl";
                break;

            }
            Point2d GPS_stop = Point2d(0,0);
            bool stopFlg = false;
            capture.set(CV_CAP_PROP_POS_FRAMES, n*frames+1);
            for(int index = 0; index < frames; index++)//number_of_frames
            {
                //		cout<<index<<endl;
                capture >> image;
                if (image.data&& n*frames+index+1<gps_points.size())
                {
                    roadImageGen(image, history, &rowIndex, &gps_points[n*frames+index], &gps_points[n*frames+index+1], &gpsAndInterval, &intrtmp, inParam, GPS_stop,stopFlg);

                    if (gpsAndInterval.intervalOfInterception)
                    {
                        GPSAndInterval.push_back(gpsAndInterval);
                    }
                    if(index==frames-1||n*frames+index+1==gps_points.size())
                    {
                        if (GPSAndInterval.size()>0)
                        {
                            rowIndex -= GPSAndInterval[GPSAndInterval.size()-1].intervalOfInterception;
                        }                      
                    }

                }
                else
                    break;
            }
            Mat historyROI = history(Rect(0,rowIndex,history.cols,history.rows-rowIndex));
            imwrite("historyroi.png",historyROI);

            rowIndex=0;
            intrtmp=0;
            roadImageProc2(historyROI, inParam, GPSAndInterval, roadPaintData, vecLandMark);
            history = Mat::zeros(history.rows,history.cols, CV_8UC1);

            int H = historyROI.rows;

            for(int i = 0; i<roadPaintData.size(); i++)
            {
                roadPaintDataAll.push_back(roadPaintData[i]);
            }

            for(int i = 0; i<vecLandMark.size(); i++)
            {
                vecLandMarkAll.push_back(vecLandMark[i]);
            }

            vecLandMark.clear();
            roadPaintData.clear();
            GPSAndInterval.clear();	
        }

        char texname[32];    
        sprintf(texname,"paintingData_%d.txt",videoIndex);  
        ofstream paintingData(texname);

        char landmarkname[32];    
        sprintf(landmarkname,"landmarker_%d.txt",videoIndex);  
        ofstream landmarkData(landmarkname);

        for(int i = 0; i<roadPaintDataAll.size(); i++)
        {		
            paintingData<<setprecision(20)<<roadPaintDataAll[i].Left_Middle_RelGPS.x<<" "<<roadPaintDataAll[i].Left_Middle_RelGPS.y<<" "<<roadPaintDataAll[i].isPaint_Left<<" "
                <<roadPaintDataAll[i].Left_Paint_Edge[0].x<<" "<<roadPaintDataAll[i].Left_Paint_Edge[0].y<<" "
                <<roadPaintDataAll[i].Left_Paint_Edge[1].x<<" "<<roadPaintDataAll[i].Left_Paint_Edge[1].y<<" "
                <<roadPaintDataAll[i].Left_Area_Pixel_Mean<<" "
                <<roadPaintDataAll[i].Middle_RelGPS.x<<" "<<roadPaintDataAll[i].Middle_RelGPS.y<<" "<<roadPaintDataAll[i].Middle_Area_Pixel_Mean<<" "
                <<roadPaintDataAll[i].Right_Middle_RelGPS.x<<" "<<roadPaintDataAll[i].Right_Middle_RelGPS.y<<" "<<roadPaintDataAll[i].isPaint_Right<<" "
                <<roadPaintDataAll[i].Right_Paint_Edge[0].x<<" "<<roadPaintDataAll[i].Right_Paint_Edge[0].y<<" "
                <<roadPaintDataAll[i].Right_Paint_Edge[1].x<<" "<<roadPaintDataAll[i].Right_Paint_Edge[1].y<<" "
                <<roadPaintDataAll[i].Right_Area_Pixel_Mean<<endl;       
        }

        for(int i = 0; i<vecLandMarkAll.size(); i++)
        {		
            landmarkData<<setprecision(20)<<vecLandMarkAll[i].centerRel.x<<" "<<vecLandMarkAll[i].centerRel.y
                <<" "<<vecLandMarkAll[i].center.x<<" "<<vecLandMarkAll[i].center.y<<endl;       
        }
        cout<<"output finish"<<endl;
        paintingData.close();
        landmarkData.close();
        roadPaintDataAll.clear();
        vecLandMarkAll.clear();

    }

}

