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

// function header definitions
void usage(int argc, char* argv[]);

int main(int argc, char* argv[])
{
    // input parameters check
    if (6 > argc)
    {
        usage(argc, argv);
        return -1;
    }

    // parse input parameters
    int parsedArgc = 0;
    char videoListFilename[_MAX_PATH];
    char gpsListFilename[_MAX_PATH];
    char cfgFilename[_MAX_PATH];
    while (parsedArgc < argc)
    {
        if (0 == strcmp(argv[parsedArgc], "-v"))
        {
            strcpy_s(videoListFilename, _MAX_PATH, argv[++parsedArgc]);
            ++parsedArgc;
        }
        else if (0 == strcmp(argv[parsedArgc], "-g"))
        {
            strcpy_s(gpsListFilename, _MAX_PATH, argv[++parsedArgc]);
            ++parsedArgc;
        }
        else
        {
            ++parsedArgc;
        }
    }

    Parameters inParam;
    bool bStatus = readParamRoadScan("../../../../Ford/inVehicle/inVehicle/config/US_Palo_Alto.txt", inParam);

    // Calculate H and H inverse for road scan and traffic sign detection
    ns_roadScan::calHAndInvertH(inParam, H, invertH);

    // open video list and gps list file to check file path
    FILE* videoListFile = NULL;
    FILE* gpsListFile   = NULL;

    errno_t err = fopen_s(&videoListFile, videoListFilename, "r");
    if (0 != err)
    {
        printf("Failed to open video list file %s\n", videoListFilename);
        return -1;
    }

    err = fopen_s(&gpsListFile, gpsListFilename, "r");
    if (0 != err)
    {
        printf("Failed to open video list file %s\n", gpsListFilename);
        return -1;
    }

    char videoFileName[_MAX_PATH];
    char gpsFileName[_MAX_PATH];

    int videoIndex = 0;

    VideoCapture capture;
    FILE* gpsFile = NULL;

    // iterate to get each video and gps file
    while (!feof(videoListFile) && !feof(gpsListFile))
    {
        fscanf_s(videoListFile, "%s\n", videoFileName,  _countof(videoFileName));
        fscanf_s(gpsListFile, "%s\n", gpsFileName,  _countof(gpsFileName));

        capture.open(videoFileName);
        capture.set(CV_CAP_PROP_POS_AVI_RATIO, 1);

        err = fopen_s(&gpsFile, gpsFileName, "r");
        if (0 != err)
        {
            printf("Failed to open video list file %s\n", gpsFileName);
            return -1;
        }
        else
        {
            printf("Opened gps file %s\n", gpsFileName);
        }

        int number_of_frames = (int)(capture.get(CV_CAP_PROP_POS_FRAMES));

        // if not success, exit program
        if (!capture.isOpened())
        {
            printf("Failed to open video file %s\n", videoFileName);
            return -1;
        }
        else
        {
            capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0);

            // get the frames per seconds of the video
            double fps = capture.get(CV_CAP_PROP_FPS);
        }

        ////////////////////////////////////////////////////////////////////////
        // Step1: Kalman initialization

        Size S = Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),
            (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
        S.height = static_cast<int>(S.height * inParam.imageScaleHeight);
        S.width  = static_cast<int>(S.width * inParam.imageScaleWidth);

        vector<dataEveryRow> roadPaintData;
        vector<dataEveryRow> roadPaintDataALL;

        gpsInformationAndInterval gpsAndInterval;
        vector<gpsInformationAndInterval> GPSAndInterval;

        ////////////////////////////////////////////////////////////////////////
        Mat history = Mat::zeros(S.height * HH * SCALE, S.width, CV_8UC1);

        Mat image;
        int rowIndex = 0;
        int intrTmp = 0;
        int frames = 350;

        Point2d gps_next;
        vector<Point2d> gps_points;

        while (!feof(gpsFile))
        {
            fscanf_s(gpsFile, "%lf,%lf\n", &gps_next.x, &gps_next.y);
            gps_points.push_back(gps_next);
        }
        fclose(gpsFile);

        int numOfGps = gps_points.size();

        for (int i = 0; i < 150; i++)
        {
            if ((i * frames + 1) > number_of_frames)
            {
                break;
            }

            capture.set(CV_CAP_PROP_POS_FRAMES, i * frames + 1);
            for (int index = 0; index < frames; index++)
            {
                capture >> image;

                if (image.data && ((i * frames + index + 1) < numOfGps))
                {
                    roadImageGen(image, history,
                        &rowIndex,
                        &gps_points[i * frames + index],
                        &gps_points[i * frames + index + 1],
                        &gpsAndInterval, &intrTmp, inParam);

                    if (gpsAndInterval.intervalOfInterception)
                    {
                        GPSAndInterval.push_back(gpsAndInterval);
                    }

                    if ((index == (frames - 1)) ||
                        ((i * frames + index + 1) == gps_points.size()))
                    {
                        rowIndex -= GPSAndInterval[GPSAndInterval.size() - 1].intervalOfInterception;
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
            intrTmp = 0;

            roadImageProc2(historyROI, GPSAndInterval, roadPaintData, inParam);

            history = Mat::zeros(S.height*HH*SCALE, S.width, CV_8UC1);

            int H = historyROI.rows;

            int numOfPnts = roadPaintData.size();
            for(int i = 0; i < numOfPnts; i++)
            {
                roadPaintDataALL.push_back(roadPaintData[i]);
            }

            roadPaintData.clear();
            GPSAndInterval.clear();
        }

        // save GPS data to files
        char texname[32];
        sprintf_s(texname, 32, "dataStruct_%d.txt", videoIndex);

        ofstream dataStruct(texname);
        dataStruct << setprecision(20) << inParam.GPSref.x << " "
            << inParam.GPSref.y << endl;

        int numOfPnts = roadPaintDataALL.size();
        for(int i = 0; i < numOfPnts; i++)
        {
            dataStruct << setprecision(20)
                << roadPaintDataALL[i].Left_Middle_RelGPS.x << " "
                << roadPaintDataALL[i].Left_Middle_RelGPS.y << " "
                << roadPaintDataALL[i].isPaint_Left << " "
                << roadPaintDataALL[i].Left_Paint_Edge[0].x << " "
                << roadPaintDataALL[i].Left_Paint_Edge[0].y << " "
                << roadPaintDataALL[i].Left_Paint_Edge[1].x << " "
                << roadPaintDataALL[i].Left_Paint_Edge[1].y << " "
                << roadPaintDataALL[i].Left_Area_Pixel_Mean << " "
                << roadPaintDataALL[i].Middle_RelGPS.x << " "
                << roadPaintDataALL[i].Middle_RelGPS.y << " "
                << roadPaintDataALL[i].Middle_Area_Pixel_Mean << " "
                << roadPaintDataALL[i].Right_Middle_RelGPS.x << " "
                << roadPaintDataALL[i].Right_Middle_RelGPS.y << " "
                << roadPaintDataALL[i].isPaint_Right << " "
                << roadPaintDataALL[i].Right_Paint_Edge[0].x << " "
                << roadPaintDataALL[i].Right_Paint_Edge[0].y << " "
                << roadPaintDataALL[i].Right_Paint_Edge[1].x << " "
                << roadPaintDataALL[i].Right_Paint_Edge[1].y << " "
                << roadPaintDataALL[i].Right_Area_Pixel_Mean <<endl;
        }

        cout << "output finish" << endl;
        dataStruct.close();
        roadPaintDataALL.clear();

        printf("GPS output finish %d\n", videoIndex);
        ++videoIndex;
    }

    fclose(videoListFile);
    fclose(gpsListFile);

    return 0;
}


void usage(int argc, char* argv[])
{
    printf("Usage: \n\n");
    printf("\t %s -v videolist.txt -g gpslist.txt -c config.txt\n", "roadDetection.exe");
    printf("\t                      videolist.txt - video file path list file\n");
    printf("\t                      gpslist.txt - gps file path list file\n");
    printf("\t                      config.txt - section configuration file path list file\n");
}