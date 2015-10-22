// video_check.cpp : Defines the entry point for the console application.
//

#include "stdlib.h"
#include <opencv2\opencv.hpp>

using namespace cv;

VideoCapture capture;
char aviNames[50][200];
char gpsNames[50][200];

bool openVideoFile(const char* videoFileName,VideoCapture& capture ,int &numFrame)
{
	capture.open(videoFileName);
	if(!capture.isOpened())
	{
		return false;
	}else
	{
		numFrame = capture.get(CV_CAP_PROP_FRAME_COUNT);
		double fps = capture.get(CV_CAP_PROP_FPS);
		capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0);

	}
	return true;
}

int main(int argc, char* argv[])
{
    /*analyze index file*/
    FILE* fp = fopen("./../../../Ford/inVehicle/inVehicle/config/US_Palo_Alto_aviGpsFiles.txt", "r");
    FILE *result = fopen("./result.txt","w+");
    char *s = "./../../../Ford/inVehicle/inVehicle/";
    char s1[200];
    char s2[200];

    if(fp == NULL)
	{
		printf("cannot open the aviGpsFiles.txt file\n");
        fprintf(result,"Can not open index file!\n");
        fclose(result);
        return 0;
	}

	int readIdx = 0;
	while(!feof(fp))
	{
		if((readIdx&0x1) == 0)
		{
			fscanf(fp,"%s",s1);
            strcpy(aviNames[readIdx>>1],s);
            strcat(aviNames[readIdx>>1],&s1[2]);
		}else
		{
			fscanf(fp,"%s",s2);
            strcpy(gpsNames[readIdx>>1],s);
            strcat(gpsNames[readIdx>>1],&s2[2]);
		}
		readIdx++;

        if(readIdx >= 100)
        {
            printf("The item is full!\n");
            break;
        }
	}
    fclose(fp);

    int numFrame,gps_num;
    int idxFile=0; 

    readIdx = readIdx >> 1;
    while(idxFile < readIdx)
    {
        /*get gps point num*/
        FILE* gpsFile = fopen(gpsNames[idxFile],"r");
        gps_num = 0;
        if(gpsFile == NULL)
	    {
		    printf("cannot open the gps file!\n");
	    }
        else
        {
            while(!feof(gpsFile))
            {
                if(fgetc(gpsFile)==','){gps_num++;}
            }
            fclose(gpsFile);
        }

        /*get video frame num*/
        numFrame = 0;
        if( !openVideoFile(aviNames[idxFile],capture ,numFrame))
	    {
		    printf("can't open file: %s",aviNames[idxFile]);
	    }

        /*check and output the result*/
        if(numFrame == gps_num)
        {
            if(0 == gps_num)
            {
                fprintf(result,"Index %d:file open failed!\n",idxFile);
            }
            else
            {
                fprintf(result,"Index %d:OK\n",idxFile);
            }
            
        }
        else
        {
            if(0 == numFrame*gps_num)
            {fprintf(result,"Index %d:file open failed!\n",idxFile);}
            else
            {fprintf(result,"Index %d:ERROR\n",idxFile);}
        }

        idxFile++;
    }
    fclose(result);

	return 0;
}

