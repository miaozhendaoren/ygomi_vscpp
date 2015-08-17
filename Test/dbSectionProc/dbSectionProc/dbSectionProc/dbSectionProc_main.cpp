#include <stdio.h>
#include <windows.h>
#include <math.h>
#include <opencv2\opencv.hpp>
#include "database.h"
#include "databaseDef.h"
#include "databaseServer.h"

using std::list;
using std::vector;

using namespace ns_database;
using namespace cv;

int sectionPointsNum = 8;
ns_database::databaseServer database_gp;

//list<vector<point3D_t>> sectionConfig;
list<segAttributes_t> sectionConfig;
list<vector<point3D_t>> newDataVec;
list<list<vector<point3D_t>>> allLines;
list<list<vector<point3D_t>>> pointMerged;

struct Scale
{
	float minXVal;
	float maxXVal;
	float minYVal;
	float maxYVal;
};
struct Offset
{
	float X;
	float Y;
};
void calculateShowImageScale(IN list<segAttributes_t> &sectionConfig,OUT Scale imageScale,OUT Offset XYoffset);
void displayData(list<segAttributes_t> &sectionConfig);
void readFilesectionConfig(char *fileName, list<segAttributes_t> &sectionConfig)
{
	char buffer[100];
	int sectionNum = 0;
	int sectionID = 0;

	segAttributes_t segmentElement;

	segmentElement.segId_used = 0;
    segmentElement.version_used = 0;
    segmentElement.type_used = 0;
    segmentElement.numPort_used = 0;
    segmentElement.ports_used = 0;
    segmentElement.links_used = 0;
    segmentElement.roadLength_used = 0;
    segmentElement.bridgeFlag_used = 0;
    segmentElement.tunnelFlag_used = 0;
    segmentElement.numFurniture_used = 0;
    segmentElement.numDynamicData_used = 0;

	FILE *fp;
	fp = fopen(fileName, "rt");
	if(NULL == fp)
	{
		return;
	}
	fscanf(fp, "%d\n", &sectionNum);
	for(int index=0; index<sectionNum; index++)
	{
		fscanf(fp, "%d\n", &sectionID);
		segmentElement.segId_used = 1;
		segmentElement.segId = sectionID;
		fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &segmentElement.ports[0].lon, &segmentElement.ports[0].lat,&segmentElement.ports[1].lon, &segmentElement.ports[1].lat,&segmentElement.ports[2].lon, &segmentElement.ports[2].lat,&segmentElement.ports[3].lon, &segmentElement.ports[3].lat,
			&segmentElement.ports[4].lon, &segmentElement.ports[4].lat,&segmentElement.ports[5].lon, &segmentElement.ports[5].lat,&segmentElement.ports[6].lon, &segmentElement.ports[6].lat,&segmentElement.ports[7].lon, &segmentElement.ports[7].lat);

		sectionConfig.push_back(segmentElement);
	}
	displayData(sectionConfig);
}
void displayData(list<segAttributes_t> &sectionConfig)
{
	Scale imageScale = {0,0,0,0};
	Offset XYoffset = {0,0};
	calculateShowImageScale(sectionConfig,imageScale,XYoffset);
	float X_start = imageScale.minXVal + XYoffset.X;
	float X_end = imageScale.maxXVal + XYoffset.X;

	float Y_start = imageScale.minYVal + XYoffset.Y;
	float Y_end = imageScale.maxXVal + XYoffset.Y;
	
	int X = (int)(X_end - X_start);
	int Y = (int)(Y_end - Y_start);

	Mat picture(X,Y,CV_8UC3,Scalar(255,255,255));
	list<segAttributes_t>::iterator iter = sectionConfig.begin();
	while(iter!= sectionConfig.end())
	{

		Point center;
		center.x= iter->ports[0].lon;
		center.y= iter->ports[0].lat+800;
		circle(picture,center,0,Scalar(255,0,0),5);
		++iter;
	}
	imwrite("Road.png",picture);
	//waitKey(60); 
}
void calculateShowImageScale(IN list<segAttributes_t> &sectionConfig,OUT Scale imageScale,OUT Offset XYoffset)
{
	//1.Get each section minimum Longitude maximum Longitude minimal Latitude maximum Latitude
	float tempMaxLon = 0,
		  tempMinLon = 0,
		  tempMaxLat = 0,
		  tempMinLat = 0;
	list<vector<float>> ltImageScale;
	vector<float> vLonLocalMinVal,vLonLocalMaxVal,vLatLocalMinVal,vLatLocalMaxVal;
	list<segAttributes_t>::iterator iter = sectionConfig.begin();
	while(iter!= sectionConfig.end())
	{
		int i = 0;
		tempMaxLon = iter->ports[0].lon;
		tempMinLon = iter->ports[0].lon;

		tempMaxLat = iter->ports[0].lat;
		tempMinLat = iter->ports[0].lat;

		while (i<8)
		{
			i++;
			tempMaxLon = tempMaxLon < iter->ports[i].lon ? iter->ports[i].lon : tempMaxLon;
			tempMinLon = tempMinLon > iter->ports[i].lon ? iter->ports[i].lon : tempMinLon;

			tempMaxLat = tempMaxLat < iter->ports[i].lat ? iter->ports[i].lat : tempMaxLat;
			tempMinLat = tempMinLat > iter->ports[i].lat ? iter->ports[i].lat : tempMinLat;
		}
		vLonLocalMaxVal.push_back(tempMaxLon);
		vLonLocalMinVal.push_back(tempMinLon);
		
		vLatLocalMaxVal.push_back(tempMaxLat);	
		vLatLocalMinVal.push_back(tempMinLat);
		iter ++ ;
	}
	//2.Get whole show image the minimum Longitude maximum Longitude minimal Latitude maximum Latitude
	vector<float>::size_type size = vLonLocalMinVal.size();
	float minLonVal = 0,
		  maxLonVal = 0,
		  minLatVal = 0,
		  maxLatVal = 0;

	for(int i = 0;i < size; i++)
	{
		maxLonVal = maxLonVal < vLonLocalMaxVal[i] ? vLonLocalMaxVal[i] : maxLonVal;
		minLonVal = minLonVal > vLonLocalMinVal[i] ? vLonLocalMinVal[i] : minLonVal;
		
		maxLatVal = maxLatVal < vLatLocalMaxVal[i] ? vLatLocalMaxVal[i] : maxLatVal;
		minLatVal = minLatVal > vLatLocalMinVal[i] ? vLatLocalMinVal[i] : minLatVal;
		
	}
	if (minLonVal < 0)
		XYoffset.X = fabs(minLonVal);
	if (minLatVal < 0)
	    XYoffset.Y = fabs(minLatVal);

	imageScale.maxXVal = maxLonVal;
	imageScale.minXVal = minLonVal;
	imageScale.maxYVal = maxLatVal;
	imageScale.minYVal = minLonVal;
}

void main()
{
	readFilesectionConfig("F:/PROJECTS/roadDB/ConfigFile/manualSeg.txt", sectionConfig);
	database_gp.sectionProc(sectionConfig, 
                newDataVec,
                allLines,
                pointMerged);
}