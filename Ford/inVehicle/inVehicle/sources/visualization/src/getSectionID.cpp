/******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  detection.h
* @brief Road furniture detection header file
*
* Change Log:
*      Date                Who             What
*      2015/09/09          Qian Xu         Create
*******************************************************************************
*/
#include <list>
#include <vector>
#include<algorithm>  
#include "string"
#include "windows.h"
#include "database.h"


#define MAX_DIST_BETWEEN_CAR_AND_LANE   20.0

using namespace std;
using namespace ns_database;

bool readSectionConfig(string configPath,list<segAttributes_t> &segCfgList)
{
	int sectionNum = 0;
	int sectionID  = 0;
	int laneNum = 0;

	segAttributes_t segmentElement;
	segmentElement.segId_used          = 0;
	segmentElement.version_used        = 0;
	segmentElement.type_used           = 0;
	segmentElement.numPort_used        = 0;
	segmentElement.ports_used          = 0;
	segmentElement.links_used          = 0;
	segmentElement.roadLength_used     = 0;
	segmentElement.bridgeFlag_used     = 0;
	segmentElement.tunnelFlag_used     = 0;
	segmentElement.numFurniture_used   = 0;
	segmentElement.numDynamicData_used = 0;

	FILE *fp = nullptr;
	errno_t err = fopen_s(&fp, configPath.c_str(), "rt");
	if(0 != err)
	{
		fclose(fp);
        return false;
	}

	fscanf(fp,"%*[^\n]%*c");
	fscanf(fp,"%*[^\n]%*c");

	while (!feof(fp))
	{
		fscanf_s(fp, "%d,%d\n", &sectionID, &laneNum);
		segmentElement.segId_used = 1;
		segmentElement.segId      = sectionID;

		fscanf_s(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
			&segmentElement.ports[0].lon, &segmentElement.ports[0].lat,
			&segmentElement.ports[1].lon, &segmentElement.ports[1].lat,
			&segmentElement.ports[2].lon, &segmentElement.ports[2].lat,
			&segmentElement.ports[3].lon, &segmentElement.ports[3].lat);

		segCfgList.push_back(segmentElement);
	}
	fclose(fp);
	return true;
}


//called by getSectionId
double getLength1(IN double point1_x,IN double point1_y, IN double point2_x, IN double point2_y)
{
	double diff_x = abs(point2_x - point1_x);      
	double diff_y = abs(point2_y - point1_y);        
	double length = sqrt(long double(diff_x* diff_x+ diff_y*diff_y));
	return length;
}

//called by getSectionId
int GetAngle1(IN point3D_t samplePoint,
              IN point3D_t sectionPoint,
              IN point3D_t leftPoint,
              IN point3D_t rightPoint,
              OUT float &cos_left,
              OUT float &cos_right)
{
     double length_section_sample = getLength1(sectionPoint.lon, sectionPoint.lat, samplePoint.lon, samplePoint.lat);
     double length_section_left = getLength1(sectionPoint.lon, sectionPoint.lat, leftPoint.lon, leftPoint.lat);      
     double length_sample_left = getLength1(samplePoint.lon, samplePoint.lat,leftPoint.lon, leftPoint.lat);     
     double length_section_right =  getLength1(sectionPoint.lon, sectionPoint.lat,rightPoint.lon, rightPoint.lat); 
     double length_sample_right =  getLength1(samplePoint.lon, samplePoint.lat,rightPoint.lon, rightPoint.lat); 
     cos_left =( length_section_sample*length_section_sample+ length_section_left*length_section_left - length_sample_left*length_sample_left) / (length_section_sample*length_section_left*2); 
     cos_right = ( length_section_sample*length_section_sample+ length_section_right*length_section_right - length_sample_right*length_sample_right) / (length_section_sample*length_section_right*2);
     return true;
}

uint32 getSectionId(IN point3D_t inPoint,
                    IN list<segAttributes_t> &segConfigList,
                    OUT uint32 &foundSectionID)
{
	//step 1
	BOOL closedLoopFlag;
	vector<point3D_t> bodySectionPoint;

	if (!bodySectionPoint.empty())
	{
		bodySectionPoint.clear();
	}

	list<segAttributes_t>::iterator itrSecConfig = segConfigList.begin();

	while (itrSecConfig!=segConfigList.end())
	{
		segAttributes_t tmpSec = *itrSecConfig;
		point3D_t tmpBodySecPoint;
		tmpBodySecPoint.lat = tmpSec.ports[0].lat;
		tmpBodySecPoint.lon = tmpSec.ports[0].lon;
		bodySectionPoint.push_back(tmpBodySecPoint);
		itrSecConfig++;
	}

	segAttributes_t firstSection,lastSection;
	firstSection = segConfigList.front();
	lastSection = segConfigList.back();

	if ((firstSection.ports[0].lat==lastSection.ports[1].lat)
		&&(firstSection.ports[0].lon==lastSection.ports[1].lon))
	{
		closedLoopFlag = 1;
	} 
	else
	{
		closedLoopFlag = 0;
		point3D_t endSecPoint;
		segAttributes_t endSec = *itrSecConfig;
		endSecPoint.lat = endSec.ports[1].lat;
		endSecPoint.lon = endSec.ports[1].lon;
		bodySectionPoint.push_back(endSecPoint);
	}

	//step2
	foundSectionID = 0;
	vector<double> distance(bodySectionPoint.size());
	for (unsigned int i=0;i<=distance.size()-1;i++)
	{
		distance[i] = getLength1(inPoint.lon,inPoint.lat,
			bodySectionPoint[i].lon,bodySectionPoint[i].lat);
	}
	uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

	//get prevSectionID & nextSectionID of the minDist sectionID
	uint32 minDisSectionID = min_pos+1;
	uint32 prevSectionID = minDisSectionID-1;
	uint32 nextSectionID = minDisSectionID+1;
	if (minDisSectionID==1)
	{
		prevSectionID = bodySectionPoint.size();
	}
	if (minDisSectionID==bodySectionPoint.size())
	{
		nextSectionID = 1;
	}

	//compare two angles
	float cos_left = 1 ,cos_right = 1;
	GetAngle1(inPoint,bodySectionPoint[min_pos],bodySectionPoint[prevSectionID-1],
		bodySectionPoint[nextSectionID-1],cos_left,cos_right);

	if (cos_left<0 && cos_right>0)
	{
		foundSectionID = minDisSectionID;
	} 
	else
	{
		if (cos_left>0 && cos_right<0)
		{
			foundSectionID = prevSectionID;
		} 
		else
		{
			foundSectionID = prevSectionID;
		}
	}

	//step3
	if ( closedLoopFlag==0 && foundSectionID==bodySectionPoint.size() )
	{
		foundSectionID = 0;
	}

	return 1;
}

bool fixVehicleLocationInLane(point3D_t reletiveGps,list<vector<point3D_t>> &allLines, point3D_t *locationInLane)
{
    bool locationFlag = false;

    if(allLines.size() >= 2) // no lane
    {
        int counter = 0;
        list<vector<point3D_t>>::iterator lineIdx = allLines.begin();
        vector<float> minDistPerLine;
        vector<point3D_t> pointIdxPerLine;
        // calculate the nearest points between  the vehicle and lines.
        while(lineIdx != allLines.end())
        {
            float minDist = 9e+10;
            point3D_t   minIdx;
            for(int pointIdx = 0; pointIdx < lineIdx->size();++pointIdx)
            {
                point3D_t currentPoint = (*lineIdx)[pointIdx];
                float distX = reletiveGps.lat - currentPoint.lat;
                float distY = reletiveGps.lon - currentPoint.lon;
                float dist  = (distX*distX + distY*distY);
                if(dist < minDist)
                {
                    minDist = dist;
                    minIdx  = currentPoint;
                }
            }
            minDistPerLine.push_back(minDist);
            pointIdxPerLine.push_back(minIdx);
            ++lineIdx;
            ++counter;
            //if(counter > allLines.size()/2)
            //{
            //    break;
            //}
        }

        // find the two point being nearest the vehicle
        float minDistPoint = 9e+10;
        float minPointIdx;

        if(minDistPerLine.size() == 2) // only one lane
        {

            point3D_t currentPoint1 = pointIdxPerLine[0];
            point3D_t currentPoint2 = pointIdxPerLine[1];

            locationInLane->lat = (currentPoint1.lat + currentPoint2.lat) * 0.5;
            locationInLane->lon = (currentPoint1.lon + currentPoint2.lon) * 0.5;
        }
        else // lane number > 2
        {
            float minDist1 = minDistPerLine[0];
            int minIndex1 = 0;
            for(int pointIdx = 1; pointIdx < minDistPerLine.size();++pointIdx)
            {
                if(minDist1 > minDistPerLine[pointIdx])
                {
                    minDist1 = minDistPerLine[pointIdx];
                    minIndex1 = pointIdx;
                }
            }

            minDistPerLine[minIndex1] = 9e+10;

            float minDist2 = minDistPerLine[0];
            int minIndex2 = 0;
            for(int pointIdx = 1; pointIdx < minDistPerLine.size();++pointIdx)
            {
                if(minDist2 > minDistPerLine[pointIdx])
                {
                    minDist2 = minDistPerLine[pointIdx];
                    minIndex2 = pointIdx;
                }
            }
           point3D_t point1 = pointIdxPerLine[minIndex1];
           point3D_t point2 = pointIdxPerLine[minIndex2];
           locationInLane->lat = (point1.lat + point2.lat)*0.5;
           locationInLane->lon = (point1.lon + point2.lon)*0.5;
        }//end if(minDistPerLine.size() == 2)
        
        // check whether the distance between vehicle and lane middle point is too far: > 20M
        {
            float distX = locationInLane->lat - reletiveGps.lat;
            float distY = locationInLane->lon - reletiveGps.lon;
            float dist = sqrt(distX*distX + distY*distY);
            if(dist < MAX_DIST_BETWEEN_CAR_AND_LANE)
            {
                locationFlag = true;
            }
        }
    }
    return locationFlag;
}

bool checkLineSection(list<list<lineAttributes_t>> &lineAttr, int sectionId, int &LineNum)
{
  list<list<lineAttributes_t>>::iterator lineAttrInSegIter = lineAttr.begin();
  int Idx = 0;

  while(lineAttrInSegIter != lineAttr.end())
  {
   if((*lineAttrInSegIter).size() > 0)
   {
    list<lineAttributes_t>::iterator lineAttrIter = (*lineAttrInSegIter).begin();
    lineAttributes_t temp = (*lineAttrIter);
    if(temp.segmentId == sectionId)
    {
     LineNum = Idx;
     return true;
    }
   }
   lineAttrInSegIter++;
   Idx++;
  }

  return false;
}
