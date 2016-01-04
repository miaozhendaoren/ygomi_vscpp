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
#include "getsectionID.h"

#define MAX_DIST_BETWEEN_CAR_AND_LANE   20.0
#define MAX_POINT_TO_SEC_DIST     500
#define REVERSE_FLAG_FILTER_ON    1

using namespace std;
using namespace ns_database;
using namespace ns_roadsegment;

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
    segmentElement.uiLaneNum_used      = 0;
    segmentElement.loopIdx_used        = 0;

	FILE *fp = nullptr;
	errno_t err = fopen_s(&fp, configPath.c_str(), "rt");
	if(0 != err)
	{
		fclose(fp);
        return false;
	}

	while (!feof(fp))
	{
		fscanf_s(fp, "segId: %d,laneNum: %d\n", &sectionID, &laneNum);
		segmentElement.segId_used = 1;
		segmentElement.segId      = sectionID;

        segmentElement.loopIdx_used = 1;
    	segmentElement.loopIdx      = 0;

	    fscanf(fp,"%*[^\n]%*c");
	    fscanf(fp,"%*[^\n]%*c");

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
double getLength(IN double point1_x,IN double point1_y, IN double point2_x, IN double point2_y)
{
	double diff_x = abs(point2_x - point1_x);      
	double diff_y = abs(point2_y - point1_y);        
	double length = sqrt(long double(diff_x* diff_x+ diff_y*diff_y));
	return length;
}

int rotPoint(IN point3D_t sourcePoint,IN double theta,OUT point3D_t &rotedPoint)
{
	rotedPoint = sourcePoint;
	double x = sourcePoint.lon, y = sourcePoint.lat, realZ = 0, imagZ = 0;
	complex<double> thetaj(0, theta);
	complex<double> yj(0, y);
	realZ = real((x + yj) * exp(thetaj));
	imagZ = imag((x + yj) * exp(thetaj));
	rotedPoint.lon = realZ;
	rotedPoint.lat = imagZ;
	rotedPoint.alt = 0;
	return true;
}

#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE) 
double getAngle(IN point3D_t samplePoint,
	            IN point3D_t sectionPoint,
	            IN point3D_t connPoint)
{
	double a = getLength(sectionPoint.lon, sectionPoint.lat, samplePoint.lon, samplePoint.lat);
	double b = getLength(sectionPoint.lon, sectionPoint.lat, connPoint.lon, connPoint.lat);      
	double c = getLength(samplePoint.lon, samplePoint.lat,connPoint.lon, connPoint.lat);     

	double cosVal = (0.0 == a*b) ? 1.0 : (( a*a+ b*b - c*c) / (a * b * 2)); 
	return cosVal;
}

void getMinDistSegId( IN  point3D_t  inPtGps, 
	                  IN  vector<uint32> compSegList,
					  IN  vector<secCfgInfo_t> secCfgInfo,
					  IN  vector<point3D_t> bodyPts3D,
	                  OUT uint32& segId )
{
	double minH = DBL_MAX;
	vector<uint32>::iterator segListIter = compSegList.begin();

	// For each segment
	while(segListIter != compSegList.end())
	{
		// TODO: locate the segment of the GPS
		double lat1 = bodyPts3D[secCfgInfo[(*segListIter)-1].prePointId].lat;
		double lat2 = bodyPts3D[secCfgInfo[(*segListIter)-1].nextPointId].lat;

		double lon1 = bodyPts3D[secCfgInfo[(*segListIter)-1].prePointId].lon;
		double lon2 = bodyPts3D[secCfgInfo[(*segListIter)-1].nextPointId].lon;

		// calculate angle
		double aPower2 = (inPtGps.lat - lat1) * (inPtGps.lat - lat1) + (inPtGps.lon - lon1) * (inPtGps.lon - lon1);
		double bPower2 = (inPtGps.lat - lat2) * (inPtGps.lat - lat2) + (inPtGps.lon - lon2) * (inPtGps.lon - lon2);
		double cPower2 = (lat1 - lat2) * (lat1 - lat2) + (lon1 - lon2) * (lon1 - lon2);
		double a = sqrt(aPower2); double b = sqrt(bPower2); double c = sqrt(cPower2);

		//cosA = (a^2+b^2-c^2)/2ab
		double cosA = (aPower2 + cPower2 - bPower2);
		double cosB = (bPower2 + cPower2 - aPower2);

		// S=sqrt(p(p-a)(p-b)(p-c)), p = (a+b+c)/2
		double p  = (a + b + c) / 2;
		double S = sqrt(p*(p-a)*(p-b)*(p-c));

		double h = 2 * S / c;

		if(minH > h)
		{
			minH = h;
			segId = (*segListIter);
		}

		++segListIter;
	}
}

bool getSectionId(IN point3D_t inPoint,
				  IN int currentloopIdx,
                  OUT uint32 &segId)
{
	static uint32 segIdHis = 0;
	vector<uint32> curLoopPtsId;
#if defined(_FRANKFORT_ALL_CAMERA)	
	vector<uint32> curLoopPtsId1;
	roadSegConfig_gp->getLoopPointInfo( 0 , curLoopPtsId );
	roadSegConfig_gp->getLoopPointInfo( 2 , curLoopPtsId1 );
	curLoopPtsId.insert(curLoopPtsId.end(),curLoopPtsId1.begin(),curLoopPtsId1.end());
#else
	roadSegConfig_gp->getLoopPointInfo( currentloopIdx , curLoopPtsId );
#endif
	vector<secPointInfo_t> secPointInfo;
	roadSegConfig_gp->getsecPointInfo( secPointInfo );

	vector<point3D_t> bodyPts3D;
	roadSegConfig_gp->getsecPointVector( bodyPts3D );

	vector<secCfgInfo_t> secCfgInfo;
	roadSegConfig_gp->getSecCfgInfo(secCfgInfo);

	//get the minDist sectionID 
	vector<double> distance;
	int i,dist;
	point3D_t curPt;
	uint32 min_pos;
	double min_Dist = MAX_POINT_TO_SEC_DIST;
	for ( i = 0;i < curLoopPtsId.size();i++)
	{
		if ( curLoopPtsId[i] < bodyPts3D.size() )
		{
			curPt = bodyPts3D[curLoopPtsId[i]];
		} 
		else
		{
			segId = 0;
			return false;
		}

		dist = getLength( inPoint.lon, inPoint.lat, curPt.lon, curPt.lat );
		if ( dist <= min_Dist )
		{
			min_Dist = dist;
			min_pos = i;
		}
	}
	if ( min_Dist >= MAX_POINT_TO_SEC_DIST )
	{
		segId = 0;
		return false;
	} 

	uint32 minPtID = curLoopPtsId[min_pos];
	point3D_t minPt3D =  bodyPts3D[minPtID];
	vector<uint32> connPtsId = secPointInfo[minPtID].connPtsId;
	vector<point3D_t> connPts3D;
	for (int i = 0;i<connPtsId.size();i++)
	{
		uint32 connPtId  = connPtsId[i];
		connPts3D.push_back(bodyPts3D[connPtId]);
	}
	vector<uint32> connSections = secPointInfo[minPtID].connSecs;
	if ( connPtsId.size()==1 )  //road start and end point
	{
		double cosVal = getAngle(inPoint,minPt3D,connPts3D[0]);
		if (cosVal < 0)
		{
			segId = 0;  //set road data before start and end section belongs section 0
			segIdHis = 0;
			return true;
		} 
		else
		{
			segId = connSections[0];
		}
	} 
	else
	{
		int connPointsNum = connPts3D.size();
		point3D_t connPt3D;
		double maxCosVal = -1;
		uint32 minDegreeLoc;
		for (int i = 0;i<connPointsNum;i++)
		{
			connPt3D = connPts3D[i];
			double cosVal = getAngle(inPoint,minPt3D,connPt3D);
			if ( cosVal >= maxCosVal )
			{
				maxCosVal = cosVal;
				minDegreeLoc = i;
			}
		}
		segId = connSections[minDegreeLoc];
	}

	if ( segIdHis == 0 || segIdHis == segId )
	{
		segIdHis = segId;
		return true;
	}
	else
	{
		if ( (secCfgInfo[segIdHis-1].secType == TROAD_CROSSING_AR_E || secCfgInfo[segIdHis-1].secType == CROSSING_AR_E) &&
			 (secCfgInfo[segId-1].secType == TROAD_CROSSING_RA_E || secCfgInfo[segId-1].secType == CROSSING_RA_E) )
		{
			segId = segIdHis;
			return true;
		}
		else if ( (secCfgInfo[segIdHis-1].secType == TROAD_CROSSING_RA_E || secCfgInfo[segIdHis-1].secType == CROSSING_RA_E) &&
			      (secCfgInfo[segId-1].secType == TROAD_CROSSING_AR_E || secCfgInfo[segId-1].secType == CROSSING_AR_E) )
		{
			segId = segIdHis;
			return true;
		}
		else
		{
			vector<uint32> preSeg = secCfgInfo[segId-1].prevSegId;
			vector<uint32> nextSeg = secCfgInfo[segId-1].nextSegId;
			int i;
			for ( i = 0; i < preSeg.size(); i++ )
			{
				if ( preSeg[i] == segIdHis )
				{
					segIdHis = segId;
					return true;
				}
			}
			for ( i = 0; i < nextSeg.size(); i++ )
			{
				if ( nextSeg[i] == segIdHis )
				{
					segIdHis = segId;
					return true;
				}
			}

			vector<uint32> compSegIdList;
			compSegIdList.push_back(segId);
			compSegIdList.push_back(segIdHis);

			getMinDistSegId( inPoint, compSegIdList, secCfgInfo, bodyPts3D, segId );

			segIdHis = segId;
			return true;
		}
	}
}

bool getSegIdAndDirection(IN point3D_t inPoint1,
	                      IN point3D_t inPoint2,
						  IN int currentloopIdx,
	                      OUT uint32 &segId,
	                      OUT bool &reverseFlag)
{
#if REVERSE_FLAG_FILTER_ON == 1
	static uint32 segIdHis = 0;
	static bool reverseFlagHis = false;
	static int revContDiffNum = 0;
	static int initHis = 0;
#endif

	getSectionId( inPoint1,currentloopIdx,segId );

	vector<secCfgInfo_t> secCfgInfo;
	roadSegConfig_gp->getSecCfgInfo(secCfgInfo);
	vector<point3D_t> bodyPts3D;
	roadSegConfig_gp->getsecPointVector(bodyPts3D);

	if ( 0 == segId )
	{
		reverseFlag = false;
		return false;
	}
	else 
	{
		point3D_t bodyLeftPt,bodyRightPt,rotSrcLeft,rotSrcRight,rotPt1,rotPt2;

		//get section body left point and body right point
		int leftPtId = secCfgInfo[segId-1].prePointId;
		bodyLeftPt = bodyPts3D[leftPtId];
		int rightPtId = secCfgInfo[segId-1].nextPointId;
		bodyRightPt = bodyPts3D[rightPtId];

		//get section angle and compare
		double theta;
		theta = -atan((bodyRightPt.lat - bodyLeftPt.lat)/(bodyRightPt.lon - bodyLeftPt.lon + 0.0000000001));
		rotPoint(bodyLeftPt,theta,rotSrcLeft);
		rotPoint(bodyRightPt,theta,rotSrcRight);
		rotPoint(inPoint1,theta,rotPt1);
		rotPoint(inPoint2,theta,rotPt2);

		//is the point2->point1 the same direction with section body left->right 
		//the same direction: reverseFlag = 0; the reverse direction: reverseFlag = 1
		reverseFlag = ( rotPt2.lon - rotPt1.lon ) * ( rotSrcLeft.lon - rotSrcRight.lon ) > 0 ? FALSE : TRUE;

#if REVERSE_FLAG_FILTER_ON == 1
		if ( 0 == initHis )
		{
			segIdHis = segId;
			reverseFlagHis = reverseFlag;
			initHis++;
			return true;
		}

		if ( segIdHis == segId && reverseFlag != reverseFlagHis )
		{
			revContDiffNum++;
#if defined(_FRANKFORT_ALL_CAMERA)
			if ( revContDiffNum < 10 )
			{
				reverseFlag = reverseFlagHis;
			}
#else
			if ( 1 == revContDiffNum )
			{
				reverseFlag = reverseFlagHis;
			}
#endif
		}
		else
		{
			segIdHis = segId;
			reverseFlagHis = reverseFlag;
			revContDiffNum = 0;
		}
#endif

		return true;
	}
}

#else
int GetAngle(IN point3D_t samplePoint,
	         IN point3D_t sectionPoint,
	         IN point3D_t leftPoint,
	         IN point3D_t rightPoint,
	         OUT float &cos_left,
	         OUT float &cos_right)
{
	double length_section_sample = getLength(sectionPoint.lon, sectionPoint.lat, samplePoint.lon, samplePoint.lat);
	double length_section_left = getLength(sectionPoint.lon, sectionPoint.lat, leftPoint.lon, leftPoint.lat);      
	double length_sample_left = getLength(samplePoint.lon, samplePoint.lat,leftPoint.lon, leftPoint.lat);     
	double length_section_right =  getLength(sectionPoint.lon, sectionPoint.lat,rightPoint.lon, rightPoint.lat); 
	double length_sample_right =  getLength(samplePoint.lon, samplePoint.lat,rightPoint.lon, rightPoint.lat); 
	cos_left =( length_section_sample*length_section_sample+ length_section_left*length_section_left - length_sample_left*length_sample_left) / (length_section_sample*length_section_left*2); 
	cos_right = ( length_section_sample*length_section_sample+ length_section_right*length_section_right - length_sample_right*length_sample_right) / (length_section_sample*length_section_right*2);
	return true;
}

bool getSectionId(IN point3D_t inPoint,
	              IN list<segAttributes_t> segConfigList,
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
		bodySectionPoint.push_back(itrSecConfig->ports[0]);
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
		bodySectionPoint.push_back(lastSection.ports[1]);
	}

	//step2
	foundSectionID = 0;
	vector<double> distance(bodySectionPoint.size());
	for (unsigned int i=0;i<=distance.size()-1;i++)
	{
		distance[i] = getLength(inPoint.lon,inPoint.lat,bodySectionPoint[i].lon,bodySectionPoint[i].lat);
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
	GetAngle(inPoint,bodySectionPoint[min_pos],bodySectionPoint[prevSectionID-1],
		     bodySectionPoint[nextSectionID-1],cos_left,cos_right);

	if (cos_left<0 && cos_right>0)
	{
		foundSectionID = minDisSectionID;
	} 
	else
	{
		foundSectionID = prevSectionID;
	}

	//step3
	if ( closedLoopFlag==0 && foundSectionID==bodySectionPoint.size() )
	{
		foundSectionID = 0;
	}

	return true;
}

bool getSegIdAndDirection(IN point3D_t inPoint1,
	                      IN point3D_t inPoint2,
	                      IN list<segAttributes_t> segConfigList,
	                      OUT uint32 &segId,
	                      OUT bool &reverseFlag)
{
	getSectionId( inPoint1,segConfigList,segId );

	if ( segId==0 )
	{
		return false;
	} 
	else
	{
		point3D_t bodyLeftPt,bodyRightPt,rotSrcLeft,rotSrcRight,rotPt1,rotPt2;
		list<segAttributes_t>::iterator itrSegCfg = segConfigList.begin();

		//get section body left point and body right point
		while ( itrSegCfg != segConfigList.end() )
		{
			if ( itrSegCfg->segId == segId )
			{
				bodyLeftPt = itrSegCfg->ports[0];
				bodyRightPt = itrSegCfg->ports[1];
				break;
			}
			else
			{
				itrSegCfg++;
				continue;
			}
		}

		//get section angle and compare
		double theta;
		theta = -atan((bodyRightPt.lat - bodyLeftPt.lat)/(bodyRightPt.lon - bodyLeftPt.lon + 0.0000000001));
		rotPoint(bodyLeftPt,theta,rotSrcLeft);
		rotPoint(bodyRightPt,theta,rotSrcRight);
		rotPoint(inPoint1,theta,rotPt1);
		rotPoint(inPoint2,theta,rotPt2);

		//is the point2->point1 the same direction with section body left->right 
		//the same direction: reverseFlag = 0; the reverse direction: reverseFlag = 1
		reverseFlag = ( rotPt2.lon - rotPt1.lon ) * ( rotSrcLeft.lon - rotSrcRight.lon ) > 0 ? FALSE : TRUE;

		return true;
	}
}
#endif

//direction: true(1) means reverser direction, false(0) means forward direction
bool fixVehicleLocationInLane(point3D_t reletiveGps,list<vector<point3D_t>> &allLines, bool direction, point3D_t *locationInLane)
{
	if(allLines.size() >= 2) //decide if there is a lane
	{
		list<vector<point3D_t>>::iterator lineIter = allLines.begin();
		vector<point3D_t> *firstLine;
		vector<point3D_t> *endLine;
		bool startFlag = false;
		int runDir = direction?1:0;
		int lineIdx = 0;
		int lineNum = allLines.size();
		int lineDirLast = 0;
		while(lineIter != allLines.end())
		{
			if(!lineIter->empty())
			{
				int count = (*lineIter)[0].count;
				int lineDir = (LANE_END_LINE_FLAG == count)?2:(count >> 24)&0x1;  // 0:forward, 1:reverser, 2. 0xdeadbeef,end line 

				if(startFlag) // if has find the start line
				{
					if((lineDir == 2)||(lineIdx == (lineNum - 1)))  //end line or the last line
					{
						endLine = &(*lineIter);
						break;
					}else if(lineDirLast != lineDir)
					{
						endLine = &(*lineIter);
						break;
					}
				}else
				{
					if(lineDir == runDir)
					{
						firstLine = &(*lineIter);
						startFlag = true;
					}
				}
				lineDirLast = lineDir;

			}
			lineIter++;
			lineIdx++;
		}

		if(!startFlag)
		{
			return false;
		}

		//only use the first and the last line to decide the road center
		//list<vector<point3D_t>>::iterator firstLine = allLines.begin();
		//list<vector<point3D_t>>::iterator endLine = allLines.end();
		//endLine--;

		//get the nearest point between the vehicle and lines
        float minDist1 = 9e+10;
		float minDist2 = 9e+10;
        point3D_t   firstMinIdx;
		point3D_t   endMinIdx;
        for(int pointIdx = 0; pointIdx < firstLine->size();++pointIdx)
        {
            point3D_t currentPoint = (*firstLine)[pointIdx];
            float distX = reletiveGps.lat - currentPoint.lat;
            float distY = reletiveGps.lon - currentPoint.lon;
            float dist  = (distX*distX + distY*distY);
            if(dist < minDist1)
            {
                minDist1 = dist;
                firstMinIdx  = currentPoint;
            }
        }

		for(int pointIdx = 0; pointIdx < endLine->size();++pointIdx)
        {
            point3D_t currentPoint = (*endLine)[pointIdx];
            float distX = reletiveGps.lat - currentPoint.lat;
            float distY = reletiveGps.lon - currentPoint.lon;
            float dist  = (distX*distX + distY*distY);
            if(dist < minDist2)
            {
                minDist2 = dist;
                endMinIdx  = currentPoint;
            }
        }

		float minDist = (minDist1>minDist2)?minDist2:minDist1;
		if(minDist < MAX_DIST_BETWEEN_CAR_AND_LANE*MAX_DIST_BETWEEN_CAR_AND_LANE)
		{
			locationInLane->lat = (firstMinIdx.lat + endMinIdx.lat)*0.5;
			locationInLane->lon = (firstMinIdx.lon + endMinIdx.lon)*0.5;
			locationInLane->alt = (firstMinIdx.alt + endMinIdx.alt)*0.5;
			return true;
		}

	}
	return false;
}

#if 0
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
#endif

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
