/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_VisualizePreProc.cpp
* @brief call Digital Horizon Data Access Manager to get the look ahead data.
*             in this demo, to match to GPS module, use GPS thread semphore to trigger one time access data operation.
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/
#include <stdio.h>
#include <process.h>
#include <math.h>

#include "database.h"
#include "databaseDef.h"
#include "databaseInVehicle.h" // databaseInVehicle

#include "AppInitCommon.h"
#include "Signal_Thread_Sync.h"
#include "NEMA_GPGGA_Proc.h"
#include "Visualization.h"
#include "VisualizeControl.h"
#include "saveLinePointInSafe.h"
#include "configure.h"
#include "getSectionID.h"
#include "TimeStamp.h"

using namespace ns_database;
using namespace std;
using namespace ns_historyLine;


//point3DFloat_t pointVecBuf[MAX_BUFFER_DEPTH_DRAW_LINE_POINT];
vector<point3DFloat_t> pointVecBuf;

void generateRect(float length, float width, quadInfo_t & rect)
{

	rect.vertex[0].x = -width/2;
	rect.vertex[0].y = 0;
	rect.vertex[0].z = length/2;

	rect.vertex[1].x = width/2;
	rect.vertex[1].y = 0;
	rect.vertex[1].z = length/2;

	rect.vertex[2].x = width/2;
	rect.vertex[2].y = 0;
	rect.vertex[2].z = -length/2;

	rect.vertex[3].x = -width/2;
	rect.vertex[3].y = 0;
	rect.vertex[3].z = -length/2;
}

bool assignSignOnRoadSharp(int type, signInfo_t &signInfo)
{
	bool returnValue = true;
	switch(type)
	{
	case 2001:
		generateRect(6, 1, signInfo.sharp);
		break;
	case 2002:
	case 2003:
	case 2004:
	case 2005:
		generateRect(6, 2, signInfo.sharp);
		break;
	case 1000:
		generateRect(0.4, 4, signInfo.sharp);
		break;
	case 1002:
		generateRect(2, 2, signInfo.sharp);
		break;
	default:
		returnValue = false;
		break;
	}
	return returnValue;
}

int convertSignType(int type)
{
	int number = 0;

#if((RD_LOCATION&RD_NATION_MASK) == RD_GERMAN)
	switch(type)
	{
	case 10100:
		number = 1;
		break;
	case 12300:
		number = 2;
		break;
	case 13100:
		number = 3;
		break;
	case 13310:
		number = 4;
		break;
	case 13810:
		number = 5;
		break;
	case 20500:
		number = 6;
		break;
	case 20600:
		number = 7;
		break;
	case 20930:
		number = 8;
		break;
	case 21500:
		number = 9;
		break;
	case 22220:
		number = 10;
		break;
	case 22400:
		number = 11;
		break;
	case 23700:
		number = 12;
		break;
	case 23900:
		number = 13;
		break;
	case 24000:
		number = 14;
		break;
	case 25000:
		number = 15;
		break;
	case 25900:
		number = 16;
		break;
	case 26100:
		number = 17;
		break;
	case 26210:
		number = 18;
		break;
	case 27452:
		number = 19;
		break;
	case 27453:
		number = 20;
		break;
	case 27454:
		number = 21;
		break;
	case 27455:
		number = 22;
		break;
	case 27456:
		number = 23;
		break;
	case 27458:
		number = 24;
		break;
	case 28300:
		number = 25;
		break;
	case 28600:
		number = 26;
		break;
	case 30100:
		number = 27;
		break;
	case 30600:
		number = 28;
		break;
	case 31400:
		number = 29;
		break;
	case 33100:
		number = 30;
		break;
	case 33600:
		number = 31;
		break;
	case 35010:
		number = 32;
		break;
	case 99900:
		number = 33;
		break;
	case 20910:
		number = 34;
		break;
	case 25400:
		number = 35;
		break;
	case 26700:
		number = 36;
		break;
	case 31401:
		number = 37;
		break;
    case 27600:
        number = 38;
        break;
    case 44100:
        number = 39;
        break;
    case 44200:
        number = 40;
        break;
	//for traffic sign on the road, need to assign the quad to texture
	case 2001:
		number = 42;
		break;
	case 2002:
		number = 43;
		break;
	case 2003:
		number = 44;
		break;
	case 2004:
		number = 45;
		break;
	case 2005:
		number = 46;
		break;
	case 1000:
		number = 47;
		break;
	case 1050:
		number = 48;
		break;
	case 10200:
		number = 49;
		break;
    case 24100:
		number = 50;
		break;
    case 28400:
		number = 51;
		break;
    case 28500:
		number = 52;
		break;
    case 28700:
		number = 53;
		break;
    case 34100:
		number = 54;
		break;
    case 10310:
        number = 55;
        break;
    case 10320:
        number = 56;
        break;
    case 22210:
        number = 57;
        break;
	default:
		number = 41;
		break;
	}
#else if((RD_LOCATION&RD_NATION_MASK) == RD_UNIT_STATES)
	switch(type)
	{
	case 1:
        // fall through
	case 2:
        // fall through
	case 3:
        // fall through
    case 4:
        // fall through
    case 5:
        // fall through
    case 6:
        // fall through
    case 7:
        // fall through
    case 8:
        // fall through
		number = type;
		break;
    case 35:
        number = 9;
        break;
	case 1000:
		number = 10;
		break;
    default: 
        number = 4;
    }

#endif
	return number;
}

void computeServerEyePosition(int number, point3DFloat_t* inPoint,eyeLookAt_t* eyeLookAt)
{
	static float max_x = -100000.0f;
	static float min_x = 1000000.0f;
	static float max_z = -100000.0f;
	static float min_z = 100000.0f;

	for(int idx = 0; idx < number; idx++)
	{
		max_x = (max_x > inPoint[idx].x)?max_x:inPoint[idx].x;
		min_x = (min_x > inPoint[idx].x)?inPoint[idx].x:min_x;
		max_z = (max_z > inPoint[idx].z)?max_z:inPoint[idx].z;
		min_z = (min_z > inPoint[idx].z)?inPoint[idx].z:min_z;
	}

	eyeLookAt->eyePosition.x = (max_x+min_x)/2;
	eyeLookAt->eyePosition.y = 180;
	eyeLookAt->eyePosition.z = (max_z+min_z)/2;

	eyeLookAt->lookatPosition.x = (max_x+min_x)/2;
	eyeLookAt->lookatPosition.y = 0;
	eyeLookAt->lookatPosition.z = (max_z+min_z)/2;

}

void coordinateChange2(point3D_t* standPoint, point3D_t* changePoint, point3D_t* outPoint)
{
	GLfloat dif_x = changePoint->lat - standPoint->lat;
	GLfloat dif_y = changePoint->lon - standPoint->lon;
	GLfloat dif_z = changePoint->alt - standPoint->alt;
	float latitude = (standPoint->lat)*PI/180;

    outPoint->lat = dif_x*COEFF_DD2METER;  //latitude
    outPoint->lon = dif_y*(111413*cos(latitude)-94*cos(3*latitude));  //longitude
    outPoint->alt = 0;//dif_z;
}

void changeDataBaseCoord(point3D_t* changePoint, point3DFloat_t* outPoint)
{
    outPoint->x = changePoint->lat;
    outPoint->y = changePoint->alt;
    outPoint->z = changePoint->lon;
}

//sample function assume the hight is the same, so it only need to compute the one surface direction
void computeEyeInfo(point3DFloat_t *startPoint, point3DFloat_t *endPoint, point3DFloat_t *oldPoint, eyeLookAt_t* buffer, int number)
{
	if((startPoint->x == oldPoint->x)&&(startPoint->z == oldPoint->z))
	{
		int index;
		//compute x,z coordinate's angle
		GLfloat angle1 = atan2((endPoint->z - startPoint->z),(endPoint->x - startPoint->x));
		//GLfloat angle1 = atan((endPoint->z - startPoint->z)/(endPoint->x - startPoint->x));

		GLfloat z_step = (endPoint->z - startPoint->z)/number;
		GLfloat x_step = (endPoint->x - startPoint->x)/number;

		for(index = 0; index < number; index++)
		{
			buffer[index].eyePosition.x = startPoint->x + x_step*index;
			buffer[index].eyePosition.y = startPoint->y + 4.5;
			buffer[index].eyePosition.z = startPoint->z + z_step*index;

			buffer[index].lookatPosition.x = buffer[index].eyePosition.x + cos(angle1);
			buffer[index].lookatPosition.y = startPoint->y + 4;
			buffer[index].lookatPosition.z = buffer[index].eyePosition.z + sin(angle1);
		}
	}
	else{
		int index;
		//compute x,z coordinate's angle
		GLfloat angle1 = atan2((endPoint->z - startPoint->z),(endPoint->x - startPoint->x));
		//GLfloat angle1 = atan((endPoint->z - startPoint->z)/(endPoint->x - startPoint->x));

		GLfloat z_step = (endPoint->z - startPoint->z)/number;
		GLfloat x_step = (endPoint->x - startPoint->x)/number;

		GLfloat angle2 = atan2((startPoint->z - oldPoint->z),(startPoint->x - oldPoint->x));
		//GLfloat angle2 = atan2((startPoint->z - oldPoint->z),(startPoint->x - oldPoint->x));
		GLfloat total_de_angle = (angle1-angle2);

		if(total_de_angle > PI)
		{
			total_de_angle -= 2*PI;
		}
		if(total_de_angle < -PI)
		{
			total_de_angle += 2*PI;
		}
		GLfloat de_angle = total_de_angle/(number);

		GLfloat angle;

		for(index = 0; index < number; index++)
		{
			angle = angle2 + de_angle*index;
			buffer[index].eyePosition.x = startPoint->x + x_step*index;
			buffer[index].eyePosition.y = startPoint->y + 4.5;
			buffer[index].eyePosition.z = startPoint->z + z_step*index;

			buffer[index].lookatPosition.x = buffer[index].eyePosition.x + cos(angle);
			buffer[index].lookatPosition.y = startPoint->y + 4;
			buffer[index].lookatPosition.z = buffer[index].eyePosition.z + sin(angle);
		}
	}

}

float computeDistance(point3DFloat_t *startPoint, point3DFloat_t *endPoint)
{
	GLfloat z_dif = (endPoint->z - startPoint->z);
	GLfloat x_dif = (endPoint->x - startPoint->x);

	return sqrt(z_dif*z_dif + x_dif*x_dif);
	//speed = speed*3.6; //km per hour
}

void computeCharPosition(point3DFloat_t *startPoint, point3DFloat_t *endPoint, point3DFloat_t* outPoint)
{
	//compute x,z coordinate's angle
	GLfloat angle = atan2((endPoint->z - startPoint->z),(endPoint->x - startPoint->x));

	outPoint->x = startPoint->x + 30*cos(angle);
	outPoint->y = startPoint->y + 4;
	outPoint->z = startPoint->z + 30*sin(angle);
}

void computeCurrentSpeedChar(point3DFloat_t *startPoint, point3DFloat_t *endPoint, drawCharInfo_t *speedChar, float duration)
{
	float distance = computeDistance(startPoint, endPoint);
	computeCharPosition(startPoint, endPoint, &(speedChar->position));

	//memcpy(&(speedChar->drawChar),(void*)"speed: ",7);
	sprintf(&(speedChar->drawChar[0]),"%2.0f",distance*3.6/duration);
	memcpy(&(speedChar->drawChar[2]),(void*)"kph",3);
	memset(&(speedChar->drawChar[5]),0,1);   
}

void convFurToSignInfo(list<list<furAttributesInVehicle_t>>& furnitureList, 
	vector<signInfo_t> &signInfo, int loopIdx)
{
	list<list<furAttributesInVehicle_t>>::iterator segIter = furnitureList.begin();

	while(segIter != furnitureList.end())
	{
		list<furAttributesInVehicle_t>::iterator furIter = (*segIter).begin();

		// For each furniture
		while(furIter != (*segIter).end())
		{
			point3DFloat_t   location;
			furAttributesInVehicle_t* furInfo = &(*furIter);

			// Check validation
			if((furInfo->type_used == 0) || 
				(furInfo->location_used == 0) ||
				(furInfo->angle_used == 0) ||
				(furInfo->reliabRating_used == 0))
			{
				furIter++;
				continue;
			}

			//coordinateChange(&standPoint, &(furInfo->location), &location);
            changeDataBaseCoord(&(furInfo->location), &location);

			int furType = convertSignType(furInfo->type);
			if(furType != 0)
			{
				signInfo_t signTemp;
				signTemp.attribute = furInfo->reliabRating;
				signTemp.position.x = location.x;
				signTemp.position.y = location.y;
				signTemp.position.z = location.z;
				signTemp.rotAngle   = furInfo->angle * 180 / PI;
				signTemp.type       = convertSignType(furInfo->type);
				signTemp.sideFlag   = furInfo->sideFlag;
				signTemp.showFlag   = (loopIdx == furInfo->inLoopIdx)?true:false;
				if(furInfo->sideFlag == 4)
				{
					assignSignOnRoadSharp(furInfo->type, signTemp);
				}
				signInfo.push_back(signTemp);
			}
			furIter++;
		}
		segIter++;
	}
}

//void gpsTimer(int value)
void WINAPI gpsTimer(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	if(10 == dwUser)
	{
		RD_ADD_TS(tsFunId_eVisualPreTimerIsr,1);
		ReleaseSemaphore(g_readySema_GPS, 1 ,NULL);
		//glutTimerFunc((unsigned int)(1000),&gpsTimer,2);
	}
}

void generateRoadSideVec(vector<point3DFloat_t> &midVec, float width, vector<point3DFloat_t> &leftVec, vector<point3DFloat_t> &rightVec)
{
	leftVec.clear();
	rightVec.clear();
	point3DFloat_t point1;
	point3DFloat_t point2;
	int lineIndex = 0;
	float cosAlpha = 0.0;
	float sinAlpha = 0.0;

	leftVec.assign(midVec.size(), point1);
	rightVec.assign(midVec.size(), point2);

    int numOfMidPnts = midVec.size();
	for(lineIndex = 0; lineIndex < (numOfMidPnts-1); lineIndex++)
	{
		point1 = midVec[lineIndex];
		point2 = midVec[lineIndex+1];

		float lonDis = point1.z - point2.z;
		float latDis = point1.x - point2.x;

		float rou = sqrt(lonDis*lonDis + latDis*latDis);
		cosAlpha = (0.0 == rou) ? 0.0 : (latDis/rou);
		sinAlpha = (0.0 == rou) ? 0.0 : (lonDis/rou);

		leftVec[lineIndex].x = point1.x - width*sinAlpha;
		leftVec[lineIndex].z = point1.z + width*cosAlpha;

		rightVec[lineIndex].x = point1.x + width*sinAlpha;
		rightVec[lineIndex].z = point1.z - width*cosAlpha;

		leftVec[lineIndex].y = 0;
		rightVec[lineIndex].y = 0;
	}

	point1 = midVec[lineIndex];
	leftVec[lineIndex].x = point1.x - width*sinAlpha;
	leftVec[lineIndex].z = point1.z + width*cosAlpha;

	rightVec[lineIndex].x = point1.x + width*sinAlpha;
	rightVec[lineIndex].z = point1.z - width*cosAlpha;

	leftVec[lineIndex].y = 0;
	rightVec[lineIndex].y = 0;
}

void extractQuadInfo(point3DFloat_t startPoint, point3DFloat_t endPoint, float width, bool contiFlag, quadInfo_t *quadInfo, float height)
{
	static float startX_left = 0.0;
	static float startY_left = 0.0;
	static float startZ_left = 0.0;

	static float startX_right = 0.0;
	static float startY_right = 0.0;
	static float startZ_right = 0.0;

	float dif_z = endPoint.z - startPoint.z;
	float dif_x = endPoint.x - startPoint.x;
	float shift_z, shift_x, tan_a;
	if(0 == dif_x)
	{
		shift_z = 0;
		shift_x = width;
	}
	else
	{
		tan_a = dif_z/dif_x;
		shift_z = sqrt(width*width/(1+tan_a*tan_a));
		shift_x = tan_a*shift_z;	
	}

	if(contiFlag)
	{
		quadInfo->vertex[0].x = startX_left;
		quadInfo->vertex[0].y = startY_left;
		quadInfo->vertex[0].z = startZ_left;

		quadInfo->vertex[1].x = startX_right;
		quadInfo->vertex[1].y = startY_right;
		quadInfo->vertex[1].z = startZ_right;
	}else
	{
		quadInfo->vertex[0].x = startPoint.x - shift_x;
		quadInfo->vertex[0].y = startPoint.y + height;//startPoint.y;
		quadInfo->vertex[0].z = startPoint.z + shift_z;

		quadInfo->vertex[1].x = startPoint.x + shift_x;
		quadInfo->vertex[1].y = startPoint.y + height;//startPoint.y;
		quadInfo->vertex[1].z = startPoint.z - shift_z;
	}

	quadInfo->vertex[2].x = endPoint.x + shift_x;
	quadInfo->vertex[2].y = endPoint.y + height;//endPoint.y;
	quadInfo->vertex[2].z = endPoint.z - shift_z;

	quadInfo->vertex[3].x = endPoint.x - shift_x;
	quadInfo->vertex[3].y = endPoint.y + height;//endPoint.y;
	quadInfo->vertex[3].z = endPoint.z + shift_z;

	startX_left = quadInfo->vertex[3].x;
	startY_left = quadInfo->vertex[3].y;
	startZ_left = quadInfo->vertex[3].z;

	startX_right = quadInfo->vertex[2].x;
	startY_right = quadInfo->vertex[2].y;
	startZ_right = quadInfo->vertex[2].z;
}

void generateRoadChar(point3DFloat_t position, int showNum, drawServerCharInfo_t &testChar)
{
	if(showNum > 99)
	{
		showNum = 99;
	}else if(showNum < 0)
	{
		showNum = 0;
	}
    
	testChar.position = position;
	testChar.position.y += 0.1;
	if(showNum < 10)
	{
		sprintf(&testChar.drawChar[0],"%d",showNum);
		memset(&testChar.drawChar[1],0,1);
	}else
	{
		sprintf(&testChar.drawChar[0],"%d",(showNum/10));
		sprintf(&testChar.drawChar[1],"%d",(showNum%10));
		memset(&testChar.drawChar[2],0,1);
	}
}

unsigned int __stdcall Thread_VisualizePreProc(void *data)
{
	point3D_t standPoint;

	eyeLookAt_t eyeInfo[200];
	eyeLookAt_t eyeInfoAhead[200];

	// Points for look ahead view
	point3D_t gpsAheadOld, gpsAheadNew;
	point3D_t gpsCurrent;
	point3DFloat_t lastPointF;
	point3D_t gpsAheadLast;
    point3D_t newPointD;
	point3D_t newPointD_Modify;
	point3D_t oldPointD;
	point3D_t oldPointD_Modify;
	bool driveViewUpdateFlag;

	list<list<vector<point3D_t>>> allLinesBak; // segment list / vector list / point list / point
	list<list<lineAttributes_t>> lineAttrBak; // segment list / vector list / attributes

	vector<uint32> SegIdInSky;
	roadSegConfig_gp->getRoadSegmentInSky(SegIdInSky);

	gpsAheadNew.lat = 0;
	gpsAheadNew.lon = 0;
	gpsAheadNew.alt = 0;

	gpsAheadOld = gpsAheadNew;
	gpsAheadLast = gpsAheadOld;

	lastPointF.x = 0;
	lastPointF.y = 0;
	lastPointF.z = 0;

	point3DFloat_t oldPointF;
	point3DFloat_t newPointF;
	oldPointF.x = 0;
	oldPointF.y = 0;
	oldPointF.z = 0;

	newPointF.x = 0;
	newPointF.y = 0;
	newPointF.z = 0;

    //newPointD.lat = 0;
	//newPointD.lon = 0;
	//newPointD.alt = 0;
	MMRESULT timer_id;
	timer_id = timeSetEvent((1000),1,(LPTIMECALLBACK)gpsTimer, DWORD(10),TIME_PERIODIC);
	//glutTimerFunc((unsigned int)(1000),&gpsTimer,2);

	standPoint.lat = inParamVec[0].GPSref.x;//42.296853333333331;//gGpsInfo.dLatitude;
	standPoint.lon = inParamVec[0].GPSref.y;//-83.213250000000002;//gGpsInfo.dLongitude;
	standPoint.alt = 0.0;//gGpsInfo.altitude;

	engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
	engine3DPtr->SwapServerEyeBuffer();
	RD_ADD_TS(tsFunId_eThread_Visual_Pre,2);

	point3D_t orgPoint;
	orgPoint.lat = gGpsInfo.dLatitudePre;
	orgPoint.alt = gGpsInfo.altitudePre;
	orgPoint.lon = gGpsInfo.dLongitudePre;
	coordinateChange2(&standPoint, &orgPoint, &oldPointD);

	while(1)
	{
		//wait for GPS thread to get GPS signal
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,3);
		WaitForSingleObject(g_readySema_GPS, INFINITE); 
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,4);

		int currentloopIdx = getLoopIdxFromFurInLoopIdx(gGpsInfo.inParamIdxs);
		driveViewUpdateFlag = true;

        orgPoint.lat = gGpsInfo.dLatitude;
		orgPoint.alt = gGpsInfo.altitude;
		orgPoint.lon = gGpsInfo.dLongitude;
        coordinateChange2(&standPoint, &orgPoint, &newPointD);
		//orgPoint.lat = gGpsInfo.dLatitudePre;
		//orgPoint.alt = gGpsInfo.altitudePre;
		//orgPoint.alt = gGpsInfo.dLatitudePre;
		//coordinateChange2(&standPoint, &orgPoint, &oldPointD);
		newPointD_Modify = newPointD;
		

		//get all lines
		{
			list<list<vector<point3D_t>>> allLines; // segment list / vector list / point list / point
			list<list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
			//database_gp->getAllVectors(allLines, lineAttr);
            bool dataAccess = database_gp->getAllVectorsAsync(allLines, lineAttr);
			RD_ADD_TS(tsFunId_eThread_Visual_Pre,5);
			//cout<<"accessFlag = " <<dataAccess<<endl;
			//point3D_t standPoint = (*(*allLines.begin()).begin())[0];
            if(dataAccess == true)
            {
			    if(allLines.size() != 0)
			    {
					allLinesBak.clear();
					allLinesBak = allLines;
					lineAttrBak.clear();
					lineAttrBak = lineAttr;
				    list<list<vector<point3D_t>>>::iterator lineInSegIter = allLines.begin();
				    list<list<lineAttributes_t>>::iterator lineAttrInSegIter = lineAttr.begin();

				    // For each segment
				    while(lineInSegIter != allLines.end())
				    {
					    list<vector<point3D_t>>::iterator lineIter = (*lineInSegIter).begin();
					    list<lineAttributes_t>::iterator lineAttrIter = (*lineAttrInSegIter).begin();
						
						bool drawFlag = true;
						float segInSkyHeight = 0;
						vector<point3DFloat_t> startVec;
						vector<point3DFloat_t> endVec;

						if(lineAttrInSegIter->size() != 0)
						{
							int segmentId = lineAttrInSegIter->front().segmentId;

							for(int skyIndex = 0; skyIndex < SegIdInSky.size(); skyIndex++)
							{
								if(segmentId == SegIdInSky[skyIndex])
								{
									segInSkyHeight = 0.01;
									break;
								}
							}

							list<segAttributes_t>::iterator segIter = g_segCfgList.begin();
							while(segIter != g_segCfgList.end())
							{
								if(segIter->segId == segmentId)
								{
#if defined(_FRANKFORT_ALL_CAMERA)

									if(((0 == segIter->loopIdx)||(2 == segIter->loopIdx))&&(segIter->loopIdx_used))
									{
										drawFlag = true;
									}else
										
									{
										drawFlag = false;
									}
									break;
#else

									if((currentloopIdx == segIter->loopIdx)&&(segIter->loopIdx_used))
									{
										drawFlag = true;
									}else
										
									{
										drawFlag = false;
									}
									break;
#endif
								}
								++segIter;
							}
						}

					    int lineNum = (*lineInSegIter).size();
					    int lineIdx = 0;
						point3DFloat_t lastPos;
				        int drawNum;
						baseColor_t color;
    					color.R = 0.25;
    					color.G = 0.25;
    					color.B = 0.25;
						triangle_t triBuf;
						triBuf.color = color;
						triBuf.showFlag = drawFlag;

						RD_ADD_TS(tsFunId_eThread_Visual_Pre,6);

					    if(lineNum >= 2)
					    {
    					    // For each vector
    					    while(lineIter != (*lineInSegIter).end())
    					    {
    						    lineTypeEnum_t lineStyle = (lineTypeEnum_t)(*lineAttrIter).lineStyle;

    						    int numberPoint = (*lineAttrIter).numPoints;
    						    point3DFloat_t tempPoint;
								RD_ADD_TS(tsFunId_eThread_Visual_Pre,7);
								if(numberPoint >0)
								{
    								for(int pointIdx = 0; pointIdx < numberPoint; pointIdx++)
    								{
										changeDataBaseCoord(&(*lineIter)[pointIdx], &tempPoint);
										tempPoint.y += segInSkyHeight;
    									pointVecBuf.push_back(tempPoint);
    								}

    								baseColor_t colorLine;
    								colorLine.R = 0;//0.15;
    								colorLine.G = 1;//0.44;
    								colorLine.B = 0;//0.12;
    								quadInfo_t quadBuf;
    								quadBuf.color = colorLine;


    								//engine3DPtr->AddOneRoadLineInfo(lineStyle,color, pointVecBuf);
    								engine3DPtr->AddOneLineInfo(lineTypeEnum_solid, colorLine, drawFlag, pointVecBuf);

    								//for(int index = 0; index < (numberPoint-1); index++)
    								//{
    								//    extractQuadInfo((pointVecBuf[index]), pointVecBuf[index+1], 0.03, index, &quadBuf,0.005);
    								//    engine3DPtr->AddQuadInfo(1,&quadBuf);
    								// }

        							if(lineIdx == 0)
        							{
        								lastPos = pointVecBuf[0];
        								int count = (*lineIter)[0].count;
										drawNum = (count == LANE_END_LINE_FLAG)?count:CLEAN_LANE_DIR_BIT(count);
										engine3DPtr->AddOneRoadLineInfo(lineStyle,color, drawFlag, pointVecBuf);
										startVec.push_back(pointVecBuf[0]);
										endVec.push_back(pointVecBuf[pointVecBuf.size()-1]);
        							}else
        							{
        								point3DFloat_t drawPos;
        								drawPos.x = (lastPos.x + pointVecBuf[0].x)/2;
        								drawPos.y = (lastPos.y + pointVecBuf[0].y)/2;
        								drawPos.z = (lastPos.z + pointVecBuf[0].z)/2;

        								drawServerCharInfo_t testChar;
										if(LANE_END_LINE_FLAG != drawNum)
										{
        									generateRoadChar(drawPos, drawNum, testChar);
        									engine3DPtr->AddOneServerCharInfo(testChar);
										}else
										{
											engine3DPtr->AddOneRoadLineInfo(lineStyle,color, drawFlag, pointVecBuf);
											startVec.push_back(pointVecBuf[0]);
											endVec.push_back(pointVecBuf[pointVecBuf.size()-1]);
										}
							
        								lastPos = pointVecBuf[0];
        								int count = (*lineIter)[0].count;
										drawNum = (count == LANE_END_LINE_FLAG)?count:CLEAN_LANE_DIR_BIT(count);
										if((LANE_END_LINE_FLAG == drawNum) || (lineIdx == (lineNum-1)))
										{
											engine3DPtr->AddOneRoadLineInfo(lineStyle,color, drawFlag, pointVecBuf);
											int startSize = startVec.size();
											int endSize = endVec.size();
											if(startSize > 1)
											{
												for(int triIdx = 1; triIdx < startSize; triIdx++)
												{
													triBuf.vertex[0] = startVec[0];
													triBuf.vertex[1] = startVec[triIdx];
													triBuf.vertex[2] = pointVecBuf[0];
													engine3DPtr->AddTriInto(1, &triBuf);
												}
											}

											if(endSize > 1)
											{
												for(int triIdx = 1; triIdx < endSize; triIdx++)
												{
													triBuf.vertex[0] = endVec[0];
													triBuf.vertex[1] = endVec[triIdx];
													triBuf.vertex[2] = pointVecBuf[pointVecBuf.size()-1];
													engine3DPtr->AddTriInto(1, &triBuf);
												}
											}
											startVec.clear();
											endVec.clear();
										}else
										{
											startVec.push_back(pointVecBuf[0]);
											endVec.push_back(pointVecBuf[pointVecBuf.size()-1]);
										}
        							}

    								//draw the paint of the line
    								{
    									baseColor_t color;
    									color.R = 1.0;
    									color.G = 0.95;
    									color.B = 0.9;

    									quadInfo_t quadBuf;
    									quadBuf.color = color;
										quadBuf.showFlag = drawFlag;
    									bool contiFlag = false;

    									for(int index = 0; index < (numberPoint-1); index+=4)
    									{
    										//extractQuadInfo(pointVecBuf[2*index], pointVecBuf[2*index+1], 0.2, false, &quadBuf);
    										if(((*lineIter)[index].paintFlag>=0.5)&&((*lineIter)[index+1].paintFlag>=0.5))
    										{
    											extractQuadInfo((pointVecBuf[index]), pointVecBuf[index+1], 0.2, contiFlag, &quadBuf,0.01);
    											contiFlag = true;
    											engine3DPtr->AddQuadInfo(1,&quadBuf);
    										}else
    										{
    											contiFlag = false;
    										}
    									}
    								}

									pointVecBuf.clear();
								}//end for if(numberPoint > 0)

    						    lineIter++;
    						    lineAttrIter++;
    						    lineIdx++;
    					    } //end for line
					    }//end for if
                        
					    lineInSegIter++;
					    lineAttrInSegIter++;
				    }//end for while



                    //put the car to the lane.
                    {
                        uint32 secId = 0;
                        int lineId; 
						bool direction = false;
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
				
					getSegIdAndDirection(newPointD,oldPointD,currentloopIdx,secId,direction);
#else
                        getSectionId(newPointD, g_segCfgList,secId);
#endif
						if(secId!=0)
						{
							int laneNumSec = roadSegConfig_gp->getLaneNumInSeg(secId,direction);
							if(laneNumSec < 1)
							{
								driveViewUpdateFlag = false;
							}
							bool emptyFlag = checkLineSection(lineAttr, secId, lineId);
							if(emptyFlag)
							{
								list<list<vector<point3D_t>>>::iterator lineInSegIter2 = allLines.begin();
								for(int idx = 0; idx < lineId; idx++)
								{
									lineInSegIter2++;
								}
							  fixVehicleLocationInLane(newPointD,(*lineInSegIter2), direction, &newPointD_Modify);
							}
						}
                    }

				    database_gp->getAllVectors_clear(allLines, lineAttr);
                } // end if(allLines.size() != 0)

		        engine3DPtr->SwapLineBuffer();
		        engine3DPtr->SwapRoadLineBuffer();
		        engine3DPtr->SwapQuadBuffer();
				engine3DPtr->SwapServerCharBuffer();
				engine3DPtr->SwapTriBuffer();
			}// end if(dataAccess == true)
			else
			{
                uint32 secId = 0;
                int lineId;     
				bool direction = false;
				RD_ADD_TS(tsFunId_eThread_Visual_Pre,8);

#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
				getSegIdAndDirection(newPointD,oldPointD,currentloopIdx,secId,direction);
#else
				getSectionId(newPointD, g_segCfgList,secId);
#endif
				if(secId != 0)
				{
					int laneNumSec = roadSegConfig_gp->getLaneNumInSeg(secId,direction);
					if(laneNumSec < 1 )
					{
						driveViewUpdateFlag = false;
					}
					bool emptyFlag = checkLineSection(lineAttrBak, secId, lineId);
					if(emptyFlag)
					{
						list<list<vector<point3D_t>>>::iterator lineInSegIter2 = allLinesBak.begin();
						for(int idx = 0; idx < lineId; idx++)
						{
							lineInSegIter2++;
						}
						fixVehicleLocationInLane(newPointD,(*lineInSegIter2),direction, &newPointD_Modify);
					}
				}
			}
        }

		RD_ADD_TS(tsFunId_eThread_Visual_Pre,10);
		//For all the furnitures
		{
			vector<signInfo_t> signInfo;
			signInfo.clear();
			//int numSign;

			list<list<furAttributesInVehicle_t>> furnitureList;

			//database_gp->getAllFurnitures(furnitureList);
            bool furAccess = database_gp->getAllFurnituresAsync(furnitureList);

            if(furAccess)
            {
				#if defined(_FRANKFORT_ALL_CAMERA)
			    convFurToSignInfo(furnitureList, signInfo, 0);
				convFurToSignInfo(furnitureList, signInfo, 2);
				#else
				convFurToSignInfo(furnitureList, signInfo, currentloopIdx);
				#endif

			    furnitureList.clear();

			    engine3DPtr->AddSignInfo(signInfo);
			    engine3DPtr->SwapSignBuffer();
            }
		}

		RD_ADD_TS(tsFunId_eThread_Visual_Pre,11);
		//update the car position and look ahead direction
		{
			float x_dif = (newPointD_Modify.lat - newPointF.x);
			float z_dif = (newPointD_Modify.lon - newPointF.z);

			float distance = sqrt(z_dif*z_dif + x_dif*x_dif);
			
			if((driveViewUpdateFlag) && (distance > 1))
			{
				lastPointF = oldPointF;
				oldPointF = newPointF;

				newPointF.x = newPointD_Modify.lat;  //latitude
				newPointF.z = newPointD_Modify.lon;  //latitude
				newPointF.y = newPointD_Modify.alt;  //latitude
				oldPointD = newPointD;
				//use current GPS position and last GPS position to compute the direction.
				if((oldPointF.x == newPointF.x)&&(oldPointF.z == newPointF.z))
				{

				}else
				{
					//update the eye
					computeEyeInfo(&oldPointF, &newPointF,&lastPointF, eyeInfo, FRAME_NUM_PER_SECOND);
					engine3DPtr->setEyeLookat(FRAME_NUM_PER_SECOND,eyeInfo);

					engine3DPtr->SwapEyeBuffer();
				}
			}
			//update look ahead figure. TBD

		}
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,12);

	}//end for while(1)

	timeKillEvent(timer_id); 
	return 0;
}
