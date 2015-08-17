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

#include "Signal_Thread_Sync.h"
#include "NEMA_GPGGA_Proc.h"
#include "Visualization.h"
#include "VisualizeControl.h"

using namespace ns_database;
using namespace std;

extern ns_database::database* database_gp;

point3DFloat_t pointVecBuf[2000];

int convertSignType(int type)
{
	int number = 0;
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
	case 22220:
		number = 8;
		break;
	case 22400:
		number = 9;
		break;
	case 23700:
		number = 10;
		break;
	case 23900:
		number = 11;
		break;
	case 24000:
		number = 12;
		break;
	case 27452:
		number = 13;
		break;
	case 27453:
		number = 14;
		break;
	case 27454:
		number = 15;
		break;
	case 27455:
		number = 16;
		break;
	case 27456:
		number = 17;
		break;
	case 28300:
		number = 18;
		break;
	case 28600:
		number = 19;
		break;
	case 30100:
		number = 20;
		break;
	case 30600:
		number = 21;
		break;
	case 31400:
		number = 22;
		break;
	case 35010:
		number = 23;
		break;
	default:
		number = 0;
		break;
	}
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

void coordinateChange(point3D_t* standPoint, point3D_t* changePoint, point3DFloat_t* outPoint)
{
	GLfloat dif_x = changePoint->lat - standPoint->lat;
	GLfloat dif_y = changePoint->lon - standPoint->lon;
	GLfloat dif_z = changePoint->alt - standPoint->alt;

	outPoint->x = dif_x*COEFF_DD2METER;
	outPoint->z = dif_y*COEFF_DD2METER;
	outPoint->y = dif_z;
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
	float speed;

	speed = sqrt(z_dif*z_dif + x_dif*x_dif);
	speed = speed*3.6; //km per hour
	return speed;
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
	sprintf(&(speedChar->drawChar[0]),"%2.0f",distance/duration);
	memcpy(&(speedChar->drawChar[2]),(void*)"kph",3);
	memset(&(speedChar->drawChar[5]),0,1);   
}

void convFurToSignInfo(list<list<furAttributes_t>>& furnitureList, 
                       point3D_t& standPoint, 
                       signInfo_t* signInfo, 
                       int* numSign)
{
    int numSignLoc = 0;

    list<list<furAttributes_t>>::iterator segIter = furnitureList.begin();

    while(segIter != furnitureList.end())
    {
        list<furAttributes_t>::iterator furIter = (*segIter).begin();

        // For each furniture
        while(furIter != (*segIter).end())
        {
            point3DFloat_t   location;
            furAttributes_t* furInfo = &(*furIter);

            // Check validation
            if((furInfo->type_used == 0) || 
               (furInfo->location_used == 0) ||
               (furInfo->angle_used == 0) ||
               (furInfo->reliabRating_used == 0))
            {
                furIter++;
                continue;
            }

            coordinateChange(&standPoint, &(furInfo->location), &location);
            
            int furType = convertSignType(furInfo->type);
            if(furType != 0)
            {
                signInfo[numSignLoc].type = convertSignType(furInfo->type);
                signInfo[numSignLoc].rotAngle = furInfo->angle * 180 / 3.14159;
                signInfo[numSignLoc].position.x = location.x;
                signInfo[numSignLoc].position.y = location.y;
                signInfo[numSignLoc].position.z = location.z;
                signInfo[numSignLoc].attribute = furInfo->reliabRating;
                numSignLoc++;
            }
            furIter++;
        }

        segIter++;
    }

    *numSign = numSignLoc;
}

unsigned int __stdcall Thread_VisualizePreProc(void *data)
{
    //load basic road data into 3D engine
	int index;

	eyeLookAt_t eyeInfo[200];
    eyeLookAt_t eyeInfoAhead[200];
    //numberMid = load_vector_data("road_data_middle.txt",midPointVec);
	//numberRight = load_vector_data("road_data_right.txt",rightPointVec);
	//numberLeft  = load_vector_data("road_data_left.txt",leftPointVec);

    list<list<vector<point3D_t>>> allLines; // segment list / vector list / point list / point
    list<list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
    database_gp->getAllVectors(allLines, lineAttr);

    point3D_t standPoint = (*(*allLines.begin()).begin())[0];

    list<list<vector<point3D_t>>>::iterator lineInSegIter = allLines.begin();
    list<list<lineAttributes_t>>::iterator lineAttrInSegIter = lineAttr.begin();

    // For each segment
    while(lineInSegIter != allLines.end())
    {
        list<vector<point3D_t>>::iterator lineIter = (*lineInSegIter).begin();
        list<lineAttributes_t>::iterator lineAttrIter = (*lineAttrInSegIter).begin();

        // For each vector
        while(lineIter != (*lineInSegIter).end())
        {
            lineTypeEnum_t lineStyle = (lineTypeEnum_t)(*lineAttrIter).lineStyle;

            int numberPoint = (*lineAttrIter).numPoints;

            for(int pointIdx = 0; pointIdx < numberPoint; pointIdx++)
            {
                coordinateChange(&standPoint,&(*lineIter)[pointIdx], &(pointVecBuf[pointIdx]));
            }

			if((lineTypeEnum_road_line == lineStyle)||(lineTypeEnum_roadside_line == lineStyle))
			{
				engine3DPtr->AddOneRoadLineInfo(numberPoint,lineStyle,pointVecBuf);
			}else
			{
				engine3DPtr->AddOneLineInfo(numberPoint, lineStyle, pointVecBuf);
			}

			computeServerEyePosition(numberPoint, pointVecBuf,serverEyeInfo);

            lineIter++;
            lineAttrIter++;
        }

        lineInSegIter++;
        lineAttrInSegIter++;
    }

	engine3DPtr->SwapLineBuffer();
	engine3DPtr->SwapRoadLineBuffer();

	//set sky perspective eye position 
	engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
	engine3DPtr->SwapServerEyeBuffer();


    database_gp->getAllVectors_clear(allLines, lineAttr);

#if 0
	computeEyeInfo(&midPointVec[0], &midPointVec[1], eyeInfo, 10);
	engine3DPtr->setEyeLookat(10,eyeInfo);
	engine3DPtr->SwapEyeBuffer();
#endif

	// Points for look ahead view
	point3D_t gpsAheadOld, gpsAheadNew;
	point3DFloat_t lastPointF;
	point3D_t gpsAheadLast;

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

    while(1)
    {
        //wait for GPS thread to get GPS signal
        WaitForSingleObject(g_readySema_GPS, INFINITE);    
        
        point3D_t oldPointD;
	    point3D_t newPointD; 



		drawCharInfo_t speedChar;
		int numFrame = (gGpsInfo.st - gGpsInfo.stPre + (1000/FRAME_NUM_PER_SECOND) - 1)/(1000/FRAME_NUM_PER_SECOND);
		numFrame = (numFrame > 200)?200:numFrame;

		//printf("numFrame = %d\n",numFrame);
		
        //get current GPS position and translate the coordinate
		oldPointD.lat = gGpsInfo.dLatitudePre;
		oldPointD.alt = gGpsInfo.altitudePre;
		oldPointD.lon = gGpsInfo.dLongitudePre;

		newPointD.lat = gGpsInfo.dLatitude;
		newPointD.alt = gGpsInfo.altitude;
		newPointD.lon = gGpsInfo.dLongitude;

		lastPointF = oldPointF;
		coordinateChange(&standPoint, &oldPointD, &oldPointF);
		coordinateChange(&standPoint, &newPointD, &newPointF);
        
        // ******************* TEST ONLY *******************
        signInfo_t signInfo[200];
        int numSign;

        list<list<furAttributes_t>> furnitureList;
        
        database_gp->getAllFurnitures(furnitureList);

        convFurToSignInfo(furnitureList, standPoint, signInfo, &numSign);

        furnitureList.clear();

        engine3DPtr->AddSignInfo(numSign,signInfo);
        engine3DPtr->SwapSignBuffer();
        // ******************* TEST ONLY END *******************

		//use current GPS position and last GPS position to compute the direction.
        if((oldPointF.x == newPointF.x)&&(oldPointF.z == newPointF.z))
		{

		}else if(0 != numFrame)
		{
			//update the eye
			computeEyeInfo(&oldPointF, &newPointF,&lastPointF, eyeInfo, numFrame);
			engine3DPtr->setEyeLookat(numFrame,eyeInfo);

			//update the speed character
			float timeDif = 1.0f;
			if(gGpsInfo.st > gGpsInfo.stPre)
			{
#if 0
				if((gGpsInfo.st - gGpsInfo.stPre) > 500) //1HZ
				{
					timeDif = 1.0f;
				}else          //10HZ
				{
					timeDif = 0.1f;
				}
#endif
				timeDif = ((float)(gGpsInfo.st - gGpsInfo.stPre))/1000.0f;
			}

			computeCurrentSpeedChar(&oldPointF, &newPointF, &speedChar, timeDif);
			engine3DPtr->setDrawChar(&speedChar);

			engine3DPtr->SwapEyeBuffer();
			engine3DPtr->SwapCharBuffer();
		}

		// Look ahead
		gpsAheadLast = gpsAheadOld;
		gpsAheadOld = gpsAheadNew;
		
        database_gp->getLookAheadView(&newPointD, 100, &gpsAheadNew);

        point3DFloat_t oldPointFAhead, newPointFAhead, lastPointFAhead;
        coordinateChange(&standPoint, &gpsAheadOld, &oldPointFAhead);
        coordinateChange(&standPoint, &gpsAheadNew, &newPointFAhead);
		coordinateChange(&standPoint, &gpsAheadLast, &lastPointFAhead);
		if((oldPointFAhead.x == newPointFAhead.x)&&(oldPointFAhead.z == newPointFAhead.z))
		{
		}else if(0 != numFrame)
		{
			computeEyeInfo(&oldPointFAhead, &newPointFAhead, &lastPointFAhead,eyeInfoAhead, numFrame);
			engine3DPtr->setLookAheadEyeLookat(numFrame, eyeInfoAhead);
			engine3DPtr->SwapLookaheadEyeBuffer();
		}
        //call Digital Horizon Data Access Manager to get the look ahead data.
        
        //notify Thread Difference Detect and Report to do image process
        
        //notify 3D drawing thread the look ahead data
        
    }
    return 0;
}