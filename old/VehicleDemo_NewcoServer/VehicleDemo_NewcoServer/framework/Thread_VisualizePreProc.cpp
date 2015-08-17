/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_VisualizePreProc.cpp
* @brief call Digital Horizon Data Access Manager to get the look ahead data.
*             in this demo, when the Horizon data base changed, it will trigger this thread to redraw.
*
* Change Log:
*      Date                Who             What
*      2014/01/25         Xin Shao        Create
*******************************************************************************
*/
#include "Thread_VisualizePreProc.h"
#include "Visualization.h"

#include "database.h"
#include "databaseDef.h"
#include "appInitCommon.h"
#include "VisualizeControl.h"

using namespace ns_database;
using namespace std;
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

	//get all the lines
	list<list<vector<point3D_t>>> allLines; // segment list / vector list / point list / point
	list<list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
	database_gp->getAllVectors(allLines, lineAttr);
	point3D_t standPoint;
	//initialize standPoint;
	standPoint.alt = 0;
	standPoint.lat = 0;
	standPoint.lon = 0;

	if(allLines.size() != 0)
	{
		standPoint = (*(*allLines.begin()).begin())[0];

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

		database_gp->getAllVectors_clear(allLines, lineAttr);

		engine3DPtr->SwapLineBuffer();
		engine3DPtr->SwapRoadLineBuffer();
		engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
		engine3DPtr->SwapServerEyeBuffer();
	}
	while(1)
	{
		//get all the furniture
		signInfo_t signInfo[200];
        int numSign;

		list<list<furAttributes_t>> furnitureList;
        database_gp->getAllFurnitures(furnitureList);

		convFurToSignInfo(furnitureList, standPoint, signInfo, &numSign);
		furnitureList.clear();

		engine3DPtr->AddSignInfo(numSign,signInfo);
        engine3DPtr->SwapSignBuffer();

		WaitForSingleObject(g_readySema_Redraw, INFINITE);  

	}


}