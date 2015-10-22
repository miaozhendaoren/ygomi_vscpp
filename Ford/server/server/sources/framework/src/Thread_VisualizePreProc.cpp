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
#include "databaseServer.h" // databaseServer
#include "databaseDef.h"
#include "appInitCommon.h"
#include "VisualizeControl.h"
#include "configure.h"
#include "TimeStamp.h"
//#include "roadSideVectorGen.h"

using namespace ns_database;
using namespace std;
//point3DFloat_t pointVecBuf[MAX_BUFFER_DEPTH_DRAW_LINE_POINT];
vector<point3DFloat_t> pointVecBuf;

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
	default:
		number = 41;
		break;
	}
#else if((RD_LOCATION&RD_NATION_MASK) == RD_UNIT_STATES)

	number = type;
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

void coordinateChange(point3D_t* standPoint, point3D_t* changePoint, point3DFloat_t* outPoint)
{
	GLfloat dif_x = changePoint->lat - standPoint->lat;
	GLfloat dif_y = changePoint->lon - standPoint->lon;
	GLfloat dif_z = changePoint->alt - standPoint->alt;
	float latitude = (standPoint->lat)*PI/180;

	outPoint->x = dif_x*COEFF_DD2METER;  //latitude
	outPoint->z = dif_y*(111413*cos(latitude)-94*cos(3*latitude));  //longitude
	outPoint->y = 0;//dif_z;
}

void changeDataBaseCoord(point3D_t* changePoint, point3DFloat_t* outPoint)
{
    outPoint->x = changePoint->lat;
    outPoint->y = changePoint->alt;
    outPoint->z = changePoint->lon;
}

void convFurToSignInfo(list<list<furAttributesServer_t>>& furnitureList, 
                       vector<signInfo_t> &signInfo)
{
    //int numSignLoc = 0;

    list<list<furAttributesServer_t>>::iterator segIter = furnitureList.begin();

    while(segIter != furnitureList.end())
    {
        list<furAttributesServer_t>::iterator furIter = (*segIter).begin();

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
				signInfo.push_back(signTemp);
            }
            furIter++;
        }
        segIter++;
    }
}

void generateRoadSideVec(vector<point3DFloat_t> &midVec, float width, vector<point3DFloat_t> &leftVec, vector<point3DFloat_t> &rightVec)
{
	leftVec.clear();
	rightVec.clear();
	point3DFloat_t point1;
	point3DFloat_t point2;
	int lineIndex = 0;
	float cosAlpha;
	float sinAlpha;

	leftVec.assign(midVec.size(), point1);
	rightVec.assign(midVec.size(), point2);

	for(lineIndex = 0; lineIndex < (midVec.size()-1); lineIndex++)
	{
		point1 = midVec[lineIndex];
		point2 = midVec[lineIndex+1];

		float lonDis = point1.z - point2.z;
		float latDis = point1.x - point2.x;

		float rou = sqrt(lonDis*lonDis + latDis*latDis);
		cosAlpha = latDis/rou;
		sinAlpha = lonDis/rou;

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
	float tan_a = dif_z/dif_x;
	float shift_z = sqrt(width*width/(1+tan_a*tan_a));
	float shift_x = tan_a*shift_z;

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
	engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
	engine3DPtr->SwapServerEyeBuffer();

	//static int testNum = 0;

	while(1)
	{

		//get all the lines
		list<list<vector<point3D_t>>> allLines; // segment list / vector list / point list / point
		list<list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
		database_gp->getAllVectors(allLines, lineAttr);
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,1);
		if(allLines.size() != 0)
		{
			//standPoint = (*(*allLines.begin()).begin())[0];

			list<list<vector<point3D_t>>>::iterator lineInSegIter = allLines.begin();

			RD_ADD_TS(tsFunId_eThread_Visual_Pre,2);
			// For each segment
			while(lineInSegIter != allLines.end())
			{
				list<vector<point3D_t>>::iterator lineIter = (*lineInSegIter).begin();

				int lineNum = (*lineInSegIter).size();
				int lineIdx = 0;
				point3DFloat_t lastPos;
				int drawNum;

				if(lineNum >= 2)
				{
					// For each vector
					while(lineIter != (*lineInSegIter).end())
					{
						lineTypeEnum_t lineStyle = lineTypeEnum_dash;// Not used //(lineTypeEnum_t)(*lineAttrIter).lineStyle;
            
						int numberPoint = lineIter->size();
						point3DFloat_t tempPoint;

						for(int pointIdx = 0; pointIdx < numberPoint; pointIdx++)
						{
							//coordinateChange(&standPoint,&(*lineIter)[pointIdx], &tempPoint);
							changeDataBaseCoord(&(*lineIter)[pointIdx], &tempPoint);
							pointVecBuf.push_back(tempPoint);
						}
				    
						baseColor_t color;
						color.R = 0.25;
						color.G = 0.25;
						color.B = 0.25;
						baseColor_t colorLine;
						colorLine.R = 0;
						colorLine.G = 1;
						colorLine.B = 0;
						quadInfo_t quadBuf;
						quadBuf.color = colorLine;

						//engine3DPtr->AddOneLineInfo(lineTypeEnum_solid, colorLine, pointVecBuf);
						//engine3DPtr->AddOneRoadLineInfo(lineStyle,color, pointVecBuf);

						for(int index = 0; index < (numberPoint-1); index++)
						{
							extractQuadInfo((pointVecBuf[index]), pointVecBuf[index+1], 0.03, index, &quadBuf,0.005);
							engine3DPtr->AddQuadInfo(1,&quadBuf);
						}

						if(lineIdx == 0)
						{
							lastPos = pointVecBuf[0];
							drawNum = (*lineIter)[0].count;
						}else
						{
							point3DFloat_t drawPos;
							drawPos.x = (lastPos.x + pointVecBuf[0].x)/2;
							drawPos.y = (lastPos.y + pointVecBuf[0].y)/2;
							drawPos.z = (lastPos.z + pointVecBuf[0].z)/2;

							drawServerCharInfo_t testChar;

							generateRoadChar(drawPos, drawNum, testChar);

							engine3DPtr->AddOneServerCharInfo(testChar);
							
							lastPos = pointVecBuf[0];
							drawNum = (*lineIter)[0].count;
							
						}

						

						if((lineIdx == 0)||(lineIdx == (lineNum-1)))
						{
							engine3DPtr->AddOneRoadLineInfo(lineStyle,color, pointVecBuf);
						}

						//draw the paint of the line
						{
							baseColor_t color;
							color.R = 1.0;
							color.G = 0.95;
							color.B = 0.9;

							quadInfo_t quadBuf;
							quadBuf.color = color;
							bool contiFlag = false;

							for(int index = 0; index < (numberPoint-1); index+=4)
							{
								//extractQuadInfo(pointVecBuf[2*index], pointVecBuf[2*index+1], 0.2, false, &quadBuf);
								if(((*lineIter)[index].paintFlag>=0.5)&&((*lineIter)[index+1].paintFlag>=0.5))
								{
									extractQuadInfo((pointVecBuf[index]), pointVecBuf[index+1], 0.2, contiFlag, &quadBuf, 0.01);
									contiFlag = true;
									engine3DPtr->AddQuadInfo(1,&quadBuf);
								}else
								{
									contiFlag = false;
								}
							}
						}

						pointVecBuf.clear();

						lineIter++;
						lineIdx++;
					}
				}

				lineInSegIter++;
			}

			database_gp->getAllVectors_clear(allLines, lineAttr);
		}

		engine3DPtr->SwapLineBuffer();
		engine3DPtr->SwapRoadLineBuffer();
		engine3DPtr->SwapServerCharBuffer();

		RD_ADD_TS(tsFunId_eThread_Visual_Pre,3);
		//get the new data to draw the paint
		{
			list<vector<point3D_t>> newDataVec;
			database_gp->getNewDataVec(newDataVec);
			if(0 != newDataVec.size())
			{
				list<vector<point3D_t>>::iterator newDataIter = newDataVec.begin();
				while(newDataIter != newDataVec.end())
				{
					point3DFloat_t tempPoint;
					for(int pointIdx = 0; pointIdx < (*newDataIter).size(); pointIdx++)
					{
						//coordinateChange(&standPoint,&(*newDataIter)[pointIdx], &tempPoint);
                        changeDataBaseCoord(&(*newDataIter)[pointIdx], &tempPoint);
						tempPoint.y += 0.01;
						pointVecBuf.push_back(tempPoint);
					}

					//draw the paint of the line
					{
						baseColor_t color;
						color.R = 1.0;
						color.G = 0.0;
						color.B = 0.0;

						quadInfo_t quadBuf;
						quadBuf.color = color;
						bool contiFlag = false;

						for(int index = 0; index < ((*newDataIter).size()-1); index+=4)
						{
							//extractQuadInfo(pointVecBuf[2*index], pointVecBuf[2*index+1], 0.2, false, &quadBuf);
							if(((*newDataIter)[index].paintFlag>=0.5)&&((*newDataIter)[index+1].paintFlag>=0.5)&&
								((*newDataIter)[index].paintFlag<=1.0)&&((*newDataIter)[index+1].paintFlag<=1.0))
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
					newDataIter++;
				}
			}

		}
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,4);
		engine3DPtr->SwapQuadBuffer();
		{
			//get all the furniture
			vector<signInfo_t> signInfo;
			//int numSign;

			list<list<furAttributesServer_t>> furnitureList;
			database_gp->getAllFurnitures(furnitureList);

			convFurToSignInfo(furnitureList, signInfo);
			furnitureList.clear();

			engine3DPtr->AddSignInfo(signInfo);
			engine3DPtr->SwapSignBuffer(); 
		}
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,5);
		WaitForSingleObject(g_readySema_Redraw, INFINITE); 
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,6);
	}//end while(1)


}