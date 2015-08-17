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

using namespace ns_database;
using namespace std;
using namespace ns_historyLine;


//point3DFloat_t pointVecBuf[MAX_BUFFER_DEPTH_DRAW_LINE_POINT];
vector<point3DFloat_t> pointVecBuf;

//baseColor_t lineColor[5];

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
	float latitude = (standPoint->lat)*PI/180;

	outPoint->x = dif_x*COEFF_DD2METER;  //latitude
	outPoint->z = dif_y*(111413*cos(latitude)-94*cos(3*latitude));  //longitude
	outPoint->y = 0;//dif_z;
}

void changeDataBaseCoord(point3D_t* changePoint, point3DFloat_t* outPoint)
{
    outPoint->x = changePoint->lat;
    outPoint->y = 0;
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

void convFurToSignInfo(list<list<furAttributesInVehicle_t>>& furnitureList, 
	point3D_t& standPoint, 
	signInfo_t* signInfo, 
	int* numSign)
{
	int numSignLoc = 0;

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
				signInfo[numSignLoc].type = convertSignType(furInfo->type);
				signInfo[numSignLoc].rotAngle = furInfo->angle * 180 / PI;
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

void gpsTimer(int value)
{
	if(2 == value)
	{
		ReleaseSemaphore(g_readySema_GPS, 1 ,NULL);
		glutTimerFunc((unsigned int)(1000),&gpsTimer,2);
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

void extractQuadInfo(point3DFloat_t startPoint, point3DFloat_t endPoint, float width, bool contiFlag, quadInfo_t *quadInfo)
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
		quadInfo->vertex[0].y = startPoint.y + 0.01;//startPoint.y;
		quadInfo->vertex[0].z = startPoint.z + shift_z;

		quadInfo->vertex[1].x = startPoint.x + shift_x;
		quadInfo->vertex[1].y = startPoint.y + 0.01;//startPoint.y;
		quadInfo->vertex[1].z = startPoint.z - shift_z;
	}

	quadInfo->vertex[2].x = endPoint.x + shift_x;
	quadInfo->vertex[2].y = endPoint.y + 0.01;//endPoint.y;
	quadInfo->vertex[2].z = endPoint.z - shift_z;

	quadInfo->vertex[3].x = endPoint.x - shift_x;
	quadInfo->vertex[3].y = endPoint.y + 0.01;//endPoint.y;
	quadInfo->vertex[3].z = endPoint.z + shift_z;

	startX_left = quadInfo->vertex[3].x;
	startY_left = quadInfo->vertex[3].y;
	startZ_left = quadInfo->vertex[3].z;

	startX_right = quadInfo->vertex[2].x;
	startY_right = quadInfo->vertex[2].y;
	startZ_right = quadInfo->vertex[2].z;
}

unsigned int __stdcall Thread_VisualizePreProc(void *data)
{
	bool standPointFlag = false;
	point3D_t standPoint;

	eyeLookAt_t eyeInfo[200];
	eyeLookAt_t eyeInfoAhead[200];

	// Points for look ahead view
	point3D_t gpsAheadOld, gpsAheadNew;
	point3D_t gpsCurrent;
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

	newPointF.x = 0;
	newPointF.y = 0;
	newPointF.z = 0;

	glutTimerFunc((unsigned int)(1000),&gpsTimer,2);

	while(1)
	{
		//wait for GPS thread to get GPS signal
		WaitForSingleObject(g_readySema_GPS, INFINITE); 
		if(!standPointFlag)
		{
			standPoint.lat = 42.296855933108084;//42.296853333333331;//gGpsInfo.dLatitude;
			standPoint.lon = -83.213250649943689;//-83.213250000000002;//gGpsInfo.dLongitude;
			standPoint.alt = 0.0;//gGpsInfo.altitude;

			serverEyeInfo[0].eyePosition.x = 328;
			serverEyeInfo[0].eyePosition.y = 300;
			serverEyeInfo[0].eyePosition.z = -545;

			serverEyeInfo[0].lookatPosition.x = 328;
			serverEyeInfo[0].lookatPosition.y = 0;
			serverEyeInfo[0].lookatPosition.z = -545;
			standPointFlag = true;

			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
		}

#if 1
		//get all lines
		{
			list<list<vector<point3D_t>>> allLines; // segment list / vector list / point list / point
			list<list<lineAttributes_t>> lineAttr; // segment list / vector list / attributes
			database_gp->getAllVectors(allLines, lineAttr);

			//point3D_t standPoint = (*(*allLines.begin()).begin())[0];
			if(allLines.size() != 0)
			{
				list<list<vector<point3D_t>>>::iterator lineInSegIter = allLines.begin();
				list<list<lineAttributes_t>>::iterator lineAttrInSegIter = lineAttr.begin();

				// For each segment
				while(lineInSegIter != allLines.end())
				{
					list<vector<point3D_t>>::iterator lineIter = (*lineInSegIter).begin();
					list<lineAttributes_t>::iterator lineAttrIter = (*lineAttrInSegIter).begin();

					int lineNum = (*lineInSegIter).size();
					int lineIdx = 0;
					bool needGen = true;
					if(2 == lineNum)
					{
						needGen = false;
					}

					// For each vector
					while(lineIter != (*lineInSegIter).end())
					{
						lineTypeEnum_t lineStyle = (lineTypeEnum_t)(*lineAttrIter).lineStyle;

						int numberPoint = (*lineAttrIter).numPoints;
						point3DFloat_t tempPoint;

						for(int pointIdx = 0; pointIdx < numberPoint; pointIdx++)
						{
							//coordinateChange(&standPoint,&(*lineIter)[pointIdx], &(pointVecBuf[pointIdx]));
							//coordinateChange(&standPoint,&(*lineIter)[pointIdx], &tempPoint);
                            changeDataBaseCoord(&(*lineIter)[pointIdx], &tempPoint);
							pointVecBuf.push_back(tempPoint);
						}

						if(needGen)
						{
							if(1 == lineIdx)
							{
								vector<point3DFloat_t> leftVec;
								vector<point3DFloat_t> rightVec;

								generateRoadSideVec(pointVecBuf, 4.5, leftVec, rightVec);
								baseColor_t color;
								color.R = 0.25;
								color.G = 0.25;
								color.B = 0.25;
								engine3DPtr->AddOneRoadLineInfo(lineStyle,color, leftVec);
								engine3DPtr->AddOneRoadLineInfo(lineStyle,color, rightVec);
							}
						}else
						{
							baseColor_t color;
							color.R = 0.25;
							color.G = 0.25;
							color.B = 0.25;
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

							for(int index = 0; index < (numberPoint-1); index++)
							{
								//extractQuadInfo(pointVecBuf[2*index], pointVecBuf[2*index+1], 0.2, false, &quadBuf);
								if(((*lineIter)[index].paintFlag)&&((*lineIter)[index+1].paintFlag))
								{
									extractQuadInfo((pointVecBuf[index]), pointVecBuf[index+1], 0.2, contiFlag, &quadBuf);
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
						lineAttrIter++;
						lineIdx++;
					}

					lineInSegIter++;
					lineAttrInSegIter++;
				}

				database_gp->getAllVectors_clear(allLines, lineAttr);

#if 0
				//get the history reference line
				{
					list<lineInfoPerVector_t> historyLine;
					historyInfoP.getHistoryBuffer(historyLine);

					list<lineInfoPerVector_t>::iterator lineIter = historyLine.begin();
					int sizeList = historyLine.size();
					int lineIdx = 0;
					while(lineIter != historyLine.end())
					{
						lineTypeEnum_t lineStyle = (lineTypeEnum_t)(*lineIter).lineStyle;
						int numPoint = (*lineIter).pointNum;
						for(int pointIdx = 0; pointIdx < numPoint; pointIdx++)
						{
							coordinateChange(&standPoint,&(*lineIter).allGps[pointIdx], &(pointVecBuf[pointIdx]));
						}

						if((lineTypeEnum_road_line == lineStyle)||(lineTypeEnum_roadside_line == lineStyle))
						{
							baseColor_t color;
							color.R = 0.25;
							color.G = 0.25;
							color.B = 0.25;
							engine3DPtr->AddOneRoadLineInfo(numPoint,lineStyle,color,pointVecBuf);
						}else
						{
							baseColor_t color;
							color.R = 0.0;
							color.G = ((float)(lineIdx+2))/(sizeList+3);
							color.B = 0.0;
							if(lineTypeEnum_invalid == lineStyle)
							{
								//draw reference line as solid
								lineStyle = lineTypeEnum_solid;
							}
							engine3DPtr->AddOneLineInfo(numPoint, lineStyle, color, pointVecBuf);
						}

						lineIdx++;
						lineIter++;
					}

					{
						list<point3D_t> gpsList;
						historyInfoP.getGpsBuffer(gpsList);
						int numPoint = gpsList.size();
						int pointIdx = 0;
						list<point3D_t>::iterator pointIter = gpsList.begin();
						while(pointIter != gpsList.end())
						{
							coordinateChange(&standPoint,&(*pointIter), &(pointVecBuf[pointIdx++]));
							pointIter++;
						}

						baseColor_t color;
						color.R = 0.0;
						color.G = 0.0;
						color.B = 1.0;
						engine3DPtr->AddOneLineInfo(numPoint, lineTypeEnum_solid, color, pointVecBuf);
					}

                }
#endif
			}

			engine3DPtr->SwapLineBuffer();
			engine3DPtr->SwapRoadLineBuffer();
			engine3DPtr->SwapQuadBuffer();
		}
#endif

		//For all the furnitures
		{
			signInfo_t signInfo[200];
			int numSign;

			list<list<furAttributesInVehicle_t>> furnitureList;

			database_gp->getAllFurnitures(furnitureList);

			convFurToSignInfo(furnitureList, standPoint, signInfo, &numSign);

			furnitureList.clear();

			engine3DPtr->AddSignInfo(numSign,signInfo);
			engine3DPtr->SwapSignBuffer();
		}

		//update the car position and look ahead direction
		{
			//point3D_t oldPointD;
			point3D_t newPointD; 

			//oldPointD.lat = gGpsInfo.dLatitudePre;
			//oldPointD.alt = gGpsInfo.altitudePre;
			//oldPointD.lon = gGpsInfo.dLongitudePre;

			newPointD.lat = gGpsInfo.dLatitude;
			newPointD.alt = gGpsInfo.altitude;
			newPointD.lon = gGpsInfo.dLongitude;

			lastPointF = oldPointF;
			oldPointF = newPointF;

			//database_gp->getLookAheadView(&newPointD, 1, &gpsCurrent);
			coordinateChange(&standPoint, &newPointD, &newPointF);

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


			//update look ahead figure. TBD

		}

	}

	return 0;
}