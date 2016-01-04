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
        // fall through
    case 1001:
        // fall through
    case 1002:
        // fall through
    case 1003:
		generateRect(0.4, 4, signInfo.sharp);
		break;
	case 1050:
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
		number = 10; // stop line
		break;
	case 1001:
		number = 11; // crosswalk
		break;
	case 1002:
		number = 12; // stop line2
		break;
	case 1003:
		number = 13; // crosswalk2
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
				signTemp.sideFlag   = furInfo->sideFlag;
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
	vector<uint32> SegIdInSky;
	engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
	engine3DPtr->SwapServerEyeBuffer();

	roadSegConfig_gp->getRoadSegmentInSky(SegIdInSky);
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
			int segmentId = 1;
			RD_ADD_TS(tsFunId_eThread_Visual_Pre,2);
			// For each segment
			while(lineInSegIter != allLines.end())
			{
				list<vector<point3D_t>>::iterator lineIter = (*lineInSegIter).begin();
				float segInSkyHeight = 0;
				vector<point3DFloat_t> startVec;
				vector<point3DFloat_t> endVec;

				//decide if it it sky section
				for(int skyIndex = 0; skyIndex < SegIdInSky.size(); skyIndex++)
				{
					if(segmentId == SegIdInSky[skyIndex])
					{
						segInSkyHeight = 0.01;
						break;
					}
				}

				int lineNum = (*lineInSegIter).size();
				int lineIdx = 0;
				point3DFloat_t lastPos;
				uint32 drawNum;
				
				baseColor_t color;
				color.R = 0.25;
				color.G = 0.25;
				color.B = 0.25;
				triangle_t triBuf;
				triBuf.color = color;
				triBuf.showFlag = true;

				if(lineNum >= 2)
				{
					// For each vector
					while(lineIter != (*lineInSegIter).end())
					{
						lineTypeEnum_t lineStyle = lineTypeEnum_dash;// Not used //(lineTypeEnum_t)(*lineAttrIter).lineStyle;
            
						int numberPoint = lineIter->size();
						point3DFloat_t tempPoint;


						if(numberPoint > 0)
						{
							for(int pointIdx = 0; pointIdx < numberPoint; pointIdx++)
							{
								//coordinateChange(&standPoint,&(*lineIter)[pointIdx], &tempPoint);
								changeDataBaseCoord(&(*lineIter)[pointIdx], &tempPoint);
								tempPoint.y += segInSkyHeight;
								pointVecBuf.push_back(tempPoint);
							}
				    
							baseColor_t colorLine;
							colorLine.R = 0;
							colorLine.G = 1;
							colorLine.B = 0;
							quadInfo_t quadBuf;
							quadBuf.color = colorLine;

							engine3DPtr->AddOneLineInfo(lineTypeEnum_solid, colorLine, true, pointVecBuf);
							//engine3DPtr->AddOneRoadLineInfo(lineStyle,color, pointVecBuf);

							//for(int index = 0; index < (numberPoint-1); index++)
							//{
							//	extractQuadInfo((pointVecBuf[index]), pointVecBuf[index+1], 0.03, index, &quadBuf,0.005);
							//	engine3DPtr->AddQuadInfo(1,&quadBuf);
							//}

							if(lineIdx == 0)
							{
								lastPos = pointVecBuf[0];
								int count = (*lineIter)[0].count;
								drawNum = (count == LANE_END_LINE_FLAG)?count:CLEAN_LANE_DIR_BIT(count);
								engine3DPtr->AddOneRoadLineInfo(lineStyle,color,true, pointVecBuf);
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
									engine3DPtr->AddOneRoadLineInfo(lineStyle,color, true, pointVecBuf);
									startVec.push_back(pointVecBuf[0]);
									endVec.push_back(pointVecBuf[pointVecBuf.size()-1]);
								}
							
								lastPos = pointVecBuf[0];
								int count = (*lineIter)[0].count;
								drawNum = (count == LANE_END_LINE_FLAG)?count:CLEAN_LANE_DIR_BIT(count);
								if((LANE_END_LINE_FLAG == drawNum) || (lineIdx == (lineNum-1)))
								{
									engine3DPtr->AddOneRoadLineInfo(lineStyle,color, true, pointVecBuf);
									
									//add triangle for other road place
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
								quadBuf.showFlag = true;
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
						}//end for if(numberPoint > 0)

						lineIter++;
						lineIdx++;
					}
				}

				lineInSegIter++;
				segmentId++;
			}

			database_gp->getAllVectors_clear(allLines, lineAttr);
		}

		//engine3DPtr->SwapLineBuffer();
		//engine3DPtr->SwapRoadLineBuffer();
		//engine3DPtr->SwapServerCharBuffer();

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
					if ((*newDataIter).empty())
					{
						newDataIter++;
						continue;
					}

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
						quadBuf.showFlag = true;
						bool contiFlag = false;

						int numOfPnts = (*newDataIter).size();
						for(int index = 0; index < (numOfPnts-1); index+=4)
						{
							//extractQuadInfo(pointVecBuf[2*index], pointVecBuf[2*index+1], 0.2, false, &quadBuf);
							if(((*newDataIter)[index].paintFlag>=0.5)&&((*newDataIter)[index+1].paintFlag>=0.5)&&
								((*newDataIter)[index].paintFlag<=1.0)&&((*newDataIter)[index+1].paintFlag<=1.0))
							{
								extractQuadInfo((pointVecBuf[index]), pointVecBuf[index+1], 0.2, contiFlag, &quadBuf,0.001);
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
		//engine3DPtr->SwapQuadBuffer();
		{
			//get all the furniture
			vector<signInfo_t> signInfo;
			//int numSign;

			list<list<furAttributesServer_t>> furnitureList;
			database_gp->getAllFurnitures(furnitureList);

			convFurToSignInfo(furnitureList, signInfo);
			furnitureList.clear();

			engine3DPtr->AddSignInfo(signInfo);
			//engine3DPtr->SwapSignBuffer(); 
		}
		engine3DPtr->Swap3DBuffers();
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,5);
		WaitForSingleObject(g_readySema_Redraw, INFINITE); 
		RD_ADD_TS(tsFunId_eThread_Visual_Pre,6);
	}//end while(1)


}