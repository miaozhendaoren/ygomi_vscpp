/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  databaseMath.cpp
* @brief Math functions for database
*
* Change Log:
*      Date                Who             What
*      2015/1/13         Linkun Xu        Create
*******************************************************************************
*/

#include <math.h>

#include "database.h"

using namespace std;

namespace ns_database
{
    void database::calcAngle(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT float* angle)
    {
        double distX = gpsPre->lon - gpsCurr->lon;
        double distY = gpsPre->lat - gpsCurr->lat;

        float degree = atan2(distX, distY);

        *angle = degree;
    }

    void database::calcNormalAngle(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT float* angle)
    {
        double distX = gpsPre->lon - gpsCurr->lon;
        double distY = gpsPre->lat - gpsCurr->lat;

        float degree = atan2(distY, distX);

        *angle = degree;
    }

	void database::calcDistance(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT double* dist)
	{
        double distX = gpsPre->lon - gpsCurr->lon;
        double distY = gpsPre->lat - gpsCurr->lat;

		*dist = sqrt(distX * distX + distY * distY) * COEFF_DD2METER;
	}

	void database::calcDistancePointToLine(IN point3D_t* gpsLinePointMP, 
		                                   IN point3D_t* gpsLinePointNP, 
										   IN point3D_t* gpsPointPP, 
										   OUT double* dist)
	{
		// Calculate distance from point P to line MN

		double x1 = gpsLinePointMP->lon;
		double y1 = gpsLinePointMP->lat;
		double x2 = gpsLinePointNP->lon;
		double y2 = gpsLinePointNP->lat;

		double x0 = gpsPointPP->lon;
		double y0 = gpsPointPP->lat;

		if((x1 == x2) && (y1 == y2))
		// M and N are same point
		{
			calcDistance(gpsLinePointMP, gpsPointPP, dist);
		}
		else
		{		
			double distMP, distNP, distMN, distLineP;

			calcDistance(gpsLinePointMP, gpsPointPP, &distMP);
			calcDistance(gpsLinePointNP, gpsPointPP, &distNP);
			calcDistance(gpsLinePointMP, gpsLinePointNP, &distMN);

			if(distMP >= distNP)
			{
				if((distMP * distMP) > (distMN * distMN + distNP * distNP))
				{
					*dist = distNP;
				}else
				{
					// Change line to A*x_P + B*y_P + C = 0
					double A = y1 - y2;
					double B = x2 - x1;
					double C = -B * y1 - A * x1;
			
					*dist = abs(A * x0 + B * y0 + C) * COEFF_DD2METER / sqrt(A * A + B * B);
				}
			}else
			{
				if((distNP * distNP) > (distMN * distMN + distMP * distMP))
				{
					*dist = distMP;
				}else
				{
					// Change line to A*x_P + B*y_P + C = 0
					double A = y1 - y2;
					double B = x2 - x1;
					double C = -B * y1 - A * x1;
			
					*dist = abs(A * x0 + B * y0 + C) * COEFF_DD2METER / sqrt(A * A + B * B);
				}
			}
		}
	}

	void database::calcProjectPointOnLine(IN point3D_t* gpsLinePointMP, 
		                                  IN point3D_t* gpsLinePointNP, 
									      IN point3D_t* gpsPointPP, 
									      OUT point3D_t* projectPointP)
	{
		// Calculate the project point of point P on line MN

		double x1 = gpsLinePointMP->lon;
		double y1 = gpsLinePointMP->lat;
		double x2 = gpsLinePointNP->lon;
		double y2 = gpsLinePointNP->lat;

		double x0 = gpsPointPP->lon;
		double y0 = gpsPointPP->lat;

		if((x1 == x2) && (y1 == y2))
		// M and N are same point
		{
			*projectPointP = *gpsLinePointMP;
		}
		else
		{
			point3D_t projectPointTmp;

			if(x1 == x2)
			{
				projectPointTmp.lon = x1;
				projectPointTmp.lat = y0;
			}
			else if(y1 == y2)
			{
				projectPointTmp.lon = x0;
				projectPointTmp.lat = y1;
			}
			else
			{
				double k = (y2 - y1) / (x2 - x1);

				projectPointTmp.lon = (x0 + k*k*x1 + k*y0 - k*y1)/(k*k + 1);
				projectPointTmp.lat = k*(projectPointTmp.lon - x1) + y1;
			}

			double distMP, distNP, distLineP;
			calcDistance(gpsLinePointMP, gpsPointPP, &distMP);
			calcDistance(gpsLinePointNP, gpsPointPP, &distNP);
			calcDistance(&projectPointTmp, gpsPointPP, &distLineP);

			double minDist = min(min(distMP, distNP), distLineP);

			if(distMP == minDist)
			{*projectPointP = *gpsLinePointMP;}
			else if(distNP == minDist)
			{*projectPointP = *gpsLinePointNP;}
			else// if(distLineP == minDist)
			{*projectPointP = projectPointTmp;}
		}

		projectPointP->alt = 0.0;
	}

	void database::calcGpsBackOnLine(IN point3D_t* gpsLinePointMP, 
									 IN point3D_t* gpsLinePointNP, 
									 IN double     backDist, 
									 OUT point3D_t* gpsOutP)
	{
		// Calculate the GPS point on line MN, and a certain distance from N
		double xM = gpsLinePointMP->lon;
		double yM = gpsLinePointMP->lat;
		double xN = gpsLinePointNP->lon;
		double yN = gpsLinePointNP->lat;

		double x2 = (xM - xN) * (xM - xN);
		double y2 = (yM - yN) * (yM - yN);

		double xP, yP;

		double tempDy = backDist / sqrt(x2/y2 + 1) / COEFF_DD2METER;

		if(yM < yN)
		{
			yP = yN - tempDy;
		}else
		{
			yP = yN + tempDy;
		}

		double tempDx = backDist / sqrt(y2/x2 + 1) / COEFF_DD2METER;

		if(xM < xN)
		{
			xP = xN - tempDx;
		}else
		{
			xP = xN + tempDx;
		}

		gpsOutP->lat = yP;
		gpsOutP->lon = xP;
		gpsOutP->alt = 0;
	}

    void database::calcGpsRoadSide(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, IN uint8 side, IN float width, OUT point3D_t* gpsRight)
    {
        double distX = gpsCurr->lon - gpsPre->lon;
        double distY = gpsCurr->lat - gpsPre->lat;

        double slopeN   = width / COEFF_DD2METER;
        double slopeD_x;
		if((distX * distX) != 0)
		{	slopeD_x = sqrt(1 + (distY * distY) / (distX * distX));}
		else
		{	slopeD_x = 1;}
        
        double offset_x;

        if(distY >= 0)
        {
            offset_x = slopeN / slopeD_x;
        }
        else
        {
            offset_x = -slopeN / slopeD_x;
        }

        if(side == 1)
        // right side
        {
            gpsRight->lon = offset_x + gpsCurr->lon;
        }
        else
        {
            gpsRight->lon = -offset_x + gpsCurr->lon;
        }

        double slopeD_y = sqrt(1 + (distX * distX) / (distY * distY));

        double offset_y;

        if(distX >= 0)
        {
            offset_y = -slopeN / slopeD_x;
        }
        else
        {
            offset_y = slopeN / slopeD_x;
        }

        if(side == 1)
        // right side
        {
            gpsRight->lat = offset_y + gpsCurr->lat;
        }
        else
        {
            gpsRight->lat = -offset_y + gpsCurr->lat;
        }

        gpsRight->alt = gpsCurr->alt;
    }

    bool database::checkGpsInRange(IN point3D_t* gpsA, IN point3D_t* gpsB, IN double distThresh)
    {
        // Distances in degrees
        double distX = (gpsA->lat - gpsB->lat); // FIXME: need to update the calculation
        double distY = (gpsA->lon - gpsB->lon);
        double distZ = (gpsA->alt - gpsB->alt);

        double dist = sqrt(distX * distX + distY * distY);

        if(dist > distThresh)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void database::getLookAheadView(IN point3D_t* gpsCurrP, IN float distanceIn, OUT point3D_t* gpsAhead)
    {
        WaitForSingleObject(_hMutexMemory,INFINITE);

		double minDist = 1000;
		uint32 minSegId;
		int    minPointIdx;

		point3D_t projectPoint;
		point3D_t aheadPointPre;
		point3D_t aheadPointCur;
		double distSum = 0;

        uint8 existFlag;
        segAttributes_t segmentAttr;
        getSegmentByGps(gpsCurrP, &existFlag, &segmentAttr);

        if(existFlag == 1)
        {
			// Locate current location in vectors
            // Find vectors in the segment
            list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIter = _vectorList.begin();

            // For each segment
            while(vecListIter != _vectorList.end())
            {
                list<vector<vector<tlvCommon_t>>>::iterator pointListIter = (*vecListIter).begin();

                // FIXME: assume the first vector is center of road
                //while(pointListIter != (*vecListIter).end())
                {
                    // Get vector info from the first element
                    lineAttributes_t lineAttrTmp;

                    lineAttrTmp.segmentId = (*pointListIter)[0][0].value;
                    lineAttrTmp.lineId = (*pointListIter)[0][data_lineId_e - data_lineBase_e + 1].value;  // +1 for skipping segment ID info
                    lineAttrTmp.width  = *(float*)(&(*pointListIter)[0][data_lineWidth_e - data_lineBase_e + 1].value);
                    lineAttrTmp.lineStyle  = (*pointListIter)[0][data_lineStyle_e - data_lineBase_e + 1].value;
                    lineAttrTmp.segVersion = (*pointListIter)[0][data_lineSegVersion_e - data_lineBase_e + 1].value;
                    lineAttrTmp.numPoints  = (*pointListIter)[0][data_linePointList_e - data_lineBase_e + 1].length;

                    // Dash line
                    //if(lineAttrTmp.lineStyle == 1)
                    {
                        point3D_t point3D;
                        point3D_t gpsAhead1Tmp, gpsAhead2Tmp;
						double dist;

						// first point in vector
                        gpsAhead1Tmp.lat = *(double *)((*pointListIter)[1][0].value);
                        gpsAhead1Tmp.lon = *(double *)((*pointListIter)[1][1].value);
                        gpsAhead1Tmp.alt = *(double *)((*pointListIter)[1][2].value);

                        for(int pointIdx = 1; pointIdx < lineAttrTmp.numPoints; pointIdx++)
                        {
                            point3D.lat = *(double *)((*pointListIter)[pointIdx+1][0].value);
                            point3D.lon = *(double *)((*pointListIter)[pointIdx+1][1].value);
                            point3D.alt = *(double *)((*pointListIter)[pointIdx+1][2].value);

                            gpsAhead2Tmp = gpsAhead1Tmp;
                            gpsAhead1Tmp = point3D;

							calcDistancePointToLine(&gpsAhead1Tmp, &gpsAhead2Tmp, gpsCurrP, &dist);

							if(dist < minDist)
							{
								minSegId = lineAttrTmp.segmentId;
								minPointIdx = pointIdx;

								minDist = dist;
							}
                        }
                    }

                    //pointListIter++;
                }

                vecListIter++;
            }

			// Calculate look ahead points
			vecListIter = _vectorList.begin();

            // For each segment
            while(vecListIter != _vectorList.end())
            {
                list<vector<vector<tlvCommon_t>>>::iterator pointListIter = (*vecListIter).begin();

                // FIXME: assume the first vector is center of road
                //while(pointListIter != (*vecListIter).end())
                {
                    // Get vector info from the first element
                    lineAttributes_t lineAttrTmp;

                    lineAttrTmp.segmentId = (*pointListIter)[0][0].value;
                    lineAttrTmp.lineId = (*pointListIter)[0][data_lineId_e - data_lineBase_e + 1].value;  // +1 for skipping segment ID info
                    lineAttrTmp.width  = *(float*)(&(*pointListIter)[0][data_lineWidth_e - data_lineBase_e + 1].value);
                    lineAttrTmp.lineStyle  = (*pointListIter)[0][data_lineStyle_e - data_lineBase_e + 1].value;
                    lineAttrTmp.segVersion = (*pointListIter)[0][data_lineSegVersion_e - data_lineBase_e + 1].value;
                    lineAttrTmp.numPoints  = (*pointListIter)[0][data_linePointList_e - data_lineBase_e + 1].length;

                    if(lineAttrTmp.segmentId != minSegId)
					{
						vecListIter++;
						continue;
					}

                    point3D_t point3D;
                    point3D_t gpsAhead1Tmp, gpsAhead2Tmp;
					double dist;

					// Locate the project point
                    {
                        gpsAhead2Tmp.lat = *(double *)((*pointListIter)[minPointIdx][0].value);
                        gpsAhead2Tmp.lon = *(double *)((*pointListIter)[minPointIdx][1].value);
                        gpsAhead2Tmp.alt = *(double *)((*pointListIter)[minPointIdx][2].value);

                        gpsAhead1Tmp.lat = *(double *)((*pointListIter)[minPointIdx+1][0].value);
                        gpsAhead1Tmp.lon = *(double *)((*pointListIter)[minPointIdx+1][1].value);
                        gpsAhead1Tmp.alt = *(double *)((*pointListIter)[minPointIdx+1][2].value);

						// Calculate project point
						calcProjectPointOnLine(&gpsAhead1Tmp, &gpsAhead2Tmp, gpsCurrP, &projectPoint);

						double dist;
						calcDistance(&projectPoint, &gpsAhead1Tmp, &dist);
						distSum += dist;
                    }

					// Look ahead
					list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIterTmp = vecListIter;
					list<vector<vector<tlvCommon_t>>>::iterator pointListIterTmp = pointListIter;
					int pointIdxAhead = minPointIdx;

					while(distSum < distanceIn)
					{
						pointIdxAhead++;

						if(pointIdxAhead >= (*pointListIterTmp)[0][data_linePointList_e - data_lineBase_e + 1].length)
						{
							vecListIterTmp++;

							if(vecListIterTmp == _vectorList.end())
							{
								vecListIterTmp = _vectorList.begin();
							}

							pointIdxAhead = 1;

							pointListIterTmp = (*vecListIterTmp).begin();
						}

						point3D.lat = *(double *)((*pointListIterTmp)[pointIdxAhead+1][0].value);
						point3D.lon = *(double *)((*pointListIterTmp)[pointIdxAhead+1][1].value);
						point3D.alt = *(double *)((*pointListIterTmp)[pointIdxAhead+1][2].value);

						gpsAhead2Tmp = gpsAhead1Tmp;
						gpsAhead1Tmp = point3D;

						double dist;
						calcDistance(&gpsAhead2Tmp, &gpsAhead1Tmp, &dist);
						distSum += dist;
					}

					// Back
					double distBack = distSum - distanceIn;

					calcGpsBackOnLine(&gpsAhead2Tmp, &gpsAhead1Tmp, distBack, gpsAhead);

					ReleaseMutex(_hMutexMemory);
					return;

                    //pointListIter++;
                }

                vecListIter++;
            }
        }

        ReleaseMutex(_hMutexMemory);
    }

	void database::getLookAheadFurnitures(IN point3D_t* gpsCurr, IN float distanceIn, OUT list<furAttributes_t>& furnitureAttrList, OUT list<point3D_t>& pointInRangeList)
	{
		WaitForSingleObject(_hMutexMemory,INFINITE);

		pointInRangeList.clear();

        uint8 existFlag;
        segAttributes_t segmentAttr;
        getSegmentByGps(gpsCurr, &existFlag, &segmentAttr);

        if(existFlag == 1)
        {
            // Find vectors in the segment
            list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIter = _vectorList.begin();

            // For each segment
            while(vecListIter != _vectorList.end())
            {
                list<vector<vector<tlvCommon_t>>>::iterator pointListIter = (*vecListIter).begin();

                // FIXME: assume the first vector is center of road
                //while(pointListIter != (*vecListIter).end())
                {
                    // Get vector info from the first element
                    lineAttributes_t lineAttrTmp;

                    lineAttrTmp.segmentId = (*pointListIter)[0][0].value;
                    lineAttrTmp.lineId = (*pointListIter)[0][data_lineId_e - data_lineBase_e + 1].value;  // +1 for skipping segment ID info
                    lineAttrTmp.width  = *(float*)(&(*pointListIter)[0][data_lineWidth_e - data_lineBase_e + 1].value);
                    lineAttrTmp.lineStyle  = (*pointListIter)[0][data_lineStyle_e - data_lineBase_e + 1].value;
                    lineAttrTmp.segVersion = (*pointListIter)[0][data_lineSegVersion_e - data_lineBase_e + 1].value;
                    lineAttrTmp.numPoints  = (*pointListIter)[0][data_linePointList_e - data_lineBase_e + 1].length;

                    // Dash line
                    //if(lineAttrTmp.lineStyle == 1)
                    {
                        point3D_t gpsAhead;
						point3D_t gpsAheadPre1;
						point3D_t gpsAheadPre2;

						gpsAheadPre1.lat = 0;
						gpsAheadPre1.lon = 0;
						gpsAheadPre1.alt = 0;
						gpsAheadPre2 = gpsAheadPre1;
						gpsAhead = gpsAheadPre1;

                        for(int pointIdx = 0; pointIdx < lineAttrTmp.numPoints; pointIdx++)
                        {
							gpsAheadPre2 = gpsAheadPre1;
							gpsAheadPre1 = gpsAhead;
                            gpsAhead.lat = *(double *)((*pointListIter)[pointIdx+1][0].value);
                            gpsAhead.lon = *(double *)((*pointListIter)[pointIdx+1][1].value);
                            gpsAhead.alt = *(double *)((*pointListIter)[pointIdx+1][2].value);

							// Current location tracked
                            if(checkGpsInRange(&gpsAhead, gpsCurr, _distThreshMid))
                            {
                                list<list<vector<vector<tlvCommon_t>>>>::iterator vecListIterTmp = vecListIter;
                                list<vector<vector<tlvCommon_t>>>::iterator pointListIterTmp = pointListIter;

                                int pointIdxAhead = pointIdx;

                                double distSum = 0;

								// Log GPS points
								pointInRangeList.push_back(gpsAheadPre2);
								pointInRangeList.push_back(gpsAheadPre1);
								pointInRangeList.push_back(gpsAhead);

                                while(distSum < distanceIn)
                                {
									double dist;
									calcDistance(&gpsAhead, &gpsAheadPre1, &dist);
                                    distSum += dist;

                                    pointIdxAhead++;

                                    if(pointIdxAhead >= (*pointListIterTmp)[0][data_linePointList_e - data_lineBase_e + 1].length)
                                    {
                                        vecListIterTmp++;

                                        if(vecListIterTmp == _vectorList.end())
                                        {
                                            vecListIterTmp = _vectorList.begin();
                                        }

                                        pointIdxAhead = 1;

                                        pointListIterTmp = (*vecListIterTmp).begin();
                                    }

                                    gpsAhead.lat = *(double *)((*pointListIterTmp)[pointIdxAhead+1][0].value);
                                    gpsAhead.lon = *(double *)((*pointListIterTmp)[pointIdxAhead+1][1].value);
                                    gpsAhead.alt = *(double *)((*pointListIterTmp)[pointIdxAhead+1][2].value);

									// Log GPS points
									pointInRangeList.push_back(gpsAhead);
                                }

								// Check if furniture in range
								list<list<furAttributes_t>>::iterator segIter = _furnitureList.begin();
								while(segIter != _furnitureList.end())
								{
									uint32 segId = (*(*segIter).begin()).segId;
									
									list<furAttributes_t> furnitureAttrListTmp;

									getFurnitureBySegId(segId, furnitureAttrListTmp);

									if(furnitureAttrListTmp.size() > 0)
									{
										list<furAttributes_t>::iterator furnIter = furnitureAttrListTmp.begin();

										// Check all furnitures in the segment
										while(furnIter != furnitureAttrListTmp.end())
										{
											if((*furnIter).location_used)
											{
												list<point3D_t>::iterator pointIter = pointInRangeList.begin();
												point3D_t prePoint = (*pointIter);
												point3D_t curPoint = prePoint;
												point3D_t calFurPosition;
												pointIter++;

												while(pointIter != pointInRangeList.end())
												{
													prePoint = curPoint;
													curPoint = (*pointIter);
													calcGpsRoadSide(&prePoint, &curPoint, (*furnIter).sideFlag, 5, &calFurPosition);
													if(checkGpsInRange(&(*furnIter).location, &calFurPosition, _distThreshMid))
													{
														furnitureAttrList.push_back((*furnIter));
														break;
													}
													pointIter++;
												}
											}
											furnIter++;
										}
									}

									segIter++;
								}

                                ReleaseMutex(_hMutexMemory);
                                return;
                            }
                        }
                    }

                    pointListIter++;
                }

                vecListIter++;
            }
        }

		ReleaseMutex(_hMutexMemory);
	}
}
