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
    void calcAngle(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT float* angle)
    {
        double distX = gpsPre->lon - gpsCurr->lon;
        double distY = gpsPre->lat - gpsCurr->lat;

        float degree = (float)atan2(distX, distY);

        *angle = degree;
    }

    void calcNormalAngle(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT float* angle)
    {
        double distX = gpsPre->lon - gpsCurr->lon;
        double distY = gpsPre->lat - gpsCurr->lat;

        float degree = (float)atan2(distY, distX);

        *angle = degree;
    }

	void calcGpsDistance(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT double* distInMeter)
	{
        double distX = (gpsPre->lat - gpsCurr->lat) * COEFF_DD2METER;
        double latitude = (gpsPre->lat)*PI/180;

        double distY = (gpsPre->lon - gpsCurr->lon) * (111413*cos(latitude)-94*cos(3*latitude));
        //double distZ = (gpsPre->alt - gpsCurr->alt);

        *distInMeter = sqrt(distX * distX + distY * distY);
	}

    void calcRelDistance(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT double* distInMeter)
    {
        double distX = gpsPre->lat - gpsCurr->lat;
        double distY = gpsPre->lon - gpsCurr->lon;
        //double distZ = (gpsPre->alt - gpsCurr->alt);

        *distInMeter = sqrt(distX * distX + distY * distY);
    }

    void calcRelDistance(IN pointRelative3D_t* gpsPre, IN pointRelative3D_t* gpsCurr, OUT double* distInMeter)
    {
        double distX = gpsPre->y - gpsCurr->y;
        double distY = gpsPre->x - gpsCurr->x;
        //double distZ = (gpsPre->alt - gpsCurr->alt);

        *distInMeter = sqrt(distX * distX + distY * distY);
    }

    void calcRelativeLocation(IN point3D_t* standPoint, IN point3D_t* changePoint, OUT pointRelative3D_t* outPoint)
    {
	    double dif_x = changePoint->lon - standPoint->lon;
        double dif_y = changePoint->lat - standPoint->lat;
	    double dif_z = changePoint->alt - standPoint->alt;
	    double latitude = (standPoint->lat)*PI/180;

        outPoint->x = (float)(dif_x*(111413*cos(latitude)-94*cos(3*latitude)));  //longitude
	    outPoint->y = (float)(dif_y*COEFF_DD2METER);  //latitude
	    outPoint->z = (float)(dif_z);
    }

    void calcGpsFromRelativeLocation(IN point3D_t* standPoint, IN pointRelative3D_t* relPoint, OUT point3D_t* outGpsPoint)
    {
        double latitude = (standPoint->lat)*PI/180;

        double diff_x = relPoint->x / (111413*cos(latitude)-94*cos(3*latitude));
        double diff_y = relPoint->y / COEFF_DD2METER;
        double diff_z = relPoint->z;

        outGpsPoint->lon = standPoint->lon + diff_x;
        outGpsPoint->lat = standPoint->lat + diff_y;
        outGpsPoint->alt = standPoint->alt + diff_z;
    }

	void calcDistancePointToLine(IN point3D_t* gpsLinePointMP, 
		                        IN point3D_t* gpsLinePointNP, 
								IN point3D_t* gpsPointPP, 
								OUT double* distInMeter)
	{
		// Calculate distance from point P to line MN

		if((gpsLinePointNP->lon == gpsLinePointMP->lon) && (gpsLinePointNP->lat == gpsLinePointMP->lat))
		// M and N are same point
		{
			calcRelDistance(gpsLinePointMP, gpsPointPP, distInMeter);
		}
		else
		{		
			double distMP, distNP, distMN;

            // Change coordinate
		    double x1 = 0;
		    double y1 = 0;
		    double x2 = gpsLinePointNP->lon - gpsLinePointMP->lon;
		    double y2 = gpsLinePointNP->lat - gpsLinePointMP->lat;

		    double x0 = gpsPointPP->lon - gpsLinePointMP->lon;
		    double y0 = gpsPointPP->lat - gpsLinePointMP->lat;

            distMP = sqrt(x0 * x0 + y0 * y0);
            distNP = sqrt((x2-x0)*(x2-x0) + (y2-y0)*(y2-y0));
            distMN = sqrt(x2 * x2 + y2 * y2);

			if(distMP >= distNP)
			{
				if((distMP * distMP) > (distMN * distMN + distNP * distNP))
				{
					*distInMeter = distNP;
				}else
				{
					// Change line to A*x_P + B*y_P + C = 0
					double A = y1 - y2;
					double B = x2 - x1;
					double C = -B * y1 - A * x1;
			
					*distInMeter = abs(A * x0 + B * y0 + C) / sqrt(A * A + B * B);
				}
			}else
			{
				if((distNP * distNP) > (distMN * distMN + distMP * distMP))
				{
					*distInMeter = distMP;
				}else
				{
					// Change line to A*x_P + B*y_P + C = 0
					double A = y1 - y2;
					double B = x2 - x1;
					double C = -B * y1 - A * x1;
			
					*distInMeter = abs(A * x0 + B * y0 + C) / sqrt(A * A + B * B);
				}
			}
		}
	}

	void calcProjectPointOnLine(IN point3D_t* gpsLinePointMP, 
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
            int calculated = 0;

			double distMP, distNP, distMN, distLineP;
			calcRelDistance(gpsLinePointMP, gpsPointPP, &distMP);
			calcRelDistance(gpsLinePointNP, gpsPointPP, &distNP);
            calcRelDistance(gpsLinePointMP, gpsLinePointNP, &distMN);

            if(distMP >= distNP)
            {
			    if((distMP * distMP) > (distMN * distMN + distNP * distNP))
			    {
				    projectPointTmp = *gpsLinePointNP;
                    
                    calculated = 1;
			    }
		    }else
		    {
			    if((distNP * distNP) > (distMN * distMN + distMP * distMP))
			    {
				    projectPointTmp = *gpsLinePointMP;

                    calculated = 1;
			    }
            }

            if (calculated == 0)
            {
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
            }

			*projectPointP = projectPointTmp;
		}

		projectPointP->alt = 0.0;
	}

	void calcRelBackOnLine(IN pointRelative3D_t* gpsLinePointMP, 
						   IN pointRelative3D_t* gpsLinePointNP, 
						   IN double     backDist, 
						   OUT pointRelative3D_t* gpsOutP)
	{
		// Calculate the GPS point on line MN, and a certain distance from N
		double xM = gpsLinePointMP->x;
		double yM = gpsLinePointMP->y;
		double xN = gpsLinePointNP->x;
		double yN = gpsLinePointNP->y;

        double distMN;

        calcRelDistance(gpsLinePointMP, gpsLinePointNP, &distMN);

        double distNP = backDist;

		gpsOutP->y = (yM - yN) * distNP / distMN + yN;
		gpsOutP->x = (xM - xN) * distNP / distMN + xN;
		gpsOutP->z = 0;
	}

    void calcRelBackOnLine(IN point3D_t* gpsLinePointMP, 
						   IN point3D_t* gpsLinePointNP, 
						   IN double     backDist, 
						   OUT point3D_t* gpsOutP)
	{
		// Calculate the GPS point on line MN, and a certain distance from N
		double xM = gpsLinePointMP->lon;
		double yM = gpsLinePointMP->lat;
		double xN = gpsLinePointNP->lon;
		double yN = gpsLinePointNP->lat;

        double distMN;

        calcRelDistance(gpsLinePointMP, gpsLinePointNP, &distMN);

        double distNP = backDist;

		gpsOutP->lat = (yM - yN) * distNP / distMN + yN;
		gpsOutP->lon = (xM - xN) * distNP / distMN + xN;
		gpsOutP->alt = 0;
	}

    void calcGpsRoadSide(IN point3D_t* gpsPre, 
                         IN point3D_t* gpsCurr, 
                         IN point3D_t* gpsRef,
                         IN uint8 side, 
                         IN float widthInMeter, 
                         OUT point3D_t* gpsRight) // gpsRight is relative position
    {
        pointRelative3D_t relLocPre, relLocCur;

        calcRelativeLocation(gpsRef, gpsPre, &relLocPre);
        calcRelativeLocation(gpsRef, gpsCurr, &relLocCur);

        double distX = relLocCur.x - relLocPre.x;
        double distY = relLocCur.y - relLocPre.y;

        double slopeN   = widthInMeter;
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
            gpsRight->lon = offset_x + relLocCur.x;
        }
        else
        {
            gpsRight->lon = -offset_x + relLocCur.x;
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
            gpsRight->lat = offset_y + relLocCur.y;
        }
        else
        {
            gpsRight->lat = -offset_y + relLocCur.y;
        }

        gpsRight->alt = gpsCurr->alt;
    }

    bool checkGpsInRange(IN point3D_t* gpsA, IN point3D_t* gpsB, IN double distThreshMeter)
    {
        // Distances in meters
        double dist;
        calcGpsDistance(gpsA, gpsB, &dist);

        if(dist > distThreshMeter)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool checkRelGpsInRange(IN point3D_t* gpsA, IN point3D_t* gpsB, IN double distThreshMeter)
    {
        // Distances in meters
        double dist;
        calcRelDistance(gpsA, gpsB, &dist);

        if(dist > distThreshMeter)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool checkAngleInRange(IN float angleA, IN float angleB, IN double angleThresh)
    {
        double angleDiff;

        if (angleA > angleB)
        {
            angleDiff = angleA - angleB;
        }else
        {
            angleDiff = angleB - angleA;
        }

        if (angleDiff > PI)
        {
            angleDiff = 2 * PI - angleDiff;
        }

        if (angleDiff > angleThresh)
        {
            return false;
        }else
        {
            return true;
        }
    }

    bool checkTwoFurnitureSame(IN furAttributes_t* fur1,
                               IN furAttributes_t* fur2,
                               IN double distThreshMeter,
                               IN float angleThresh)
    {
	    bool sameFlag = false;
	    bool usedFlag = fur1->location_used && fur1->sideFlag_used && fur1->type_used && fur1->angle_used
                     && fur2->location_used && fur2->sideFlag_used && fur2->type_used && fur2->angle_used;

	    if(usedFlag)
	    {
		    if((fur1->sideFlag == fur2->sideFlag) && (fur1->type == fur2->type) 
			    && checkRelGpsInRange(&(fur1->location),&(fur2->location),distThreshMeter)
			    && checkAngleInRange(fur1->angle, fur2->angle, angleThresh))
		    {
			    sameFlag = true;
		    }
	    }
	    return sameFlag;
    }
    
    bool checkTwoFurnitureSameNoRange(IN furAttributes_t* fur1,
                                      IN furAttributes_t* fur2, 
                                      IN float angleThresh)
    {
	    bool sameFlag = false;
	    bool usedFlag = fur1->sideFlag_used && fur1->type_used && fur1->angle_used
                     && fur2->sideFlag_used && fur2->type_used && fur2->angle_used;

	    if(usedFlag)
	    {
		    if((fur1->sideFlag == fur2->sideFlag) && (fur1->type == fur2->type) 
			    && checkAngleInRange(fur1->angle, fur2->angle, angleThresh))
		    {
			    sameFlag = true;
		    }
	    }
        //if((!sameFlag) && (fur1->type == fur2->type == 27453))
        //{
        //    cout << "compared type 1:" << fur1->type << "angle:" << fur1->angle <<"side:"<<(int)(fur1->sideFlag)<< endl;
        //    cout << "compared type 2:" << fur2->type << "angle:" << fur2->angle <<"side:"<<(int)(fur2->sideFlag)<< endl;
        //    cout << "usedFlag:" << (int)usedFlag<<endl;
        //    cout << "sameFlag:" << (int)sameFlag<<endl;
        //}
	    return sameFlag;
    }

	

	int roadSideGpsGen(IN  point3D_t gpsPre, 
						IN  point3D_t gps, 
						IN  double distanceInMeterL,
						IN  double distanceInMeterR,
						OUT point3D_t *gpsOutL,
						OUT point3D_t *gpsOutR)
	{
		int lineIndex = 0;
		point3D_t point1, point2;
		double rou, cosAlpha, sinAlpha;
		double lonDis, latDis;
		double latitude, coeffDd2MeterLon;

		point1 = gpsPre;
		point2 = gps;

		latitude = (point1.lat)*PI/180;
		coeffDd2MeterLon = (111413*cos(latitude)-94*cos(3*latitude));

		lonDis = (point1.lon - point2.lon) * coeffDd2MeterLon;
		latDis = (point1.lat - point2.lat) * COEFF_DD2METER;

		if((0 == lonDis) && (0 == latDis))
		{
			return 0;
		}
    
		rou = sqrt((lonDis * lonDis + latDis * latDis));
    
		cosAlpha = latDis / rou;
    
		sinAlpha = lonDis / rou;
    
		gpsOutL->lat = point2.lat - (distanceInMeterL)*(sinAlpha) / COEFF_DD2METER;
		gpsOutL->lon = point2.lon + (distanceInMeterL)*(cosAlpha) / coeffDd2MeterLon;
		gpsOutL->alt = 0;

		gpsOutR->lat = point2.lat + (distanceInMeterR)*(sinAlpha) / COEFF_DD2METER;
		gpsOutR->lon = point2.lon - (distanceInMeterR)*(cosAlpha) / coeffDd2MeterLon;
		gpsOutR->alt = 0;

		return 1;
	}

    bool lookAheadOnTrackPoint(IN vector<point3D_t> &locVec, 
                               IN int startLocIdx, 
                               IN double distanceAhead, 
                               OUT point3D_t &locAhead)
    {
        // Look ahead
        double distanceSum = 0;
        int pointIdx = startLocIdx;
        bool status = true;

        point3D_t locTempPre, locTempNext;

        if(distanceAhead == 0)
        {
            if(pointIdx < locVec.size())
            {
                locAhead = locVec[pointIdx];
            }else
            {
                status = false;
            }
        }else
        {
            while(distanceSum < distanceAhead)
            {
                if((pointIdx+1) >= locVec.size())
                {
                    // pointIdx+1 is out of the input vector
                    status = false;
                    break;
                }

                locTempPre = locVec[pointIdx];
                locTempNext = locVec[pointIdx+1];

                double dist;
                calcGpsDistance(&locTempPre, &locTempNext, &dist);

                ++pointIdx;
                distanceSum += dist;
            }

            if (status)
            {
                // Back
                double distBack = distanceSum - distanceAhead;

                pointRelative3D_t relLocPre, relLocNext, relLocAhead;

                calcRelativeLocation(&locTempPre, &locTempPre, &relLocPre);
                calcRelativeLocation(&locTempPre, &locTempNext, &relLocNext);

                calcRelBackOnLine(&relLocPre, &relLocNext, distBack, &relLocAhead);

                calcGpsFromRelativeLocation(&locTempPre, &relLocAhead, &locAhead);
            }
        }

        return status;
    }

    void lookAheadOnTrack(IN vector<point3D_t> &locVec, 
                          IN double distanceAhead, 
                          OUT vector<point3D_t> &locVecAhead)
    {
        locVecAhead.clear();
        point3D_t locAhead;
        int locIdx = 0;

        while(lookAheadOnTrackPoint(locVec, locIdx, distanceAhead, locAhead))
        {
            locVecAhead.push_back(locAhead);
            ++locIdx;
        }
    }
    void smoothGps(IN int filterOrder,
                   IN vector<point3D_t> &locVec, 
                   OUT vector<point3D_t> &locSmoothed)
    {
        int   step = (int)(filterOrder/2);
        for(int pointIdx = step; pointIdx < locVec.size() - step; pointIdx++)
        {
            double sumX = 0;
            double sumY = 0;
           
            for(int kk = 0; kk < filterOrder; kk++)
            {
                sumX += locVec[pointIdx - step + kk].lat;
                sumY += locVec[pointIdx - step + kk].lon;
            }
            point3D_t tempGps;
            tempGps.lat = sumX/7;
            tempGps.lon = sumY/7;
            tempGps.alt = locVec[pointIdx].alt;
            locSmoothed.push_back(tempGps);
        }

    }
}

