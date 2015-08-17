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

	void calcDistance(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT double* dist)
	{
        double distX = gpsPre->lon - gpsCurr->lon;
        double distY = gpsPre->lat - gpsCurr->lat;

		*dist = sqrt(distX * distX + distY * distY) * COEFF_DD2METER;
	}

	void calcDistancePointToLine(IN point3D_t* gpsLinePointMP, 
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
			double distMP, distNP, distMN;

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
			calcDistance(gpsLinePointMP, gpsPointPP, &distMP);
			calcDistance(gpsLinePointNP, gpsPointPP, &distNP);
            calcDistance(gpsLinePointMP, gpsLinePointNP, &distMN);

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

	void calcGpsBackOnLine(IN point3D_t* gpsLinePointMP, 
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

    void calcGpsRoadSide(IN point3D_t* gpsPre, 
                         IN point3D_t* gpsCurr, 
                         IN uint8 side, 
                         IN float width, 
                         OUT point3D_t* gpsRight)
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

    bool checkGpsInRange(IN point3D_t* gpsA, IN point3D_t* gpsB, IN double distThresh)
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
                               IN double distThresh,
                               IN float angleThresh)
    {
	    bool sameFlag = false;
	    bool usedFlag = fur1->location_used && fur1->sideFlag_used && fur1->type_used && fur1->angle_used
                     && fur2->location_used && fur2->sideFlag_used && fur2->type_used && fur2->angle_used;

	    if(usedFlag)
	    {
		    if((fur1->sideFlag == fur2->sideFlag) && (fur1->type == fur2->type) 
			    && checkGpsInRange(&(fur1->location),&(fur2->location),distThresh)
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
}
