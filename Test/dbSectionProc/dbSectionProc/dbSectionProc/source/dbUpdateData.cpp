/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  dbUpdateData.cpp
*
* @brief Function implementation for background and foreground database data
*        updating.
*
* Change Log:
*      Date                Who             What
*    2015/08/18          Ming Chen         Create
*******************************************************************************
*/

#include "dbUpdateData.h"
#include "polynomialFit.h"

const uint32 RESAMPLE_SEG_ID[] = {1, 8, 9, 10, 11, 12, 20, 21};

bool isResampleSeg(uint32 segId)
{
    int size = sizeof(RESAMPLE_SEG_ID) / sizeof(RESAMPLE_SEG_ID[0]);
    for (int i = 0; i < size; i++)
    {
        if (segId == RESAMPLE_SEG_ID[i])
        {
            return true;
            break;
        }
    }

    return false;
}

void interpolationSample(IN    vector<point3D_t>     sourceLine,
                         INOUT vector<point3D_t>    &sampledLine)
{

}


void calcRotationAngle(IN  segAttributes_t           sectionConfig,
                       OUT double                   &theta,
                       OUT vector<double>           &xLimitation)
{

}


void lineRotation(IN  vector<point3D_t>              sourceLine,
                  IN  double                         theta,
                  OUT vector<point3D_t>             &rotatedLine)
{

}


void matchLaneType(IN  list<vector<point3D_t>>       sourceLane,
                   OUT uint32                       &laneNumber)
{

}


bool mergeSectionLane(IN    segAttributes_t          sectionConfig,
                      IN    reportSectionData        reportData,
                      INOUT backgroundSectionData   &bgDatabaseData)
{
    if (sectionConfig.segId != reportData.sectionId ||
        sectionConfig.segId != bgDatabaseData.sectionId)
    {
        return false;
    }

    // calculate current section rotation angle
    double theta = 0;
    vector<double> xlimit;
    calcRotationAngle(sectionConfig, theta, xlimit);

    // adjust distribution points of x direction according to x limitation
    vector<point3D_t> sampledLine;
    {
    }


    // match lane type of reported new data, section by section, lane by lane,
    // line by line
    // multiple reported
    list<list<list<vector<point3D_t>>>>::iterator segItor = reportData.rptSecData.begin();
    while (segItor != reportData.rptSecData.end())
    {
        // lane
        list<list<vector<point3D_t>>>::iterator laneItor = segItor->begin();

        // matched lane type
        uint32 matchedLane = 1;
        matchLaneType(*laneItor, matchedLane);

        //

        while (laneItor != segItor->end())
        {
            // line
            list<vector<point3D_t>>::iterator lineItor = laneItor->begin();

            // rotate line
            vector<point3D_t> rotatedLine;
            lineRotation(*lineItor, theta, rotatedLine);

            if (isResampleSeg(reportData.sectionId))
            {
                // re-sample for special sections
                interpolationSample(rotatedLine, sampledLine);
            }
            else
            {
                // polynomial fitting for other sections
                uint32 numOfPoints = rotatedLine.size();
                double *dx = new double[numOfPoints];
                double *dy = new double[numOfPoints];
                if (dx && dy)
                {
                    for (uint32 ii = 0; ii < numOfPoints; ii++)
                    {
                        dx[ii] = rotatedLine[ii].lat;
                        dy[ii] = rotatedLine[ii].lon;
                    }

                    double coefficient[MAX_DEGREE + 2];
                    for (int ii = 0; ii < MAX_DEGREE + 2; ii++)
                    {
                        coefficient[ii] = 0;
                    }

                    Point2d normalization;
                    parametersNormalized(dx, numOfPoints, normalization);
                    double mse = EMatrix(dx, dy, numOfPoints, DEFAULT_DEGREE, normalization, coefficient);

                    vector<point3D_t>::iterator pointItor = sampledLine.begin();
                    while (pointItor != sampledLine.end())
                    {
                        pointItor->lat = calValue(pointItor->lon, coefficient, normalization);
                    }
                }
            }

            // check lane data in database
            if (bgDatabaseData.bgSectionData.empty())
            {
            }


            laneItor++;
        }
        segItor++;
    }

    return true;
}


bool stitchSectionLanes(IN  segAttributes_t          sectionConfig,
                        IN  backgroundSectionData    bgDatabaseData,
                        OUT foregroundSectionData   &fgSectionData)
{

    return true;
}


void getSegAndDbData(IN  uint32                         sectionId,
                     IN  list<segAttributes_t>          sectionConfig,
                     IN  vector<backgroundSectionData>  bgDatabase,
                     OUT segAttributes_t               &configSegData,
                     OUT backgroundSectionData         &bgSegData)
{
    if (sectionConfig.empty() || bgDatabase.empty())
    {
        return;
    }

    // zero previous memory
    memset(&configSegData, sizeof(segAttributes_t), 0);

    if (!bgSegData.bgSectionData.empty())
    {
        list<list<vector<point3D_t>>>::iterator laneItor = bgSegData.bgSectionData.begin();
        while (laneItor != bgSegData.bgSectionData.end())
        {
            list<vector<point3D_t>>::iterator lineItor = laneItor->begin();
            while(lineItor != laneItor->end())
            {
                lineItor->clear();
                lineItor++;
            }
            laneItor->clear();
            laneItor++;
        }
    }

    // get section configuration
    list<segAttributes_t>::iterator configItor = sectionConfig.begin();
    while (configItor != sectionConfig.end())
    {
        if (sectionId == configItor->segId)
        {
            configSegData = *configItor;
            break;
        }
    }

    // get background database data
    vector<backgroundSectionData>::iterator dbItor = bgDatabase.begin();
    while (dbItor != bgDatabase.end())
    {
        if (sectionId == dbItor->sectionId)
        {
            bgSegData = *dbItor;
            break;
        }
        dbItor++;
    }

    return;
}
