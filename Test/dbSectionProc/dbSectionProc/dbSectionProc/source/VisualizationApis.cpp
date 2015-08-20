/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  VisualizationApis.h
* @brief Visualization APIs implementation file. Used to show results of steps.
*
* Change Log:
*      Date                Who             What
*      2015/08/20       Chen Ming        Create
*******************************************************************************
*/

#include <float.h>
#include "VisualizationApis.h"

void readReportData(char *filenamelist, list<list<vector<point3D_t>>> &newData)
{
    if (nullptr == filenamelist)
    {
        printf("filename is not correct\n");
        return;
    }

    // release previous data
    if (!newData.empty())
    {
        list<list<vector<point3D_t>>>::iterator grpIt = newData.begin();
        while (grpIt != newData.end())
        {
            list<vector<point3D_t>>::iterator it = grpIt->begin();
            while (it != grpIt->end())
            {
                it->clear();
                it++;
            }
            grpIt->clear();
            grpIt++;
        }
        newData.clear();
    }

    FILE *fp = nullptr;
    errno_t err = fopen_s(&fp, filenamelist, "rt");
    if (0 != err)
    {
        printf("failed to open file %s\n", filename);
        return;
    }

    double tempData[19];
    point3D_t currentPoint = { 0 };
    vector<point3D_t> leftLine;
    vector<point3D_t> rightLine;

    // initialize tempData
    memset(tempData, 0, 19);

    while (!feof(fp))
    {
        // scan row data
        fscanf_s(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \
                     %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                     &tempData[ 0], &tempData[ 1], &tempData[ 2], &tempData[ 3],
                     &tempData[ 4], &tempData[ 5], &tempData[ 6], &tempData[ 7],
                     &tempData[ 8], &tempData[ 9], &tempData[10], &tempData[11],
                     &tempData[12], &tempData[13], &tempData[14], &tempData[15],
                     &tempData[16], &tempData[17], &tempData[18]);

        // left line point
        currentPoint.lat       = tempData[0];
        currentPoint.lon       = tempData[1];
        currentPoint.paintFlag = (float)tempData[2];

        leftLine.push_back(currentPoint);

        // right line point
        currentPoint.lat       = tempData[11];
        currentPoint.lon       = tempData[12];
        currentPoint.paintFlag = (float)tempData[13];

        rightLine.push_back(currentPoint);
    }

    // add lane data
    newData.push_back(leftLine);
    newData.push_back(rightLine);
}


void showImage(list<vector<point3D_t>> dataInput, Scalar scalar, string winname)
{
    int numOfLines = dataInput.size();
    if (0 == numOfLines)
    {
        return;
    }

    // get the minimum and maximum values of X and Y
    double minX = DBL_MAX;
    double maxX = DBL_MIN;
    double minY = DBL_MAX;
    double maxY = DBL_MIN;

    // iterate each line to get min/max range
    int numOfPoints = 0;
    list<vector<point3D_t>>::iterator lineItor = dataInput.begin();
    while (lineItor != dataInput.end())
    {
        numOfPoints = lineItor->size();
        for (int i = 0;i < numOfPoints; i++)
        {
            // only check painted points
            if (-1.0 != lineItor->at(i).paintFlag)
            {
                if (lineItor->at(i).lon <= minX)
                {
                    minX = lineItor->at(i).lon;
                }
                if (lineItor->at(i).lon > maxX)
                {
                    maxX = lineItor->at(i).lon;
                }
                if (lineItor->at(i).lat <= minY)
                {
                    minY = lineItor->at(i).lat;
                }
                if (lineItor->at(i).lat > maxY)
                {
                    maxY = lineItor->at(i).lat;
                }
            }
        }
        lineItor++;
    }

    // image width and height
    int width  = (int)(maxX - minX);
    int height = (int)(maxY - minY);
    Mat outputImage(height, width, CV_8UC3, Scalar(255, 255, 255));

    // iterate each line again to paint line points
    Point curr;
    lineItor = dataInput.begin();
    while (lineItor != dataInput.end())
    {
        numOfPoints = lineItor->size();
        for (int i = 0; i < numOfPoints; i++)
        {
            // flip Y axis as OpenCV starts from top left to right bottom
            curr.x = (int)(lineItor->at(i).lon - minX);
            curr.y = (int)(height - (lineItor->at(i).lat - minY));
            circle(outputImage, curr, 0, scalar, 1);
        }

        lineItor++;
    }

    imwrite(winname, outputImage);
}
