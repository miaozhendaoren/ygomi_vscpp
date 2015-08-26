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

#ifndef __VISUALIZATION_APIS_H
#define __VISUALIZATION_APIS_H

#include "apiDataStruct.h"

void readReportData(char *filename, list<list<vector<point3D_t>>> &newData);


void showImage(list<vector<point3D_t>> dataInput, Scalar scalar, string winname);

void saveListVec(list<vector<point3D_t>> &dataInput, char *filename);

#endif
