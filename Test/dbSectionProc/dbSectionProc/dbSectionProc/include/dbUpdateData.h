/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  dbUpdateData.h
* @brief Function headers definition for background and foreground database data
*        updating.
*
* Change Log:
*      Date                Who             What
*      2015/08/17       Ming Chen         Create
*******************************************************************************
*/

#ifndef __DB_UPDATE_DATA_H
#define __DB_UPDATE_DATA_H

#include "apiDataStruct.h"

/*
 * @FUNC
 *     Re-sample input line data by using interpolation.
 *
 * @PARAMS
 *     soureLine   - source line data of x, y.
 *     sampledLine - sample data of input line, x is used to calculate
 *                   corresponding y values.
 *
 */
void interpolationSample(IN    vector<point3D_t>  sourceLine,
                         INOUT vector<point3D_t> &sampledLine);

/*
 * @FUNC
 *     Calculate line rotation angle according to section full configuration.
 *     The returned angle is the line from start points to end points.
 *
 * @PARAMS
 *     sectionConfig - section configuration data, including all sections ID
 *                     and section full parameters.
 *     sectionID     - current section to calculate rotation angle.
 *     theta         - returned rotation angel, 0 ~ pi.
 *     xLimitation   - x min/max limitation.
 *                     4 values, min max min max
 *
 */

void calcRotationAngle(IN  segAttributes_t        sectionConfig,
                       OUT double                &theta,
                       OUT vector<double>        &xLimitation);

/*
 * @FUNC
 *     Rotate input line according to rotation angle.
 *
 * @PARAMS
 *     sourceLine  - source line data of x, y.
 *     theta       - rotation angle.
 *     rotatedLine - returned rotated line.
 *
 */
void lineRotation(IN  vector<point3D_t>  sourceLine,
                  IN  double             theta,
                  OUT vector<point3D_t> &rotatedLine);

/*
 * @FUNC
 *     Match input lane to correct lane number, 1, 2, 3 ... from left to right.
 *     For example, if line type of input lane is solid-dash then matched lane
 *     number is 1, or line type is dash-dash, then lane number is 2, or line
 *     type is dash-solid, lane number is 3. Currently, only three lanes is
 *     used.
 *
 * @PARAM
 *     sourceLane - source lane data, including two lines. First line is the
 *                  the left one, and second line is the right one.
 *     laneNumber - matched lane number, 1/2/3/...
 *
 */
void matchLaneType(IN  list<vector<point3D_t>>  sourceLane,
                   OUT uint32                  &laneNumber);

/*
 * @func
 *     Background database update, update each lane separately.
 *
 * @params
 *     sectionConfig  - section configuration data, including section ID
 *                      and section full parameters.
 *     reportData     - reported data from vehicle, including lines of
 *                      the lane vehicle running.
 *     bgDatabaseData - background database data of current section,
 *                      stored lane by lane.
 *
 */
bool mergeSectionLane(IN    segAttributes_t          sectionConfig,
                      IN    reportSectionData        reportData,
                      INOUT backgroundSectionData   &bgDatabaseData);

/*
 * @func
 *     Foreground database update. Merge common line of two adjacent lanes.
 *
 * @params
 *     sectionConfig  - section configuration data, including section ID
 *                      and section full parameters.
 *     bgDatabaseData - background database data of current section,
 *                      stored lane by lane.
 *     fgSectionData  - foreground lines of road in current section.
 *
 */
bool stitchSectionLanes(IN  segAttributes_t          sectionConfig,
                        IN  backgroundSectionData    bgDatabaseData,
                        OUT foregroundSectionData   &fgSectionData);

/*
 * @FUNC
 *     Get corresponding section configuration data and background database
 *     data according to input section ID.
 *
 * @PARAMS
 *     sectionId     - section configuration ID.
 *     sectionConfig - all section configuration attributes list.
 *     bgDatabase    - background database of all sections.
 *     configSegData - output extracted section configuration data from
 *                     segAttributes list.
 *     bgSegData     - output extracted background data of input section
 *
 */

void getSegAndDbData(IN  uint32                         sectionId,
                     IN  list<segAttributes_t>          sectionConfig,
                     IN  vector<backgroundSectionData>  bgDatabase,
                     OUT segAttributes_t               &configSegData,
                     OUT backgroundSectionData         &bgSegData);

#endif
