/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  RoadVecGen.cpp
* @brief This is class implementation file for RoadVecGen, which gets new
*        data from vehicle and merges them with database data to update road
*        lane information.
*
* Change Log:
*      Date                Who             What
*      2015/08/19       Ming Chen         Create
*******************************************************************************
*/

#include <complex>

#include "apiDataStruct.h"
#include "polynomialFit.h"
#include "RoadVecGen.h"

#if 1 // VISUALIZATION_ON || DATA_SAVE_ON
#include "VisualizationApis.h"

static uint32 PREVIOUS_SEGID = 0;
static uint32 MERGED_TIMES   = 0;
static uint32 FG_MERGED_NUM  = 0;

char IMAGE_NAME_STR[MAX_PATH] = { '\0' };
#endif

using namespace std;

namespace ns_database
{
#define DASH_CONT_POINTS_TH 10
#define MINDIST             10
#define LINE_TYPE_TH        50
#define SOLID_OR_DASH_TH    0.5
#define SAMPLE_SPACE        0.1
#define SINGLE_LANE         1
#define DOUBLE_LANE         2
#define MAX_SUPPORTED_LANES 3
#define INVALID_LANE_FLAG   -1
#define VALID_LANE_FLAG     1
#define EACH_LANE_LINES_NUM 2
#define SOLID_DASH          0
#define DASH_DASH           1
#define DASH_SOLID          2
#define SOLID_SOLID         0
#define DIST_DIFF           2.5

    // add a control to use interpolation or polynomial fitting
#define USE_INTERPOLATION   1

const uint32 RESAMPLE_SEC_ID[] = {1, 8, 9, 10, 11, 12, 20, 21};

const double LINE_TYPE_THRESHOLD[] = {
    50,
    50.0, 80.0, //  1,  2
    50.0, 50.0, //  3,  4
    30.0, 50.0, //  5,  6
    50.0, 50.0, //  7,  8
    50.0, 50.0, //  9, 10
    50.0, 80.0, // 11, 12
    50.0, 50.0, // 13, 14
    50.0, 50.0, // 15, 16
    50.0, 50.0, // 17, 18
    50.0, 50.0, // 19, 20
    50.0, 50.0, // 21, 22
    50.0, 50.0, // 23, 24
    50.0        // 25
};

bool isResampleSec(uint32 segId)
{
    int size = sizeof(RESAMPLE_SEC_ID) / sizeof(RESAMPLE_SEC_ID[0]);
    for (int i = 0; i < size; i++)
    {
        if (segId == RESAMPLE_SEC_ID[i])
        {
            return true;
        }
    }

    return false;
}


    CRoadVecGen::CRoadVecGen()
    {
        _configPath = "";
    }


    CRoadVecGen::CRoadVecGen(string configFilePath)
    {
        _configPath = configFilePath;

        // read section configuration file
        list<segAttributes_t> segConfigList;
        readSecConfig(segConfigList);

        // initialize database
        initDatabase();
    }


    CRoadVecGen::~CRoadVecGen()
    {
        _configPath = "";

        // release list or vector data
        _segConfigList.clear();

        list<backgroundSectionData>::iterator secItor = _bgDatabaseList.begin();
        while (secItor != _bgDatabaseList.end())
        {
            list<list<vector<point3D_t>>>::iterator laneItor = secItor->bgSectionData.begin();
            while (laneItor != secItor->bgSectionData.end())
            {
                list<vector<point3D_t>>::iterator lineItor = laneItor->begin();
                while (lineItor != laneItor->end())
                {
                    lineItor->clear();
                    lineItor++;
                }

                laneItor->clear();
                laneItor++;
            }

            secItor->bgSectionData.clear();
            secItor++;
        }
    }


    bool CRoadVecGen::roadSectionsGen(IN  list<list<vector<point3D_t>>> &rptData,
                                      OUT list<list<vector<point3D_t>>> &fgData)
    {
        if (rptData.empty())
        {
            return false;
        }

        // release data first
        if (!fgData.empty())
        {
            list<list<vector<point3D_t>>>::iterator secItor = fgData.begin();
            while (secItor != fgData.end())
            {
                list<vector<point3D_t>>::iterator lineItor = secItor->begin();
                while (lineItor != secItor->end())
                {
                    lineItor->clear();
                    lineItor++;
                }
                secItor++;
            }
        }

        // suppose output is list<reportSectionData> rptData;
        list<reportSectionData> secData;

        // section partition, input is rptData
        _extractSecObj.extractSections(_segConfigList, _stSecConfig, rptData, secData);

#if SAVE_DATA_ON
        int groupNum = 0, laneNum = 0;
        list<reportSectionData>::iterator fsec = secData.begin();
        while (fsec != secData.end())
        {
            if (!fsec->rptSecData.empty())
            {
                groupNum = 0;
                list<list<list<vector<point3D_t>>>>::iterator grp = fsec->rptSecData.begin();
                while (grp != fsec->rptSecData.end())
                {
                    laneNum = 0;
                    list<list<vector<point3D_t>>>::iterator lane = grp->begin();
                    while (lane != grp->end())
                    {
                        sprintf_s(IMAGE_NAME_STR, MAX_PATH - 1, "fg_%d_sec_%d_group_%d_lane_%d.txt",
                            FG_MERGED_NUM, fsec->sectionId, groupNum, laneNum);
                        saveListVec(*lane, IMAGE_NAME_STR);

                        lane++; laneNum++;
                    }

                    grp++; groupNum++;
                }
            }

            fsec++;
        }
#endif

        list<reportSectionData>::size_type numOfRptSecs = secData.size();

        segAttributes_t       configSecData;
        backgroundSectionData *bgSecData = nullptr;

        // iterate each section
        list<reportSectionData>::iterator secItor = secData.begin();
        while (secItor != secData.end())
        {
            // current reported section ID
            uint32 segId = secItor->sectionId;

            // get section configuration and database data
            getSegAndDbData(segId, configSecData, &bgSecData);

            // merge new data with database
            mergeSectionLane(configSecData, *secItor, bgSecData);

            secItor++;
        }

        // stitch background database to generate foreground data
        stitchSectionLanes();

        if (!_fgDatabaseList.empty())
        {
            list<foregroundSectionData>::size_type numOfFgSecs = _fgDatabaseList.size();
            list<foregroundSectionData>::iterator secItor = _fgDatabaseList.begin();
            for (uint32 ii = 0; ii < numOfFgSecs; ii++)
            {
                fgData.push_back(secItor->fgSectionData);
                secItor++;
            }
        }

#if 1 //VISUALIZATION_ON || SAVE_DATA_ON
        list<vector<point3D_t>> fglines;
        list<list<vector<point3D_t>>>::iterator fgItor = fgData.begin();
        list<vector<point3D_t>>::iterator fglineItor;
        while (fgItor != fgData.end())
        {
            if (!fgItor->empty())
            {
                fglineItor = fgItor->begin();
                while (fglineItor != fgItor->end())
                {
                    fglines.push_back(*fglineItor);

                    fglineItor++;
                }
            }

            fgItor++;
        }
        sprintf_s(IMAGE_NAME_STR, "fgdatabase_%d.png", FG_MERGED_NUM++);
#if 1 // VISUALIZATION_ON
        showImage(fglines, Scalar(0, 0, 255), IMAGE_NAME_STR);
#endif
#if 1 // SAVE_DATA_ON
        saveListVec(fglines, "fgroup_0.txt");
#endif
#endif

        return true;
    }


    void CRoadVecGen::setSectionConfigPath(IN string filename, OUT list<segAttributes_t> &segConfigList)
    {
        _configPath = filename;

        // read section configuration file
        readSecConfig(segConfigList);

        // initialize database
        initDatabase();
    }


    void CRoadVecGen::initDatabase()
    {
        // when initialing database should be empty, so add structure directly
        list<segAttributes_t>::iterator configItor = _segConfigList.begin();
        while (configItor != _segConfigList.end())
        {
            backgroundSectionData bgData;
            bgData.sectionId = configItor->segId;
            _bgDatabaseList.push_back(bgData);

            foregroundSectionData fgData;
            fgData.sectionId = configItor->segId;
            _fgDatabaseList.push_back(fgData);

            configItor++;
        }
    }


    void CRoadVecGen::readSecConfig(OUT list<segAttributes_t> &segCfgList)
    {
        int sectionNum = 0;
        int sectionID  = 0;
        int laneNum = 0;

        segAttributes_t segmentElement;
        memset(&segmentElement, 0, sizeof(segAttributes_t));

        FILE *fp = nullptr;
        errno_t err = fopen_s(&fp, _configPath.c_str(), "rt");
        if(0 != err)
        {
            return;
        }

        // number of sections
        fscanf_s(fp, "%d\n", &sectionNum);

        // width, length, overlap, paint and step size information
        fscanf_s(fp, "width=%lf,overlap=%lf,minLength=%lf,maxLength=%lf,\
                      stepSize=%d,paintV=%lf\n",
                      &(_stSecConfig.dbWidth), &(_stSecConfig.dbOverlap),
                      &(_stSecConfig.dbMinLength), &(_stSecConfig.dbMaxLength),
                      &(_stSecConfig.uiStepSize), &(_stSecConfig.dbPaintV));
        while (!feof(fp))
        {
            fscanf_s(fp, "%d,%d\n", &sectionID, &laneNum);
            segmentElement.segId_used = 1;
            segmentElement.segId      = sectionID;
            segmentElement.uiLaneNum_used = 1;
            segmentElement.uiLaneNum      = laneNum;
            fscanf_s(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
                &segmentElement.ports[0].lon, &segmentElement.ports[0].lat,
                &segmentElement.ports[1].lon, &segmentElement.ports[1].lat,
                &segmentElement.ports[2].lon, &segmentElement.ports[2].lat,
                &segmentElement.ports[3].lon, &segmentElement.ports[3].lat,
                &segmentElement.ports[4].lon, &segmentElement.ports[4].lat,
                &segmentElement.ports[5].lon, &segmentElement.ports[5].lat,
                &segmentElement.ports[6].lon, &segmentElement.ports[6].lat,
                &segmentElement.ports[7].lon, &segmentElement.ports[7].lat);

            _segConfigList.push_back(segmentElement);
        }

        fclose(fp);

        segCfgList = _segConfigList;
    }


    void CRoadVecGen::interpolationSample(IN    vector<point3D_t> &sourceLine,
                                          INOUT vector<point3D_t> &sampledLine)
    {
        if(sourceLine.empty() || sampledLine.empty())
        {
            printf("ERROR: interpolationSample : Input empty. \n");
            return;
        }

        uint32 srcPointCnt = sourceLine.size();
        uint32 splPointCnt = sampledLine.size();

        if(1 == srcPointCnt)
        {
            // only one dimension
            for(uint32 index = 0; index < splPointCnt; index++)
            {
                sampledLine[index].lat = sourceLine[0].lat;
            }
        }
        else
        {
            // for two dimension

            bool findInterval = false;
            int x0 = -1, x1 = -1;

            for(uint32 index = 0; index < splPointCnt; index++)
            {
                findInterval = false;
                x0 = -1;
                x1 = -1;

                for(uint32 i = 0; i < srcPointCnt; i++)
                {
                    // find exactly matched value in source line
                    if(sampledLine[index].lon == sourceLine[i].lon)
                    {
                        x0 = i;
                        x1 = i;

                        findInterval = true;
                        break;
                    }

                    // within an interval in source line, between the interval [i-1, i]
                    if((0 < i) && (0 > (sampledLine[index].lon - sourceLine[i-1].lon) *
                        (sampledLine[index].lon - sourceLine[i].lon)))
                    {
                        x0 = i - 1;
                        x1 = i;

                        findInterval = true;
                        break;
                    }
                }

                if(!findInterval)
                {
                    if(abs(sampledLine[index].lon - sourceLine[0].lon) <
                        abs(sampledLine[index].lon - sourceLine[srcPointCnt - 1].lon))
                    {
                        x0 = 0;
                        x1 = 0;
                    }
                    else
                    {
                        // more than 2 points, use last three points to generate
                        // points not in source line, otherwise use last two points
                        x0 = srcPointCnt - 3;
                        x1 = srcPointCnt - 2;


                        if(srcPointCnt <= 2)
                        {
                            x0 = srcPointCnt - 2;
                            x1 = srcPointCnt - 1;
                        }
                    }
                }

                if(x0 == x1)
                {
                    sampledLine[index].lat = sourceLine[x0].lat;
                }
                else
                {
                    sampledLine[index].lat = (sourceLine[x0].lon - sampledLine[index].lon) * sourceLine[x1].lat / (sourceLine[x0].lon - sourceLine[x1].lon) +
                        (sampledLine[index].lon - sourceLine[x1].lon) * sourceLine[x0].lat / (sourceLine[x0].lon - sourceLine[x1].lon);
                }
            }//end for sampledLine
        }// end if
    }


    void CRoadVecGen::calcRotationAngle(IN  segAttributes_t &sectionConfig,
                                        OUT double          &theta,
                                        OUT vector<double>  &xLimitation)
    {
        double X0[MAX_NUM_PORT];
        memset((uint32 *)X0, 0, sizeof(double) * MAX_NUM_PORT);

        double x0 = (sectionConfig.ports[0].lon + sectionConfig.ports[1].lon) / 2;
        double x1 = (sectionConfig.ports[2].lon + sectionConfig.ports[3].lon) / 2;

        double y0 = (sectionConfig.ports[0].lat + sectionConfig.ports[1].lat) / 2;
        double y1 = (sectionConfig.ports[2].lat + sectionConfig.ports[3].lat) / 2;

        theta = atan2((y0 - y1), (x0 - x1));

        complex<double> thetaj(0, -1 * theta);

        double X0Temp = 0;
        for (int i = 0; i < MAX_NUM_PORT - 2; i++)
        {
            X0Temp = 0;
            complex<double> yi(0, sectionConfig.ports[i].lat);
            X0Temp = real((sectionConfig.ports[i].lon + yi) * exp(thetaj));
            X0[i] = X0Temp;
        }

        if (X0[0] < X0[2])
        {
            xLimitation.push_back(ceil(min(X0[0], X0[1])));
            xLimitation.push_back(ceil(max(X0[2], X0[3])));
            xLimitation.push_back(ceil(min(X0[4], X0[5])));
            xLimitation.push_back(ceil(max(X0[6], X0[7])));
        }
        else
        {
            xLimitation.push_back(ceil(max(X0[0], X0[1])));
            xLimitation.push_back(ceil(min(X0[2], X0[3])));
            xLimitation.push_back(ceil(max(X0[4], X0[5])));
            xLimitation.push_back(ceil(min(X0[6], X0[7])));
        }
    }


    void CRoadVecGen::lineRotation(IN  vector<point3D_t> &sourceLine,
                                   IN  double             theta,
                                   OUT vector<point3D_t> &rotatedLine)
    {
        int vecSize = sourceLine.size();

        complex<double> thetaj(0, theta);
        double x = 0, y = 0, realZ = 0, imagZ = 0;
        point3D_t pointZ = { 0 };

        for (int i = 0; i < vecSize; i++)
        {
            y = sourceLine[i].lat;
            x = sourceLine[i].lon;

            complex<double> yj(0, y);
            realZ = real((x + yj) * exp(thetaj));
            imagZ = imag((x + yj) * exp(thetaj));

            pointZ.lon       = realZ;
            pointZ.lat       = imagZ;
            pointZ.alt       = 0;
            pointZ.paintFlag = (sourceLine[i].paintFlag >= 0) ? \
                                sourceLine[i].paintFlag : 0;
            pointZ.count     = 1;

            rotatedLine.push_back(pointZ);
        }
    }


    void CRoadVecGen::matchLaneType(IN  list<vector<point3D_t>> &sourceLane,
                                    IN  uint32                   segId,
                                    IN  uint32                   maxLaneNum,
                                    OUT uint32                  &laneNumber)
    {
        // check number of input lines
        if (2 != sourceLane.size())
        {
            laneNumber = 0;
            return;
        }

        // get left and right line of input lane
        int lanetype = SOLID_SOLID;
        double threshold = LINE_TYPE_THRESHOLD[segId];

        // left line of input lane
        double leftVal = getLineEstValue(sourceLane.front());

        // right line of input lane
        double rightVal = getLineEstValue(sourceLane.back());

        // judge matched lane number
        if (leftVal > threshold && rightVal < threshold)
        {
            lanetype = SOLID_DASH;
        }
        if (leftVal < threshold && rightVal < threshold)
        {
            lanetype = DASH_DASH;
        }
        if (leftVal < threshold && rightVal > threshold)
        {
            lanetype = DASH_SOLID;
        }

        if (SINGLE_LANE == maxLaneNum)
        {
            laneNumber = SOLID_SOLID;
        }
        if (DOUBLE_LANE == maxLaneNum)
        {
            laneNumber = (SOLID_DASH == lanetype) ? SOLID_DASH : DOUBLE_LANE - 1;
        }
        if (MAX_SUPPORTED_LANES == maxLaneNum)
        {
            laneNumber = lanetype;
        }
    }


    bool CRoadVecGen::mergeSectionLane(IN    segAttributes_t       &sectionConfig,
                                       IN    reportSectionData     &reportData,
                                       INOUT backgroundSectionData *bgDatabaseData)
    {
        // check section ID
        if (nullptr == bgDatabaseData ||
            sectionConfig.segId != reportData.sectionId ||
            sectionConfig.segId != bgDatabaseData->sectionId)
        {
            return false;
        }

        // calculate current section rotation angle
        double theta = 0;
        vector<double> xlimits;
        calcRotationAngle(sectionConfig, theta, xlimits);

        // re-sample spacing in x direction
        vector<point3D_t> leftSample;
        vector<point3D_t> rightSample;
        point3D_t pointSpl = { 0 };
        int pOrder = 1;
        int numOfSplPnts = (int)((xlimits[3] - xlimits[2]) / SAMPLE_SPACE);
        if (xlimits[2] >= xlimits[3])
        {
            pOrder = -1;
            numOfSplPnts = (int)((xlimits[2] - xlimits[3]) / SAMPLE_SPACE);
        }

        for (int i = 0; i < numOfSplPnts; i++)
        {
            pointSpl.lon = xlimits[2] + i * pOrder * SAMPLE_SPACE;

            leftSample.push_back(pointSpl);
            rightSample.push_back(pointSpl);
        }

        // number of lanes in current section
        uint32 numOfLanes = sectionConfig.uiLaneNum;

        // matched lane number
        uint32 matchedLane = 0;

        vector<point3D_t> leftRotated, leftRotatedValid;
        vector<point3D_t> rightRotated, rightRotatedValid;

        // number of data group in report data
        int numOfGroups = reportData.rptSecData.size();
        list<list<list<vector<point3D_t>>>>::iterator grpItor = reportData.rptSecData.begin();
        while (grpItor != reportData.rptSecData.end())
        {

#if VISUALIZATION_ON
            if (PREVIOUS_SEGID == sectionConfig.segId)
            {
                MERGED_TIMES++;
            }
            else
            {
                PREVIOUS_SEGID = sectionConfig.segId;
                MERGED_TIMES = 0;
            }
#endif

            // lane
            list<list<vector<point3D_t>>>::iterator laneItor = grpItor->begin();
            while (laneItor != grpItor->end())
            {
                // release local variables to prepare data storage
                leftRotated.clear();
                rightRotated.clear();
                leftRotatedValid.clear();
                rightRotatedValid.clear();

                // step 1 - Lane number identification
                matchedLane = 0;
                matchLaneType(*laneItor, sectionConfig.segId, numOfLanes, matchedLane);

                // step 2 - Process new data before merging
                // line - there should be two lines. rotate first
                lineRotation(laneItor->front(), -theta, leftRotated); // left line
                lineRotation(laneItor->back(), -theta, rightRotated); // right line

#if VISUALIZATION_ON
                list<vector<point3D_t>> rotated;
                rotated.push_back(leftRotated);
                rotated.push_back(rightRotated);
                sprintf_s(IMAGE_NAME_STR, "section_%d_lane_%d_merging_%d_rotated.png",
                          sectionConfig.segId, matchedLane, MERGED_TIMES);
                showImage(rotated, Scalar(0, 255, 0), IMAGE_NAME_STR);
#endif

                // extract valid points for re-sampling or polynomial fitting
                for (uint32 i = 0; i < leftRotated.size(); i++)
                {
                    if (0 < leftRotated[i].paintFlag)
                    {
                        leftRotatedValid.push_back(leftRotated[i]);
                    }
                }
                for (uint32 i = 0; i < rightRotated.size(); i++)
                {
                    if (0 < rightRotated[i].paintFlag)
                    {
                        rightRotatedValid.push_back(rightRotated[i]);
                    }
                }

                // get paint information for sample
                getLinePaintInfo(leftRotated, leftSample);
                getLinePaintInfo(rightRotated, rightSample);

                // re-sample or polynomial fitting
                if (USE_INTERPOLATION/*isResampleSec(sectionConfig.segId)*/)
                {
                    interpolationSample(leftRotatedValid,  leftSample);
                    interpolationSample(rightRotatedValid, rightSample);
#if VISUALIZATION_ON
                    list<vector<point3D_t>> intersampled;
                    intersampled.push_back(leftSample);
                    intersampled.push_back(rightSample);
                    sprintf_s(IMAGE_NAME_STR, "section_%d_lane_%d_merging_%d_interpolate_sample.png",
                              sectionConfig.segId, matchedLane, MERGED_TIMES);
                    showImage(intersampled, Scalar(0, 0, 255), IMAGE_NAME_STR);
#endif
                }
                else
                {
                    polynomialFitting(leftRotatedValid,  leftSample);
                    polynomialFitting(rightRotatedValid, rightSample);
#if VISUALIZATION_ON
                    list<vector<point3D_t>> polysampled;
                    polysampled.push_back(leftSample);
                    polysampled.push_back(rightSample);
                    sprintf_s(IMAGE_NAME_STR, "section_%d_lane_%d_merging_%d_polyval_sample.png",
                              sectionConfig.segId, matchedLane, MERGED_TIMES);
                    showImage(polysampled, Scalar(0, 0, 255), IMAGE_NAME_STR);
#endif
                }

                // step 3 - merge with database data
                // if database is empty, add it directly. otherwise merge with it
                if (bgDatabaseData->bgSectionData.empty())
                {
                    for (uint32 i = 0; i < numOfLanes; i++)
                    {
                        if (i == matchedLane)
                        {
                            list<vector<point3D_t>> lines;
                            lines.push_back(leftSample);
                            lines.push_back(rightSample);
                            bgDatabaseData->bgSectionData.push_back(lines);
                        }
                        else
                        {
                            list<vector<point3D_t>> lines;
                            bgDatabaseData->bgSectionData.push_back(lines);
                        }
                    } // end of lane match iteration
                }
                else
                {
                    // if the matched lane number is empty, add it directly

                    list<list<vector<point3D_t>>>::iterator bgLaneItor = bgDatabaseData->bgSectionData.begin();
                    for (uint32 i = 0; i < numOfLanes; i++)
                    {
                        if (i == matchedLane)
                        {
                            if (bgLaneItor->empty())
                            {
                                list<vector<point3D_t>> lines;
                                lines.push_back(leftSample);
                                lines.push_back(rightSample);
                                *bgLaneItor = lines;
                                break;
                            }
                            else
                            {
                                // iterate each sample points to merge data
                                // left and right line
                                for (int i = 0; i < numOfSplPnts; i++)
                                {
                                    bgLaneItor->front().at(i).lat = (0.75 * bgLaneItor->front().at(i).lat + 0.25 * leftSample[i].lat);
                                    bgLaneItor->front().at(i).paintFlag = (float)(0.75 * bgLaneItor->front().at(i).paintFlag + 0.25 * leftSample[i].paintFlag);

                                    bgLaneItor->back().at(i).lat = (0.75 * bgLaneItor->back().at(i).lat + 0.25 * rightSample[i].lat);
                                    bgLaneItor->back().at(i).paintFlag = (float)(0.75 * bgLaneItor->back().at(i).paintFlag + 0.25 * rightSample[i].paintFlag);
                                }
                            }
                        } // end of matched lane

                        bgLaneItor++;

                    } // end of lane iteration
                } // end of db not empty

#if VISUALIZATION_ON
                list<backgroundSectionData>::iterator bgDBItor = _bgDatabaseList.begin();
                while (bgDBItor != _bgDatabaseList.end())
                {
                    if (bgDBItor->sectionId == sectionConfig.segId)
                    {
                        int laneNum = 0;

                        list<list<vector<point3D_t>>>::iterator bgDBLaneItor = bgDBItor->bgSectionData.begin();
                        while (bgDBLaneItor != bgDBItor->bgSectionData.end())
                        {
                            if (!bgDBLaneItor->empty())
                            {
                                sprintf_s(IMAGE_NAME_STR, MAX_PATH - 1, "section_%d_lane_%d_merged_%d.png", sectionConfig.segId, laneNum, MERGED_TIMES);
                                showImage(*bgDBLaneItor, Scalar(0, 0, 0), IMAGE_NAME_STR);
                            }

                            laneNum++;

                            bgDBLaneItor++;
                        }

                        break;
                    }

                    bgDBItor++;
                }
#endif

                laneItor++;
            } // end of new data lane iterator

            grpItor++;
        } // end of new data group iterator

        return true;
    }


    bool CRoadVecGen::stitchSectionLanes()
    {
        // check database data
        if (_segConfigList.empty() || _bgDatabaseList.empty())
        {
            return false;
        }

        // erase previous foreground data
        if (!_fgDatabaseList.empty())
        {
            list<foregroundSectionData>::iterator fgSecItor;
            for (fgSecItor = _fgDatabaseList.begin(); fgSecItor != _fgDatabaseList.end(); fgSecItor++)
            {
                if (!fgSecItor->fgSectionData.empty())
                {
                    fgSecItor->fgSectionData.clear();
                }
            }
        }

        // number of sections
        int numOfSeg = _segConfigList.size();
        int numOfBgSeg = _bgDatabaseList.size();

        if (numOfBgSeg != numOfSeg)
        {
            return false;
        }

        // iterate each section in background database
        int numOfLanes = 0;
        list<segAttributes_t>::iterator configSecItor = _segConfigList.begin();
        list<backgroundSectionData>::iterator bgSecItor = _bgDatabaseList.begin();
        list<foregroundSectionData>::iterator fgSecItor = _fgDatabaseList.begin();
        while (bgSecItor != _bgDatabaseList.end())
        {
            numOfLanes = 0;

            // none empty sections
            if (!bgSecItor->bgSectionData.empty())
            {
                // calculate rotation angle and x limitation
                double theta = 0;
                vector<double> xlimits;
                calcRotationAngle(*configSecItor, theta, xlimits);

                // re-sample spacing in x direction
                vector<point3D_t> leftSample, rightSample;
                point3D_t pointSpl = { 0 };
                int pOrder = 1;
                int numOfSplPnts = (int)((xlimits[1] - xlimits[0]) / SAMPLE_SPACE);
                if (xlimits[0] >= xlimits[1])
                {
                    pOrder = -1;
                    numOfSplPnts = (int)((xlimits[0] - xlimits[1]) / SAMPLE_SPACE);
                }

                for (int i = 0; i < numOfSplPnts; i++)
                {
                    pointSpl.lon = xlimits[0] + i * pOrder * SAMPLE_SPACE;

                    leftSample.push_back(pointSpl);
                    rightSample.push_back(pointSpl);
                }

                // number of lanes in current section
                numOfLanes = bgSecItor->bgSectionData.size();

                if (0 < numOfLanes && numOfLanes <= MAX_SUPPORTED_LANES)
                {
                    // calculate the valid lanes number, and record the number
                    // of start lane
                    vector<int> validLaneInd;
                    int numOfValidLanes = numOfLanes;

                    list<list<vector<point3D_t>>>::iterator laneItor = bgSecItor->bgSectionData.begin();
                    while (laneItor != bgSecItor->bgSectionData.end())
                    {
                        if (!laneItor->empty())
                        {
                            validLaneInd.push_back(VALID_LANE_FLAG);
                        }
                        else
                        {
                            numOfValidLanes--;
                            validLaneInd.push_back(INVALID_LANE_FLAG);
                        }

                        laneItor++;
                    }

                    if (0 != numOfValidLanes)
                    {
                        // current section has max supported lanes(3)
                        // if lane 1 and 3 are valid, keep lane 1 and discard lane 3
                        if ((MAX_SUPPORTED_LANES == validLaneInd.size()) &&
                            (MAX_SUPPORTED_LANES - 1 == numOfValidLanes) &&
                            (INVALID_LANE_FLAG == validLaneInd[DASH_DASH]))
                        {
                            numOfValidLanes = 1;
                            validLaneInd[MAX_SUPPORTED_LANES - 1] = INVALID_LANE_FLAG;
                        }

                        // iterate valid lane to merge common lines
                        list<vector<point3D_t>> dblines;
                        vector<int>::iterator laneIndItor = validLaneInd.begin();
                        laneItor = bgSecItor->bgSectionData.begin();

                        while (laneItor != bgSecItor->bgSectionData.end())
                        {
                            if (VALID_LANE_FLAG == *laneIndItor)
                            {
                                // get paint information
                                getLinePaintInfo(laneItor->front(), leftSample);
                                getLinePaintInfo(laneItor->back(), rightSample);

                                // suppose there should be 2 lines in each lane
                                if (0/*USE_INTERPOLATION*/)
                                {
                                    interpolationSample(laneItor->front(), leftSample);
                                    interpolationSample(laneItor->back(), rightSample);
                                }
                                else
                                {
                                    polynomialFitting(laneItor->front(), leftSample);
                                    polynomialFitting(laneItor->back(), rightSample);
                                }

                                dblines.push_back(leftSample);
                                dblines.push_back(rightSample);
                            }

                            laneIndItor++;
                            laneItor++;
                        }

#if VISUALIZATION_ON
                        sprintf_s(IMAGE_NAME_STR, "fg_section_%d_polyval_sampled.png",
                                  configSecItor->segId);
                        showImage(dblines,  Scalar(0, 0, 255), IMAGE_NAME_STR);
#endif

                        // number of dblines should be double of valid lanes
                        if (2 * numOfValidLanes != dblines.size())
                        {
                            printf("number of lines not match with number of lanes\n");
                            continue;
                        }

                        // merge middle common lines
                        uint32 usedDblines = 0;
                        point3D_t curPnt = { 0 };
                        vector<double> dOne, dTwo;
                        list<vector<double>> distlines;
                        vector<point3D_t> lineOne, lineTwo, lineMerged;
                        list<vector<point3D_t>> midlines;
                        list<vector<point3D_t>>::iterator dblineItor = dblines.begin();

                        // get next items
                        if (2 < dblines.size())
                        {
                            dblineItor++; usedDblines++;
                            while ((usedDblines < dblines.size() - 1) &&
                                   (dblineItor != dblines.end()))
                            {
                                lineOne = *dblineItor;
                                dblineItor++; usedDblines++;

                                lineTwo = *dblineItor;
                                dblineItor++; usedDblines++;

                                if (lineOne.size() != lineTwo.size())
                                {
                                    printf("points of different lines on one section is not matched\n");

                                    fgSecItor++;
                                    bgSecItor++;
                                    configSecItor++;
                                    continue;
                                }

                                for (uint32 i = 0; i < lineOne.size(); i++)
                                {
                                    curPnt.lon = lineOne[i].lon;
                                    curPnt.lat = (lineOne[i].lat + lineTwo[i].lat) / 2;
                                    curPnt.paintFlag = (lineOne[i].paintFlag + lineTwo[i].paintFlag) / 2;

                                    lineMerged.push_back(curPnt);
                                    dOne.push_back(curPnt.lat - lineOne[i].lat);
                                    dTwo.push_back(curPnt.lat - lineTwo[i].lat);
                                }

                                midlines.push_back(lineMerged);
                                distlines.push_back(dOne);
                                distlines.push_back(dTwo);

                                lineMerged.clear();
                                dOne.clear();
                                dTwo.clear();
                            }
#if VISUALIZATION_ON
                            sprintf_s(IMAGE_NAME_STR, "fg_%d_section_%d_mergedmiddle_lines.png",
                                      FG_MERGED_NUM, configSecItor->segId);
                            showImage(midlines,  Scalar(0, 0, 255), IMAGE_NAME_STR);
#endif
#if SAVE_DATA_ON
                            sprintf_s(IMAGE_NAME_STR, "fg_%d_section_%d_mergedmiddle_lines.txt",
                                      FG_MERGED_NUM, configSecItor->segId);
                            saveListVec(midlines, IMAGE_NAME_STR);
#endif
                        }

                        // add lines to foreground database
                        if (1 == numOfValidLanes)
                        {
                            // two lines
                            vector<point3D_t> rotline;
                            lineRotation(dblines.front(), theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();

                            lineRotation(dblines.back(), theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                        }
                        else if (2 == numOfValidLanes)
                        {
                            // three lines
                            vector<point3D_t> rotline;
                            vector<point3D_t> linein;

                            // 1st line
                            for (uint32 i = 0; i < dblines.front().size(); i++)
                            {
                                curPnt.lon = dblines.front().at(i).lon;
                                curPnt.lat = dblines.front().at(i).lat + distlines.front().at(i);
                                curPnt.paintFlag = dblines.front().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            lineRotation(linein, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                            linein.clear();

                            // 2nd line
                            lineRotation(midlines.front(), theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();

                            // 3rd line
                            for (uint32 i = 0; i < dblines.back().size(); i++)
                            {
                                curPnt.lon = dblines.back().at(i).lon;
                                curPnt.lat = dblines.back().at(i).lat + distlines.back().at(i);
                                curPnt.paintFlag = dblines.back().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            lineRotation(linein, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                            linein.clear();
                        }
                        else
                        {
                            // four lines

                            list<vector<double>>::iterator distItor = distlines.begin();
                            vector<double> d0 = *distItor ++;
                            vector<double> d1 = *distItor ++;
                            vector<double> d2 = *distItor ++;
                            vector<double> d3 = *distItor ++;
                            vector<point3D_t> rotline;
                            vector<point3D_t> linein;

                            // 1st line
                            for (uint32 i = 0; i < dblines.front().size(); i++)
                            {
                                curPnt.lon = dblines.front().at(i).lon;
                                curPnt.lat = dblines.front().at(i).lat + d0[i] + d2[i];
                                curPnt.paintFlag = dblines.front().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            lineRotation(linein, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                            linein.clear();

                            // 2nd line
                            for (uint32 i = 0; i < midlines.front().size(); i++)
                            {
                                curPnt.lon = midlines.front().at(i).lon;
                                curPnt.lat = midlines.front().at(i).lat + d2[i];
                                curPnt.paintFlag = midlines.front().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            lineRotation(linein, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                            linein.clear();

                            // 3rd line
                            for (uint32 i = 0; i < midlines.back().size(); i++)
                            {
                                curPnt.lon = midlines.back().at(i).lon;
                                curPnt.lat = midlines.back().at(i).lat + d1[i];
                                curPnt.paintFlag = midlines.back().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            lineRotation(linein, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                            linein.clear();

                            // 4th line
                            for (uint32 i = 0; i < dblines.back().size(); i++)
                            {
                                curPnt.lon = dblines.back().at(i).lon;
                                curPnt.lat = dblines.back().at(i).lat + d1[i] + d3[i];
                                curPnt.paintFlag = dblines.back().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            lineRotation(linein, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                            linein.clear();
                        } // 3 lanes

#if VISUALIZATION_ON
                        sprintf_s(IMAGE_NAME_STR, "fg_section_%d_rotated_lines.png",
                            configSecItor->segId);
                        showImage(fgSecItor->fgSectionData, Scalar(0, 0, 255), IMAGE_NAME_STR);
#endif

                    } // end of none valid lanes
                }
                else
                {
                    printf("number of lanes in section %d is not valid, %d\n",
                           bgSecItor->sectionId, numOfLanes);
                } // end of number of lanes
            } // end of merging valid section

#if SAVE_DATA_ON
            sprintf_s(IMAGE_NAME_STR, MAX_PATH - 1, "fg_section_%d_merged_%d.txt",
                fgSecItor->sectionId, FG_MERGED_NUM);
            saveListVec(fgSecItor->fgSectionData, IMAGE_NAME_STR);
#endif

            fgSecItor++;
            bgSecItor++;
            configSecItor++;
        } // end of section iteration

        return true;
    }


    void CRoadVecGen::getSegAndDbData(IN  uint32                         sectionId,
                                      OUT segAttributes_t               &configSegData,
                                      OUT backgroundSectionData        **bgSegData)
    {
        if (_segConfigList.empty() || _bgDatabaseList.empty() || nullptr == bgSegData)
        {
            return;
        }

        // zero previous memory
        memset(&configSegData, 0, sizeof(segAttributes_t));

        // get section configuration
        list<segAttributes_t>::iterator configItor = _segConfigList.begin();
        while (configItor != _segConfigList.end())
        {
            if (sectionId == configItor->segId)
            {
                configSegData = *configItor;
                break;
            }

            configItor++;
        }

        // get background database data
        list<backgroundSectionData>::iterator dbItor = _bgDatabaseList.begin();
        while (dbItor != _bgDatabaseList.end())
        {
            if (sectionId == dbItor->sectionId)
            {
                *bgSegData = &(*dbItor);
                break;
            }
            dbItor++;
        }

        return;
    }


    void CRoadVecGen::dotLineBlockIndex(IN  vector<point3D_t> &lineData,
                                        OUT vector<int>       &dotBlkIndexSt,
                                        OUT vector<int>       &dotBlkIndexEd)
    {
        // number of points in input line
        int numOfPoints = lineData.size();
        if (0 >= numOfPoints)
        {
            dotBlkIndexSt.clear();
            dotBlkIndexEd.clear();
            return;
        }

        // extract x, y, and paint value
        vector<double> dx, dy, dp, dd;
        vector<int> index0, index1;
        for (int i = 1; i < numOfPoints; i++)
        {
            dx.push_back(lineData[i].lon - lineData[i - 1].lon);
            dy.push_back(lineData[i].lat - lineData[i - 1].lat);

            dd.push_back(dx[i - 1] + dy[i - 1]);
            if (DIST_DIFF < dd[i - 1])
            {
                index1.push_back(i - 1);
            }

            dp.push_back(lineData[i].paintFlag - lineData[i - 1].paintFlag);
            if (0 != dp[i - 1])
            {
                index0.push_back(i - 1);
            }
        }

        // dash line painting start and stop
        if (index0.empty())
        {
            dotBlkIndexSt.push_back(0);
            dotBlkIndexEd.push_back(numOfPoints - 1);
        }
        else
        {
            for (uint32 ii = 0; ii < index0.size(); ii++)
            {
                if (1.0 == dp[index0[ii]])
                {
                    // start of dash painting
                    dotBlkIndexSt.push_back(index0[ii] + 1);
                    if ((index0.size() - 1) == ii)
                    {
                        dotBlkIndexEd.push_back(numOfPoints - 1);
                    }
                }
                else
                {
                    // end of dash painting
                    dotBlkIndexEd.push_back(index0[ii]);
                    if (dotBlkIndexSt.empty())
                    {
                        dotBlkIndexSt.push_back(0);
                    }
                }
            }
        }

        // merge block
        int ind = 0;
        for (uint32 jj = 0; jj < index1.size(); jj++)
        {
            ind = index1[jj] + 1;

            for (uint32 ii = 0; ii < dotBlkIndexSt.size(); ii++)
            {
                if ((dotBlkIndexSt[ii] < ind) && (dotBlkIndexEd[ii] >= ind))
                {
                    dotBlkIndexSt.insert(dotBlkIndexSt.begin() + ii + 1, ind);
                    dotBlkIndexEd.insert(dotBlkIndexEd.begin() + ii, ind - 1);
                }
            }
        }
    }

    void CRoadVecGen::blockCombine(INOUT vector<int> &dotBlkIndexSt,
                                   INOUT vector<int> &dotBlkIndexEd)
    {
        // number of start/end block
        int stBlkSz = dotBlkIndexSt.size();
        int edBlkSz = dotBlkIndexEd.size();

        if (0 == stBlkSz || 0 == edBlkSz || stBlkSz != edBlkSz)
        {
            return;
        }

        // iterate each block
        for (int i = 1; i < stBlkSz; i++)
        {
            if (MINDIST >= abs(dotBlkIndexSt[i] - dotBlkIndexEd[i - 1]))
            {
                dotBlkIndexEd.erase(dotBlkIndexEd.begin() + i - 1);
                dotBlkIndexSt.erase(dotBlkIndexSt.begin() + i);
                stBlkSz -= 1;
            }
        }
    }

    double CRoadVecGen::getLineEstValue(IN vector<point3D_t> lineData)
    {
        // left line of input lane
        int lPointCnt = lineData.size();
        for (int i = 0; i < lPointCnt; i++)
        {
            if (lineData[i].paintFlag != 1)
            {
                lineData[i].paintFlag = 0;
            }
        }

        vector<int> dotBlkIndexSt, dotBlkIndexEd, lengthList;
        dotLineBlockIndex(lineData, dotBlkIndexSt, dotBlkIndexEd);
        blockCombine(dotBlkIndexSt, dotBlkIndexEd);

        double meanVal = 0, stdVal = 0;
        for (uint32 i = 0; i < dotBlkIndexSt.size(); i++)
        {
            int temp = dotBlkIndexEd[i] - dotBlkIndexSt[i];
            lengthList.push_back(temp);
            meanVal += temp;
        }
        meanVal = meanVal / dotBlkIndexSt.size();

        for (uint32 i = 0; i < lengthList.size(); i++)
        {
            stdVal += (lengthList[i] - meanVal) * (lengthList[i] - meanVal);
        }
        stdVal = sqrt(stdVal / lengthList.size());

        return (meanVal + stdVal);
    }


    void CRoadVecGen::polynomialFitting(IN    vector<point3D_t> &sourceLine,
                                        INOUT vector<point3D_t> &fittedLine)
    {
        // check number of input points
        int numOfSrcPnts = sourceLine.size();
        int numOfFitPnts = fittedLine.size();
        if (0 == numOfSrcPnts || 0 == numOfFitPnts)
        {
            return;
        }

        double coefficient[MAX_DEGREE + 2];
        memset(coefficient, 0, sizeof(double) * (MAX_DEGREE + 2));

        double *pDX = new double[numOfSrcPnts];
        double *pDY = new double[numOfSrcPnts];
        if (pDX && pDY)
        {
            // get X and Y value from point
            for (int i = 0; i < numOfSrcPnts; i++)
            {
                pDX[i] = sourceLine[i].lon;
                pDY[i] = sourceLine[i].lat;
            }

            // data normalization
            Point2d normalization;
            parametersNormalized(pDX, numOfSrcPnts, normalization);

            double mse = EMatrix(pDX, pDY, numOfSrcPnts, DEFAULT_DEGREE, normalization, coefficient);

            // calculate Y values
            for (int i = 0; i < numOfFitPnts; i++)
            {
                fittedLine[i].lat = calValue(fittedLine[i].lon, coefficient, normalization);

                // paint information
            }

            delete [] pDX;
            delete [] pDY;
        }
    }


    void CRoadVecGen::getLinePaintInfo(IN  vector<point3D_t> &sourceline,
                                       OUT vector<point3D_t> &leftline)
    {
        // check input lines in current lane
        if (sourceline.empty())
        {
            return;
        }

        // dash line block start/end index of input line
        vector<int> dotBlkIndexSt, dotBlkIndexEd;
        dotLineBlockIndex(sourceline, dotBlkIndexSt, dotBlkIndexEd);
        blockCombine(dotBlkIndexSt, dotBlkIndexEd);

        // number of blocks should be the same
        int numOfSt = dotBlkIndexSt.size();
        int numOfEd = dotBlkIndexEd.size();
        if ((numOfSt != numOfEd) || (numOfSt == 0) || (numOfEd == 0))
        {
            return;
        }

        // get the x values of each block start/end
        int blkIndSt = 0, blkIndEd = 0;
        vector<double> leftValSt, leftValEd;
        int numOfPnts = sourceline.size();
        for (int i = 0; i < numOfPnts; i++)
        {
            if (i == dotBlkIndexSt[blkIndSt])
            {
                leftValSt.push_back(sourceline[i].lon);
                blkIndSt = ((blkIndSt + 1) >= (numOfSt - 1)) ? (numOfSt - 1) : (blkIndSt + 1);
            }

            if (i == dotBlkIndexEd[blkIndEd])
            {
                leftValEd.push_back(sourceline[i].lon);
                blkIndEd = ((blkIndEd + 1) >= (numOfEd - 1)) ? (numOfEd - 1) : (blkIndEd + 1);
            }
        }

        // number of values should be the same
        if (leftValSt.size() != leftValEd.size())
        {
            return;
        }

#if 1
        // map to x value on the sample line
        int numOfLeftPnts = leftline.size();
        double step = leftline[1].lon - leftline[0].lon;
        vector<int> leftSmpSt, leftSmpEd;
        for (uint32 i = 0; i < leftValSt.size(); i++)
        {
            blkIndSt = (int)((leftValSt[i] - leftline[0].lon) / step + 0.5) + 1;
            blkIndEd = (int)((leftValEd[i] - leftline[0].lon) / step + 0.5) + 1;

            if (blkIndSt < 0)
            {
                leftSmpSt.push_back(0);
            }
            else if (blkIndSt >= numOfLeftPnts - 1)
            {
                leftSmpSt.push_back(numOfLeftPnts - 1);
            }
            else
            {
                leftSmpSt.push_back(blkIndSt);
            }

            if (blkIndEd < 0)
            {
                leftSmpEd.push_back(0);
            }
            else if (blkIndEd >= numOfLeftPnts - 1)
            {
                leftSmpEd.push_back(numOfLeftPnts - 1);
            }
            else
            {
                leftSmpEd.push_back(blkIndEd);
            }
        }

        // get paint information for dash line block
        bool bPaintedBlk = false;
        uint32 blkIndStEd = 0;
        uint32 blkNumStEd = leftSmpSt.size();

        // find first none 0 blocks
        while ((blkIndStEd < blkNumStEd) && (0 == leftSmpSt[blkIndStEd]) && (0 == leftSmpEd[blkIndStEd]))
        {
            blkIndStEd++;
        }

        if (blkIndStEd < blkNumStEd)
        {
            for (int i = 0; i < numOfLeftPnts; i++)
            {
                if (leftSmpSt[blkIndStEd] <= i && leftSmpEd[blkIndStEd] >= i)
                {
                    bPaintedBlk = true;
                    leftline[i].paintFlag = 1;
                }

                if (i > leftSmpEd[blkIndStEd] && bPaintedBlk)
                {
                    // add block index
                    bPaintedBlk = false;
                    blkIndStEd = ((blkIndStEd + 1) >= (blkNumStEd - 1)) ? \
                        (blkNumStEd - 1) : (blkIndStEd + 1);
                }
            }
        }
#else
        // assign paint information according to x range
        bool bPaintedBlk = false;
        int blkIndStEd = 0;
        int blkNumStEd = leftValSt.size();
        int numOfLinePnts = leftline.size();
        bool bNormalOrder = leftValSt[0] < leftValSt[blkNumStEd - 1];
        if (bNormalOrder)
        {
            for (int i = 0; i < numOfLinePnts; i++)
            {
                if (leftValSt[blkIndStEd] <= leftline[i].lon &&
                    leftValEd[blkIndStEd] >= leftline[i].lon)
                {
                    bPaintedBlk = true;
                    leftline[i].paintFlag = 1;
                }

                if (leftValEd[blkIndStEd] < leftline[i].lon && bPaintedBlk)
                {
                    // add block index
                    bPaintedBlk = false;
                    blkIndStEd = ((blkIndStEd + 1) >= (blkNumStEd - 1)) ? \
                        (blkNumStEd - 1) : (blkIndStEd + 1);
                }
            }
        }
        else
        {
            for (int i = 0; i < numOfLinePnts; i++)
            {
                if (leftValSt[blkIndStEd] >= leftline[i].lon &&
                    leftValEd[blkIndStEd] <= leftline[i].lon)
                {
                    bPaintedBlk = true;
                    leftline[i].paintFlag = 1;
                }

                if (leftValEd[blkIndStEd] > leftline[i].lon && bPaintedBlk)
                {
                    // add block index
                    bPaintedBlk = false;
                    blkIndStEd = ((blkIndStEd + 1) >= (blkNumStEd - 1)) ? \
                        (blkNumStEd - 1) : (blkIndStEd + 1);
                }
            }
        }
#endif
    }


}
