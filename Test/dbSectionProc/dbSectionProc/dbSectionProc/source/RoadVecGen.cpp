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

#ifdef VISUALIZATION_ON
#include "VisualizationApis.h"

static uint32 PREVIOUS_SEGID = 0;
static uint32 MERGED_TIMES = 0;
static uint32 FG_MERGED_NUM = 0;

char IMAGE_NAME_STR[MAX_PATH] = { '\0' };
#endif

using namespace std;

namespace ns_database
{
#define DASH_CONT_POINTS_TH 10
#define MINDIST             10
#define LINE_TYPE_TH        50
#define SAMPLE_SPACE        0.1
#define MAX_SUPPORTED_LANES 3
#define INVALID_LANE_FLAG   -1
#define VALID_LANE_FLAG     1
#define LEFT_LANE           1
#define MIDDLE_LANE         2
#define RIGHT_LANE          3

const uint32 RESAMPLE_SEC_ID[] = {1, 8, 9, 10, 11, 12, 20, 21};

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
        _configPath[0] = '\0';
    }


    CRoadVecGen::CRoadVecGen(char *configFilePath)
    {
        if (nullptr == configFilePath)
        {
            _configPath[0] = '\0';
        }
        else
        {
            memcpy_s(_configPath, MAX_PATH - 1, configFilePath, MAX_PATH - 1);
        }

        // read section configuration file
        readSecConfig();

        // initialize database
        initDatabase();
    }


    CRoadVecGen::~CRoadVecGen()
    {
        _configPath[0] = '\0';

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


    void CRoadVecGen::roadSectionsGen(IN  list<list<vector<point3D_t>>>  rptData,
                                      OUT list<list<vector<point3D_t>>> &fgData)
    {
        if (rptData.empty())
        {
            return;
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

#ifdef VISUALIZATION_ON
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
        showImage(fglines, Scalar(0, 0, 255), IMAGE_NAME_STR);

#endif

        return;
    }


    void CRoadVecGen::setSectionConfigPath(char *filename)
    {
        if (nullptr != filename)
        {
            memcpy_s(_configPath, MAX_PATH - 1, filename, MAX_PATH - 1);
        }

        // read section configuration file
        readSecConfig();

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


    void CRoadVecGen::readSecConfig()
    {
        int sectionNum = 0;
        int sectionID  = 0;

        segAttributes_t segmentElement;

        segmentElement.segId_used          = 0;
        segmentElement.version_used        = 0;
        segmentElement.type_used           = 0;
        segmentElement.numPort_used        = 0;
        segmentElement.ports_used          = 0;
        segmentElement.links_used          = 0;
        segmentElement.roadLength_used     = 0;
        segmentElement.bridgeFlag_used     = 0;
        segmentElement.tunnelFlag_used     = 0;
        segmentElement.numFurniture_used   = 0;
        segmentElement.numDynamicData_used = 0;

        FILE *fp = nullptr;
        errno_t err = fopen_s(&fp, _configPath, "rt");
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
            fscanf_s(fp, "%d,%d\n", &sectionID, &segmentElement.uiLaneNum);
            segmentElement.segId_used = 1;
            segmentElement.segId      = sectionID;
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
    }


    void CRoadVecGen::interpolationSample(IN    vector<point3D_t>  sourceLine,
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
                sampledLine[index].paintFlag = sourceLine[0].paintFlag;
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
                    sampledLine[index].paintFlag = sourceLine[x0].paintFlag;
                }
                else
                {
                    sampledLine[index].lat = (sourceLine[x0].lon - sampledLine[index].lon) * sourceLine[x1].lat / (sourceLine[x0].lon - sourceLine[x1].lon) +
                        (sampledLine[index].lon - sourceLine[x1].lon) * sourceLine[x0].lat / (sourceLine[x0].lon - sourceLine[x1].lon);
                    sampledLine[index].paintFlag = (sourceLine[x0].paintFlag + sourceLine[x1].paintFlag) / 2;
                }
            }//end for sampledLine
        }// end if
    }


    void CRoadVecGen::calcRotationAngle(IN  segAttributes_t  sectionConfig,
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
            xLimitation.push_back(min(X0[0], X0[1]));
            xLimitation.push_back(max(X0[2], X0[3]));
            xLimitation.push_back(min(X0[4], X0[5]));
            xLimitation.push_back(max(X0[6], X0[7]));
        }
        else
        {
            xLimitation.push_back(max(X0[0], X0[1]));
            xLimitation.push_back(min(X0[2], X0[3]));
            xLimitation.push_back(max(X0[4], X0[5]));
            xLimitation.push_back(min(X0[6], X0[7]));
        }
    }


    void CRoadVecGen::lineRotation(IN  vector<point3D_t>  sourceLine,
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
            pointZ.paintFlag = sourceLine[i].paintFlag;
            pointZ.count     = 1;

            rotatedLine.push_back(pointZ);
        }
    }


    void CRoadVecGen::matchLaneType(IN  list<vector<point3D_t>>  sourceLane,
                                    OUT uint32                  &laneNumber)
    {
        // check number of input lines
        if (2 != sourceLane.size())
        {
            laneNumber = 0;
            return;
        }

        // get left and right line of input lane
        vector<point3D_t> lineLeft, lineRight;
        list<vector<point3D_t>>::iterator it = sourceLane.begin();
        lineLeft  = *it;
        it++;
        lineRight = *it;

        // left line of input lane
        double leftVal = getLineEstValue(lineLeft);

        // right line of input lane
        double rightVal = getLineEstValue(lineRight);

        // judge matched lane number, default is left lane
        laneNumber = LEFT_LANE;
        if (leftVal > LINE_TYPE_TH && rightVal < LINE_TYPE_TH)
        {
            laneNumber = LEFT_LANE;
        }
        if (leftVal < LINE_TYPE_TH && rightVal < LINE_TYPE_TH)
        {
            laneNumber = MIDDLE_LANE;
        }
        if (leftVal < LINE_TYPE_TH && rightVal > LINE_TYPE_TH)
        {
            laneNumber = RIGHT_LANE;
        }
    }


    bool CRoadVecGen::mergeSectionLane(IN    segAttributes_t        sectionConfig,
                                       IN    reportSectionData      reportData,
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
            // lane
            list<list<vector<point3D_t>>>::iterator laneItor = grpItor->begin();
            while (laneItor != grpItor->end())
            {
                leftRotated.clear();
                rightRotated.clear();
                leftRotatedValid.clear();
                rightRotatedValid.clear();

                // step 1 - Lane number identification
                matchLaneType(*laneItor, matchedLane);

                // step 2 - Process new data before merging
                // line - there should be two lines. rotate first

                lineRotation(laneItor->front(), -theta, leftRotated);  // left line
                lineRotation(laneItor->back(), -theta, rightRotated);  // right line

#ifdef VISUALIZATION_ON
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

                // re-sample or polynomial fitting
                if (isResampleSec(sectionConfig.segId))
                {
                    interpolationSample(leftRotatedValid,  leftSample);
                    interpolationSample(rightRotatedValid, rightSample);
#ifdef VISUALIZATION_ON
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
#ifdef VISUALIZATION_ON
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
                        if (i == matchedLane - 1)
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
                        if (i == matchedLane - 1)
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
                                list<vector<point3D_t>>::iterator bgLineItor = bgLaneItor->begin();

                                // iterate each sample points to merge data
                                // left line
                                for (int i = 0; i < numOfSplPnts; i++)
                                {
                                    bgLineItor->at(i).lat = (bgLineItor->at(i).lat + leftSample[i].lat) / 2;
                                }

                                // right line
                                bgLineItor++;
                                for (int i = 0; i < numOfSplPnts; i++)
                                {
                                    bgLineItor->at(i).lat = (bgLineItor->at(i).lat + rightSample[i].lat) / 2;
                                }
                            }
                        } // end of matched lane

                        bgLaneItor++;

                    } // end of lane iteration
                } // end of db not empty

#ifdef VISUALIZATION_ON
                list<backgroundSectionData>::iterator bgDBItor = _bgDatabaseList.begin();
                while (bgDBItor != _bgDatabaseList.end())
                {
                    if (bgDBItor->sectionId == sectionConfig.segId)
                    {
                        int laneNum = 1;

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

#ifdef VISUALIZATION_ON
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
                            (INVALID_LANE_FLAG == validLaneInd[MAX_SUPPORTED_LANES - 1]))
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
                                // suppose there should be 2 lines in each lane
                                polynomialFitting(laneItor->front(),  leftSample);
                                polynomialFitting(laneItor->back(), rightSample);

                                dblines.push_back(leftSample);
                                dblines.push_back(rightSample);
                            }

                            laneIndItor++;
                            laneItor++;
                        }

#ifdef VISUALIZATION_ON
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
                        point3D_t curPnt = { 0 };
                        vector<double> dOne, dTwo;
                        list<vector<double>> distlines;
                        vector<point3D_t> lineOne, lineTwo, lineMerged;
                        list<vector<point3D_t>> midlines;
                        list<vector<point3D_t>>::iterator dblineItor = dblines.begin();

                        // get next items
                        if (2 < dblines.size())
                        {
                            dblineItor++;
                            while (dblineItor != dblines.end())
                            {
                                lineOne = *dblineItor;
                                dblineItor++;

                                lineTwo = *dblineItor;
                                dblineItor++; dblineItor++; // next lane common line

                                if (lineOne.size() != lineTwo.size())
                                {
                                    printf("points of different lines on one section is not matched\n");
                                    continue;
                                }

                                for (uint32 i = 0; i < lineOne.size(); i++)
                                {
                                    curPnt.lon = lineOne[i].lon;
                                    curPnt.lat = (lineOne[i].lat + lineTwo[i].lat) / 2;

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
#ifdef VISUALIZATION_ON
                            sprintf_s(IMAGE_NAME_STR, "fg_section_%d_mergedmiddle_lines.png",
                                      configSecItor->segId);
                            showImage(midlines,  Scalar(0, 0, 255), IMAGE_NAME_STR);
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

                            // 1st line
                            vector<point3D_t> linest;

                            list<vector<double>>::iterator distItor = distlines.begin();

                            dblineItor = dblines.begin();
                            for (uint32 i = 0; i < dblineItor->size(); i++)
                            {
                                curPnt.lon = dblineItor->at(i).lon;
                                curPnt.lat = dblineItor->at(i).lat + distItor->at(i);

                                linest.push_back(curPnt);
                            }
                            lineRotation(linest, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();

                            // 2nd line
                            lineRotation(midlines.front(), theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();

                            // 3rd line
                            dblineItor++; dblineItor++; dblineItor++; distItor++;
                            for (uint32 i = 0; i < dblineItor->size(); i++)
                            {
                                curPnt.lon = dblineItor->at(i).lon;
                                curPnt.lat = dblineItor->at(i).lat + distItor->at(i);

                                linest.push_back(curPnt);
                            }
                            lineRotation(linest, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
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

                            // 1st line
                            vector<point3D_t> linest;
                            vector<point3D_t> dbfirst = dblines.front();
                            for (uint32 i = 0; i < dbfirst.size(); i++)
                            {
                                curPnt.lon = dbfirst[i].lon;
                                curPnt.lat = dbfirst[i].lat + d0[i] + d2[i];

                                linest.push_back(curPnt);
                            }
                            lineRotation(linest, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();

                            // 2nd line
                            lineRotation(midlines.front(), theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();

                            // 3rd line
                            lineRotation(midlines.back(), theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();

                            // 4th line
                            vector<point3D_t> linend;
                            vector<point3D_t> dblast = dblines.back();
                            for (uint32 i = 0; i < dblast.size(); i++)
                            {
                                curPnt.lon = dblineItor->at(i).lon;
                                curPnt.lat = dblineItor->at(i).lat + distItor->at(i);

                                linend.push_back(curPnt);
                            }
                            lineRotation(linend, theta, rotline);
                            fgSecItor->fgSectionData.push_back(rotline);
                            rotline.clear();
                        } // 3 lanes

#ifdef VISUALIZATION_ON
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


    void CRoadVecGen::dotLineBlockIndex(IN  vector<point3D_t>  lineData,
                                        OUT vector<int>       &dotBlkIndexSt,
                                        OUT vector<int>       &dotBlkIndexEd)
    {
        // number of points in input line
        int numOfPoints = lineData.size();

        int blkIndex = 1;
        bool bChange = false;

        // iterate each point
        for (int i = 0; i < numOfPoints; i++)
        {
            float value = lineData[i].paintFlag;
            if (value == 1.0)
            {
                if (bChange == false)
                {
                    dotBlkIndexSt.push_back(i);
                }
                bChange = true;
            }
            else
                if (bChange == true)
                {
                    dotBlkIndexEd.push_back(i-1);
                    if (DASH_CONT_POINTS_TH < (dotBlkIndexEd[blkIndex - 1] -
                                               dotBlkIndexSt[blkIndex - 1]))
                    {
                        blkIndex += 1;
                    }
                    bChange = false;
                }
        }

        if ((true == bChange) && (dotBlkIndexSt.size() != dotBlkIndexEd.size()))
        {
            dotBlkIndexEd.push_back(numOfPoints);
        }
    }

    void CRoadVecGen::blockCombine(INOUT vector<int> &dotBlkIndexSt,
                                   INOUT vector<int> &dotBlkIndexEd)
    {
        // number of start/end block
        int stBlkSz = dotBlkIndexSt.size();
        int edBlkSz = dotBlkIndexEd.size();

        if (0 == stBlkSz || 0 == edBlkSz)
        {
            return;
        }

        // iterate each block
        for (int i = 0; i < stBlkSz - 1; i++)
        {
            if (i < stBlkSz - 1)
            {
                Point lastLine = Point(dotBlkIndexSt[i],   dotBlkIndexEd[i]);
                Point nextLine = Point(dotBlkIndexSt[i+1], dotBlkIndexEd[i+1]);

                if (abs(nextLine.x - lastLine.y) < MINDIST)
                {
                    dotBlkIndexEd[i] = nextLine.y;
                    for (int j = i + 1; j < stBlkSz - 1;j++)
                    {
                        dotBlkIndexSt[j] = dotBlkIndexSt[j + 1];
                        dotBlkIndexEd[j] = dotBlkIndexEd[j + 1];
                    }
                    dotBlkIndexSt[stBlkSz - 1] = 0;
                    dotBlkIndexEd[stBlkSz - 1] = 0;
                    stBlkSz -= 1;
                }
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


    void CRoadVecGen::polynomialFitting(IN    vector<point3D_t>  sourceLine,
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



}
