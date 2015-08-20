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

static uint32 MERGED_TIMES = 1;
#endif

using namespace std;

namespace ns_database
{
#define DASH_CONT_POINTS_TH 10
#define MINDIST             10
#define LINE_TYPE_TH        50
#define SAMPLE_SPACE        0.05

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

        fscanf_s(fp, "%d\n", &sectionNum);
		
		fscanf_s(fp, "width=%lf,overlap=%lf,minLength=%lf,maxLength=%lf,stepSize=%d,paintV=%lf\n",
        &(_stSecConfig.dbWidth), &(_stSecConfig.dbOverlap),
        &(_stSecConfig.dbMinLength), &(_stSecConfig.dbMaxLength),
        &(_stSecConfig.uiStepSize), &(_stSecConfig.dbPaintV));
        while (!feof(fp))/*for(int index = 0; index < sectionNum; index++)*/
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

        // judge matched lane number
        if (leftVal > LINE_TYPE_TH && rightVal < LINE_TYPE_TH)
        {
            laneNumber = 1;
        }
        if (leftVal < LINE_TYPE_TH && rightVal < LINE_TYPE_TH)
        {
            laneNumber = 2;
        }
        if (leftVal < LINE_TYPE_TH && rightVal > LINE_TYPE_TH)
        {
            laneNumber = 3;
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
        // for test only, should get it from section configuration
        uint32 numOfLanes = 3;

        // matched lane number
        uint32 matchedLane = 0;

        vector<point3D_t> leftRotated;
        vector<point3D_t> rightRotated;

        // number of data group in report data
        int numOfGroups = reportData.rptSecData.size();
        list<list<list<vector<point3D_t>>>>::iterator grpItor = reportData.rptSecData.begin();
        while (grpItor != reportData.rptSecData.end())
        {
            // lane
            list<list<vector<point3D_t>>>::iterator laneItor = grpItor->begin();
            while (laneItor != grpItor->end())
            {
                // step 1 - Lane number identification
                matchLaneType(*laneItor, matchedLane);

                // step 2 - Process new data before merging
                // line - there should be two lines. rotate first
                list<vector<point3D_t>>::iterator lineItor = laneItor->begin();

                lineRotation(*lineItor, -theta, leftRotated);  // left line
                lineItor++;
                lineRotation(*lineItor, -theta, rightRotated); // right line

#ifdef VISUALIZATION_ON
                list<vector<point3D_t>> rotated;
                rotated.push_back(leftRotated);
                rotated.push_back(rightRotated);
                showImage(rotated,  Scalar(0, 255, 0), "rotated.png");
#endif

                // re-sample or polynomial fitting
                if (isResampleSec(sectionConfig.segId))
                {
                    interpolationSample(leftRotated,  leftSample);
                    interpolationSample(rightRotated, rightSample);
#ifdef VISUALIZATION_ON
                    list<vector<point3D_t>> intersampled;
                    intersampled.push_back(leftSample);
                    intersampled.push_back(rightSample);
                    showImage(intersampled,  Scalar(0, 0, 255), "interpolateSampled.png");
#endif
                }
                else
                {
                    polynomialFitting(leftRotated,  leftSample);
                    polynomialFitting(rightRotated, rightSample);
#ifdef VISUALIZATION_ON
                    list<vector<point3D_t>> polysampled;
                    polysampled.push_back(leftSample);
                    polysampled.push_back(rightSample);
                    showImage(polysampled,  Scalar(0, 0, 255), "polyvalSampled.png");
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
                        char winname[MAX_PATH];

                        list<list<vector<point3D_t>>>::iterator bgDBLaneItor = bgDBItor->bgSectionData.begin();
                        while (bgDBLaneItor != bgDBItor->bgSectionData.end())
                        {
                            if (!bgDBLaneItor->empty())
                            {
                                sprintf_s(winname, MAX_PATH - 1, "section_%d_lane_%d_merged_%d.png", sectionConfig.segId, laneNum, MERGED_TIMES++);
                                showImage(*bgDBLaneItor, Scalar(0, 0, 0), winname);
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


    bool CRoadVecGen::stitchSectionLanes(IN  segAttributes_t          sectionConfig,
                                         IN  backgroundSectionData    bgDatabaseData,
                                         OUT foregroundSectionData   &fgSectionData)
    {

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

        if ((*bgSegData) && !(*bgSegData)->bgSectionData.empty())
        {
            list<list<vector<point3D_t>>>::iterator laneItor = (*bgSegData)->bgSectionData.begin();
            while (laneItor != (*bgSegData)->bgSectionData.end())
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
        stdVal = stdVal / lengthList.size();

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
