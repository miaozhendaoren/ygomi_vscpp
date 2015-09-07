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
#include "RoadVecGen2.h"

#if VISUALIZATION_ON || SAVE_DATA_ON
#include "VisualizationApis.h"

static uint32 PREVIOUS_SEGID = 0;
static uint32 MERGED_TIMES   = 0;
static uint32 FG_MERGED_NUM  = 0;

char IMAGE_NAME_STR2[MAX_PATH] = { '\0' };

using namespace std;
#endif


namespace ns_database
{
    // number of section configuration points
#define SEG_CFG_PNT_NUM           4

#define MINDIST                   10
#define LINE_TYPE_TH              40
#define SAMPLE_SPACE              0.1
#define SINGLE_LANE               1
#define DOUBLE_LANE               2
#define MAX_SUPPORTED_LANES       3
#define INVALID_LANE_FLAG        -1
#define VALID_LANE_FLAG           1
#define DASH_DIST_DIFF            2.5
#define INVALID_LANE_NUM         -1
#define SOLID_DASH                0
#define DASH_DASH                 1
#define DASH_SOLID                2
#define SOLID_SOLID               0

#define SAFEARR_DELETE(p) if (p) { delete [] (p); p = nullptr; }

    enum RESAMPLE_METHOD
    {
        USE_INTERPOLATION = 0,
        USE_POLYNOMIALFIT = 1,
    };

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

    CRoadVecGen2::CRoadVecGen2(void)
    {
        _configPath = "";
        _curSecId = 0;

        // create mutex
        _hMutexMerging = CreateMutex(NULL, FALSE, NULL);
        ReleaseMutex(_hMutexMerging);
    }


    CRoadVecGen2::CRoadVecGen2(string configFilePath)
    {
        _configPath = configFilePath;

        // read section configuration file
        list<segAttributes_t> segConfigList;
        readSecConfig(segConfigList);

        // initialize database
        initDatabase();

        // calculate section rotation angle and X data range
        calcRotAngleAndRange();

        // create mutex
        _hMutexMerging = CreateMutex(NULL, FALSE, NULL);
        ReleaseMutex(_hMutexMerging);
    }


    CRoadVecGen2::~CRoadVecGen2(void)
    {
        _configPath = "";

        // release list or vector data
        _secRotAngle.clear();
        _secBodyStInd.clear();
        _secBodyEdInd.clear();
        _secLeftData.clear();
        _secRightData.clear();
        _segConfigList.clear();
        _bgDatabaseList.clear();
        _fgDatabaseList.clear();

        // close mutex
        CloseHandle(_hMutexMerging);
    }


    bool CRoadVecGen2::roadSectionsGen(IN  list<list<vector<point3D_t>>> &rptData,
                                       OUT list<list<vector<point3D_t>>> &fgData)
    {
        if (rptData.empty())
        {
            return false;
        }

        WaitForSingleObject(_hMutexMerging, INFINITE);

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
                        sprintf_s(IMAGE_NAME_STR2, MAX_PATH - 1,
                                  "fg_%d_sec_%d_group_%d_lane_%d.txt",
                                  FG_MERGED_NUM, fsec->sectionId, groupNum, laneNum);
                        saveListVec(*lane, IMAGE_NAME_STR2);

                        lane++; laneNum++;
                    }

                    grp++; groupNum++;
                }
            }

            fsec++;
        }
#endif

        list<reportSectionData>::size_type numOfRptSecs = secData.size();

        backgroundSectionData *bgSecData = nullptr;

        // iterate each section
        list<reportSectionData>::iterator secItor = secData.begin();
        while (secItor != secData.end())
        {
            // current reported section ID
            _curSecId = secItor->sectionId;

            // get section configuration and database data
            getDatabaseData(&bgSecData);

            // merge new data with database
            mergeSectionLane(*secItor, bgSecData);

            secItor++;
        }

        // stitch background database to generate foreground data
        stitchSectionLanes();

        // remove section overlap and output data
        removeOverlap(/*fgData*/);

        jointProcessing(fgData);

#if VISUALIZATION_ON || SAVE_DATA_ON
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
        sprintf_s(IMAGE_NAME_STR2, "fgdatabase_%d.png", FG_MERGED_NUM++);
#if VISUALIZATION_ON
        showImage(fglines, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif // end of visualization
#if SAVE_DATA_ON
        saveListVec(fglines, "fgroup_0.txt");
#endif // end of save data
#endif // end of visualization or save data

        ReleaseMutex(_hMutexMerging);

        return true;
    }


    void CRoadVecGen2::setSectionConfigPath(IN string                  filename,
                                            OUT list<segAttributes_t> &segConfigList)
    {
        _configPath = filename;

        // read section configuration file
        readSecConfig(segConfigList);

        // initialize database
        initDatabase();

        // calculate section rotation angle and X data range
        calcRotAngleAndRange();
    }


    uint32 CRoadVecGen2::getSectionId(IN point3D_t p)
    {


        return 0;
    }


    void CRoadVecGen2::getShitDist(OUT vector<double> &dist)
    {

    }


    void CRoadVecGen2::resetDatabase()
    {
        WaitForSingleObject(_hMutexMerging, INFINITE);

        list<backgroundSectionData>::iterator bgItor = _bgDatabaseList.begin();
        while (bgItor != _bgDatabaseList.end())
        {
            bgItor->bgSectionData.clear();

            bgItor++;
        }

        list<foregroundSectionData>::iterator fgItor = _fgDatabaseList.begin();
        while (fgItor != _fgDatabaseList.end())
        {
            fgItor->fgSectionData.clear();

            fgItor++;
        }

        ReleaseMutex(_hMutexMerging);
    }


    void CRoadVecGen2::readSecConfig(OUT list<segAttributes_t> &segCfgList)
    {
        int sectionNum = 0;
        int sectionID  = 0;
        int laneNum = 0;

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
        segmentElement.uiLaneNum_used      = 0;

        FILE *fp = nullptr;
        errno_t err = fopen_s(&fp, _configPath.c_str(), "rt");
        if(0 != err)
        {
            return;
        }

        // number of sections
        fscanf_s(fp, "%d\n", &(_stSecConfig.uiSecNum));

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
            fscanf_s(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
                &segmentElement.ports[0].lon, &segmentElement.ports[0].lat,
                &segmentElement.ports[1].lon, &segmentElement.ports[1].lat,
                &segmentElement.ports[2].lon, &segmentElement.ports[2].lat,
                &segmentElement.ports[3].lon, &segmentElement.ports[3].lat);

            _secLaneNum.push_back(laneNum);
            _segConfigList.push_back(segmentElement);
        }

        fclose(fp);

        segCfgList = _segConfigList;
    }


    void CRoadVecGen2::initDatabase()
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


    void CRoadVecGen2::interpolationSample(IN    vector<point3D_t> &sourceLine,
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


    void CRoadVecGen2::calcRotAngleAndRange()
    {
        // check section configuration list
        if (_segConfigList.empty())
        {
            return;
        }

        double x0(0.0), x1(0.0), y0(0.0), y1(0.0), X0Temp(0.0);
        double theta = 0.0;
        double xlimits[SEG_CFG_PNT_NUM] = {0.0, 0.0, 0.0, 0.0};

        // re-sample variables
        point3D_t pointSpl = { 0 };
        int pOrder = 1, numOfSplPnts = 0, stInd = 0, edInd = 0;
        vector<point3D_t> leftSample, rightSample;

        // calculate each rotation angle and X range
        list<segAttributes_t>::iterator segItor = _segConfigList.begin();
        while (segItor != _segConfigList.end())
        {
            // clear internal variables
            leftSample.clear();
            rightSample.clear();

            x0 = segItor->ports[2].lon;
            x1 = segItor->ports[3].lon;

            y0 = segItor->ports[2].lat;
            y1 = segItor->ports[3].lat;

            theta = atan2((y0 - y1), (x0 - x1));

            // push rotation angle to _secRotAngle
            _secRotAngle.push_back(theta);

            complex<double> thetaj(0, -1 * theta);

            for (int i = 0; i < SEG_CFG_PNT_NUM; i++)
            {
                complex<double> yi(0, segItor->ports[i].lat);
                xlimits[i] = real((segItor->ports[i].lon + yi) * exp(thetaj));
            }


            // calculate re-sample points
            numOfSplPnts = (int)((xlimits[3] - xlimits[2]) / SAMPLE_SPACE);
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

            _secLeftData.push_back(leftSample);
            _secRightData.push_back(rightSample);

            // section body start and end index of sample points
            stInd = (int)(ceil((xlimits[0] - xlimits[2]) / (pOrder * SAMPLE_SPACE))) + 1;
            edInd = (int)(ceil((xlimits[1] - xlimits[2]) / (pOrder * SAMPLE_SPACE))) + 1;
            _secBodyStInd.push_back(stInd);
            _secBodyEdInd.push_back(edInd);

            segItor++;
        }
    }

    void CRoadVecGen2::lineRotation(IN  vector<point3D_t> &sourceLine,
                                    IN  double             theta,
                                    OUT vector<point3D_t> &rotatedLine)
    {
        int numOfPnts = sourceLine.size();

        complex<double> thetaj(0, theta);
        double x = 0, y = 0, realZ = 0, imagZ = 0;
        point3D_t pointZ = { 0 };

        for (int i = 0; i < numOfPnts; i++)
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


    bool CRoadVecGen2::mergeSectionLane(IN    reportSectionData     &reportData,
                                        INOUT backgroundSectionData *bgDatabaseData)
    {
        // check section Id first
        if (nullptr == bgDatabaseData ||
            _curSecId != reportData.sectionId ||
            _curSecId != bgDatabaseData->sectionId)
        {
            return false;
        }

        // get left/right re-sample line data according to current section Id
        uint32 curItem = 0;
        vector<point3D_t> *leftSample = nullptr, *rightSample = nullptr;
        list<vector<point3D_t>>::iterator splLItor = _secLeftData.begin();
        list<vector<point3D_t>>::iterator splRItor = _secRightData.begin();
        while (splLItor != _secLeftData.end())
        {
            if (curItem  == _curSecId - 1)
            {
                leftSample  = &(*splLItor);
                rightSample = &(*splRItor);
            }
            splLItor++; splRItor++; curItem++;
        }
        int numOfSplPnts = leftSample->size();

        // number of lanes in current section
        uint32 numOfLanes = _secLaneNum[_curSecId - 1];

        // matched lane number
        int matchedLane = 0;
        bool bValid = false;

        // number of data group in report data
        int numOfGroups = reportData.rptSecData.size();

        // iterator definition
        list<list<vector<point3D_t>>>::iterator laneItor;
        list<list<vector<point3D_t>>>::iterator bgLaneItor;
        list<list<list<vector<point3D_t>>>>::iterator grpItor;

        // iterate for each group data
        grpItor = reportData.rptSecData.begin();
        while (grpItor != reportData.rptSecData.end())
        {
#if VISUALIZATION_ON
            if (PREVIOUS_SEGID == _curSecId)
            {
                MERGED_TIMES++;
            }
            else
            {
                PREVIOUS_SEGID = _curSecId;
                MERGED_TIMES = 0;
            }
#endif

            // lane
            laneItor = grpItor->begin();
            while (laneItor != grpItor->end())
            {
                // reset Y and paint for sample vector
                for (int i = 0; i < numOfSplPnts; i++)
                {
                    leftSample->at(i).lat        = 0.0;
                    leftSample->at(i).paintFlag  = 0.0;
                    rightSample->at(i).lat       = 0.0;
                    rightSample->at(i).paintFlag = 0.0;
                }

                // data preprocessing
                bValid = LaneDataPreprocessing(*laneItor, matchedLane,
                                               *leftSample, *rightSample);

                // if valid, merge with database lane data
                if (bValid)
                {
                    // if it is empty, add it directly. otherwise merge with it
                    if (bgDatabaseData->bgSectionData.empty())
                    {
                        for (uint32 i = 0; i < numOfLanes; i++)
                        {
                            if (i == matchedLane)
                            {
                                list<vector<point3D_t>> lines;
                                lines.push_back(*leftSample);
                                lines.push_back(*rightSample);
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
                        bgLaneItor = bgDatabaseData->bgSectionData.begin();
                        for (uint32 i = 0; i < numOfLanes; i++)
                        {
                            if (i == matchedLane)
                            {
                                if (bgLaneItor->empty())
                                {
                                    list<vector<point3D_t>> lines;
                                    lines.push_back(*leftSample);
                                    lines.push_back(*rightSample);
                                    *bgLaneItor = lines;
                                    break;
                                }
                                else
                                {
                                    // iterate each sample points to merge data
                                    // left and right line
                                    for (int i = 0; i < numOfSplPnts; i++)
                                    {
                                        bgLaneItor->front().at(i).lat = (0.75 * bgLaneItor->front().at(i).lat + 0.25 * leftSample->at(i).lat);
                                        bgLaneItor->front().at(i).paintFlag = (float)(bgLaneItor->front().at(i).paintFlag + leftSample->at(i).paintFlag);

                                        bgLaneItor->back().at(i).lat = (0.75 * bgLaneItor->back().at(i).lat + 0.25 * rightSample->at(i).lat);
                                        bgLaneItor->back().at(i).paintFlag = (float)(bgLaneItor->back().at(i).paintFlag + rightSample->at(i).paintFlag);
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
                        if (bgDBItor->sectionId == _curSecId)
                        {
                            int laneNum = 0;

                            list<list<vector<point3D_t>>>::iterator bgDBLaneItor = bgDBItor->bgSectionData.begin();
                            while (bgDBLaneItor != bgDBItor->bgSectionData.end())
                            {
                                if (!bgDBLaneItor->empty())
                                {
                                    sprintf_s(IMAGE_NAME_STR2, MAX_PATH - 1,
                                              "section_%d_lane_%d_merged_%d.png",
                                              _curSecId, laneNum, MERGED_TIMES);
                                    showImage(*bgDBLaneItor, Scalar(0, 0, 0), IMAGE_NAME_STR2);
                                }

                                laneNum++;

                                bgDBLaneItor++;
                            }

                            break;
                        }

                        bgDBItor++;
                    }
#endif

                } // end of valid processing

                laneItor++;
            } // end of lane iterator of each group

            grpItor++;
        } // end of reported data group iterator

        return true;
    }


    bool CRoadVecGen2::stitchSectionLanes()
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
        double theta = 0.0;
        list<backgroundSectionData>::iterator bgSecItor = _bgDatabaseList.begin();
        list<foregroundSectionData>::iterator fgSecItor = _fgDatabaseList.begin();
        while (bgSecItor != _bgDatabaseList.end())
        {
            numOfLanes = 0;

            // none empty sections
            if (!bgSecItor->bgSectionData.empty())
            {
                _curSecId = bgSecItor->sectionId;

                theta = _secRotAngle[_curSecId - 1];

                // number of lanes in current section
                numOfLanes = _secLaneNum[_curSecId - 1];

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
                                point3D_t curPnt = { 0 };
                                vector<point3D_t> leftSample, rightSample;

                                // get paint information
                                int numOfSmpPnts = laneItor->front().size();
                                for (int i = 0; i < numOfSmpPnts; i++)
                                {
                                    curPnt.lon = laneItor->front().at(i).lon;
                                    curPnt.paintFlag = laneItor->front().at(i).paintFlag;
                                    leftSample.push_back(curPnt);

                                    curPnt.lon = laneItor->back().at(i).lon;
                                    curPnt.paintFlag = laneItor->back().at(i).paintFlag;
                                    rightSample.push_back(curPnt);
                                }

                                // suppose there should be 2 lines in each lane
                                polynomialFitting(laneItor->front(), leftSample);
                                polynomialFitting(laneItor->back(), rightSample);

                                dblines.push_back(leftSample);
                                dblines.push_back(rightSample);
                            }

                            laneIndItor++;
                            laneItor++;
                        }

#if VISUALIZATION_ON
                        sprintf_s(IMAGE_NAME_STR2, "fg_section_%d_polyval_sampled.png",
                                  _curSecId);
                        showImage(dblines,  Scalar(0, 0, 255), IMAGE_NAME_STR2);
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
                                    continue;
                                }

                                for (uint32 i = 0; i < lineOne.size(); i++)
                                {
                                    curPnt.lon = lineOne[i].lon;
                                    curPnt.lat = (lineOne[i].lat + lineTwo[i].lat) / 2;
                                    curPnt.paintFlag = lineOne[i].paintFlag; // + lineTwo[i].paintFlag;

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
                            sprintf_s(IMAGE_NAME_STR2, "fg_%d_section_%d_mergedmiddle_lines.png",
                                      FG_MERGED_NUM, _curSecId);
                            showImage(midlines,  Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
#if SAVE_DATA_ON
                            sprintf_s(IMAGE_NAME_STR2, "fg_%d_section_%d_mergedmiddle_lines.txt",
                                      FG_MERGED_NUM, _curSecId);
                            saveListVec(midlines, IMAGE_NAME_STR2);
#endif
                        }

                        // add lines to foreground database
                        if (SINGLE_LANE == numOfValidLanes)
                        {
                            // two lines
                            fgSecItor->fgSectionData.push_back(dblines.front());
                            fgSecItor->fgSectionData.push_back(dblines.back());
                        }
                        else if (DOUBLE_LANE == numOfValidLanes)
                        {
                            // three lines
                            vector<point3D_t> linein;

                            // 1st line
                            for (uint32 i = 0; i < dblines.front().size(); i++)
                            {
                                curPnt.lon = dblines.front().at(i).lon;
                                curPnt.lat = dblines.front().at(i).lat + distlines.front().at(i);
                                curPnt.paintFlag = dblines.front().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            fgSecItor->fgSectionData.push_back(linein);
                            linein.clear();

                            // 2nd line
                            fgSecItor->fgSectionData.push_back(midlines.front());

                            // 3rd line
                            for (uint32 i = 0; i < dblines.back().size(); i++)
                            {
                                curPnt.lon = dblines.back().at(i).lon;
                                curPnt.lat = dblines.back().at(i).lat + distlines.back().at(i);
                                curPnt.paintFlag = dblines.back().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            fgSecItor->fgSectionData.push_back(linein);
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
                            fgSecItor->fgSectionData.push_back(linein);
                            linein.clear();

                            // 2nd line
                            for (uint32 i = 0; i < midlines.front().size(); i++)
                            {
                                curPnt.lon = midlines.front().at(i).lon;
                                curPnt.lat = midlines.front().at(i).lat + d2[i];
                                curPnt.paintFlag = midlines.front().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            fgSecItor->fgSectionData.push_back(linein);
                            linein.clear();

                            // 3rd line
                            for (uint32 i = 0; i < midlines.back().size(); i++)
                            {
                                curPnt.lon = midlines.back().at(i).lon;
                                curPnt.lat = midlines.back().at(i).lat + d1[i];
                                curPnt.paintFlag = midlines.back().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            fgSecItor->fgSectionData.push_back(linein);
                            linein.clear();

                            // 4th line
                            for (uint32 i = 0; i < dblines.back().size(); i++)
                            {
                                curPnt.lon = dblines.back().at(i).lon;
                                curPnt.lat = dblines.back().at(i).lat + d1[i] + d3[i];
                                curPnt.paintFlag = dblines.back().at(i).paintFlag;

                                linein.push_back(curPnt);
                            }
                            fgSecItor->fgSectionData.push_back(linein);
                            linein.clear();
                        } // 3 lanes

#if VISUALIZATION_ON
                        sprintf_s(IMAGE_NAME_STR2, "fg_section_%d_rotated_lines.png",
                                  _curSecId);
                        showImage(fgSecItor->fgSectionData, Scalar(0, 0, 255), IMAGE_NAME_STR2);
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
            sprintf_s(IMAGE_NAME_STR2, MAX_PATH - 1, "fg_section_%d_merged_%d.txt",
                fgSecItor->sectionId, FG_MERGED_NUM);
            saveListVec(fgSecItor->fgSectionData, IMAGE_NAME_STR2);
#endif

            fgSecItor++;
            bgSecItor++;
        } // end of section iteration

        return true;
    }


    void CRoadVecGen2::removeOverlap(/*OUT list<list<vector<point3D_t>>> &fgData*/)
    {
        if (!_fgDatabaseList.empty())
        {
            _fgOutputList.clear();

            int numOfFGLines = 0, ind = 0;
            double *py11 = nullptr, *py12 = nullptr,
                   *py21 = nullptr, *py22 = nullptr;
            double step = 0.0, stx = 0.0, edx = 0.0;
            double my1 = 0.0, my2 = 0.0, sy1 = 0.0, sy2 = 0.0;

            vector<point3D_t> line1, line2, fgl, rotline;
            list<vector<point3D_t>> fglines;

            list<vector<point3D_t>>::iterator lItor;
            list<foregroundSectionData>::iterator fgSecItor = _fgDatabaseList.begin();

            // remove overlap for each section
            while (fgSecItor != _fgDatabaseList.end())
            {
                fglines.clear();

                if (fgSecItor->fgSectionData.empty())
                {
                    //fgData.push_back(fglines);

                    foregroundSectionData fgSecData;
                    fgSecData.sectionId = fgSecItor->sectionId;
                    _fgOutputList.push_back(fgSecData);

                    fgSecItor++;
                    continue;
                }

                // current section Id
                _curSecId = fgSecItor->sectionId;

                numOfFGLines = fgSecItor->fgSectionData.size();
                py11 = new double[numOfFGLines];
                py12 = new double[numOfFGLines];
                py21 = new double[numOfFGLines];
                py22 = new double[numOfFGLines];

                // adjacent Y value of two body start/end points
                if (py11 && py12 && py21 && py22)
                {
                    ind = 0;
                    my1 = 0.0; my2 = 0.0; sy1 = 0.0; sy2 = 0.0;

                    lItor = fgSecItor->fgSectionData.begin();

                    step = lItor->at(1).lon - lItor->at(0).lon;
                    stx  = lItor->at(_secBodyStInd[_curSecId - 1]).lon;
                    edx  = lItor->at(_secBodyEdInd[_curSecId - 1]).lon;

                    while (lItor != fgSecItor->fgSectionData.end() && !lItor->empty())
                    {
                        py11[ind] = lItor->at(_secBodyStInd[_curSecId - 1]).lat;
                        py12[ind] = lItor->at(_secBodyStInd[_curSecId - 1] + 1).lat;
                        my1 += (py11[ind] - py12[ind]);
                        sy1 += py11[ind];

                        py21[ind] = lItor->at(_secBodyEdInd[_curSecId - 1] - 1).lat;
                        py22[ind] = lItor->at(_secBodyEdInd[_curSecId - 1]).lat;
                        my2 += (py21[ind] - py22[ind]);
                        sy2 += py21[ind];

                        ind++;
                        lItor++;
                    }

                    double c1 = sy1 / numOfFGLines;
                    double c2 = sy2 / numOfFGLines;

                    double angle1 = atan(my1 / (numOfFGLines * step));
                    double angle2 = atan(my2 / (numOfFGLines * step));

                    // start point
                    complex<double> theta1(0, angle1);
                    complex<double> yj1(0, c1);
                    double cox1 = real((stx + yj1) * exp(theta1));

                    // end point
                    complex<double> theta2(0, angle2);
                    complex<double> yj2(0, c2);
                    double cox2 = real((edx + yj2) * exp(theta2));


                    foregroundSectionData fgSecData;
                    fgSecData.sectionId = _curSecId;


                    lItor = fgSecItor->fgSectionData.begin();
                    while (lItor != fgSecItor->fgSectionData.end() && !lItor->empty())
                    {
                        line1.clear();
                        line2.clear();
                        fgl.clear();
                        rotline.clear();

                        lineRotation(*lItor, angle1, line1);
                        lineRotation(*lItor, angle2, line2);

                        if (step > 0)
                        {
                            for (uint32 i = 0; i < line1.size(); i++)
                            {
                                if (cox1 <= line1[i].lon && line2[i].lon <= cox2)
                                {
                                    fgl.push_back(lItor->at(i));
                                }
                            }
                        }
                        else
                        {
                            for (uint32 i = 0; i < line1.size(); i++)
                            {
                                if (cox2 <= line2[i].lon && line1[i].lon <= cox1)
                                {
                                    fgl.push_back(lItor->at(i));
                                }
                            }
                        }

                        getFGLine(fgl, _secRotAngle[_curSecId - 1], rotline);
                        fglines.push_back(rotline);


                        fgSecData.fgSectionData.push_back(rotline);


                        lItor++;
                    }

                    //fgData.push_back(fglines);
                    _fgOutputList.push_back(fgSecData);

#if VISUALIZATION_ON
                    sprintf_s(IMAGE_NAME_STR2, "fg_section_%d_output_lines.png",
                              _curSecId);
                    showImage(fglines, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
                }

                SAFEARR_DELETE(py11);
                SAFEARR_DELETE(py12);
                SAFEARR_DELETE(py21);
                SAFEARR_DELETE(py22);

                fgSecItor++;
            } // end of section iterator
        } // end of foreground database not empty
    }


    void CRoadVecGen2::jointProcessing(OUT list<list<vector<point3D_t>>> &fgData)
    {
        // if output database is empty, clear output data
        if (_fgOutputList.empty())
        {
            fgData.clear();
            return;
        }

        fgData.clear();

        // number of sections
        int numOfSegs = _fgOutputList.size();

        // iterate each adjacent sections to join together
        list<foregroundSectionData>::iterator fgSecItor = _fgOutputList.begin();
        foregroundSectionData currData, prevData;

        for (int i = 0; i < numOfSegs; i++)
        {
            prevData = *fgSecItor;
            // if current is the first section and road is circle, previous
            // section is the first one
            if (i == numOfSegs - 1)
            {
                currData = _fgOutputList.front();
            }
            else
            {
                fgSecItor++;
                currData = *fgSecItor;
            }

            // current section Id
            _curSecId = currData.sectionId;

            // current section data is empty, push empty lines to match lane number
            if (currData.fgSectionData.empty())
            {
                list<vector<point3D_t>> fglines;
                for (int i = 0; i <= _secLaneNum[_curSecId - 1]; i++)
                {
                    vector<point3D_t> line;
                    fglines.push_back(line);
                }

                fgData.push_back(fglines);
                continue;
            }

            // previous section data is empty
            if (prevData.fgSectionData.empty())
            {
                fgData.push_back(currData.fgSectionData);
                continue;
            }

            // both current and previous section data is not empty,
            // number of lines in current and previous sections
            int currlines = currData.fgSectionData.size();
            int prevlines = prevData.fgSectionData.size();

            // line data pointers
            vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
            vector<point3D_t> **ppPrevLines = new vector<point3D_t> *[prevlines];
            if (*ppCurrLines && *ppPrevLines)
            {
                list<vector<point3D_t>>::iterator lineItor = currData.fgSectionData.begin();

                // current section
                for (int i = 0; i < currlines; i++)
                {
                    ppCurrLines[i] = &(*lineItor);
                    lineItor++;
                }

                // previous section
                lineItor = prevData.fgSectionData.begin();
                for (int i = 0; i < prevlines; i++)
                {
                    ppPrevLines[i] = &(*lineItor);
                    lineItor++;
                }

                // match to correct lines, get matched end points
                int numOfLines = min(currlines, prevlines);
                vector<int> index;
                vector<point3D_t> stPnts;
                vector<point3D_t> edPnts;

                for (int i = 0; i < numOfLines; i++)
                {
                    if (ppCurrLines[i] && ppPrevLines[i])
                    {
                        index.push_back(i);
                        stPnts.push_back(ppCurrLines[i]->front());
                        edPnts.push_back(ppPrevLines[i]->back());
                    }
                }

                if (!stPnts.empty() && !edPnts.empty())
                {
                    for (uint32 i = 0; i < index.size(); i++)
                    {
                        double dx = edPnts[i].lon - stPnts[i].lon;
                        double dy = edPnts[i].lat - stPnts[i].lat;

                        int numOfHalfPnts = ppCurrLines[index[i]]->size() / 2;

                        double ddx = dx / numOfHalfPnts;
                        double ddy = dx / numOfHalfPnts;

                        for (int pi = 0; pi < numOfHalfPnts; pi++)
                        {
                            ppCurrLines[index[i]]->at(pi).lon += (numOfHalfPnts - pi) * ddx;
                            ppCurrLines[index[i]]->at(pi).lat += (numOfHalfPnts - pi) * ddy;
                        }
                    }
                }

                // push current section lines to fgData
                list<vector<point3D_t>> fglines;
                for (int i = 0; i < currlines; i++)
                {
                    fglines.push_back(*(ppCurrLines[i]));
                }

                fgData.push_back(fglines);
            }
        }
    }


    void CRoadVecGen2::getDatabaseData(OUT backgroundSectionData **bgSegData)
    {
        if (_segConfigList.empty() || _bgDatabaseList.empty() || nullptr == bgSegData)
        {
            return;
        }

        // get background database data
        list<backgroundSectionData>::iterator dbItor = _bgDatabaseList.begin();
        while (dbItor != _bgDatabaseList.end())
        {
            if (_curSecId == dbItor->sectionId)
            {
                *bgSegData = &(*dbItor);
                break;
            }
            dbItor++;
        }

        return;
    }


    void CRoadVecGen2::dotLineBlockIndex(IN  vector<point3D_t> &lineData,
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

            dd.push_back(abs(dx[i - 1] + dy[i - 1]));
            if (DASH_DIST_DIFF < dd[i - 1])
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

        // combine break points
        // blockCombine(dotBlkIndexSt, dotBlkIndexEd);
    }


    void CRoadVecGen2::blockCombine(INOUT vector<int> &dotBlkIndexSt,
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


    void CRoadVecGen2::polynomialFitting(IN    vector<point3D_t> &sourceLine,
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


    void CRoadVecGen2::getLinePaintInfo(IN  vector<point3D_t> &sourceline,
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

    void CRoadVecGen2::getFGLine(IN  vector<point3D_t> &bgline,
                                 double                 theta,
                                 OUT vector<point3D_t> &fgline)
    {
        if (bgline.empty())
        {
            fgline.clear();
            return;
        }

        // rotate back to generate foreground line data
        lineRotation(bgline, theta, fgline);

        if (fgline.empty())
        {
            return;
        }

        // get max paint value of line
        float maxPaint = fgline[0].paintFlag;
        for (uint32 i = 1; i < fgline.size(); i++)
        {
            if (fgline[i].paintFlag > maxPaint)
            {
                maxPaint = fgline[i].paintFlag;
            }
        }

        // normalize paint information
        for (uint32 i = 1; i < fgline.size(); i++)
        {
            fgline[i].paintFlag /= maxPaint;
        }
    }


    bool CRoadVecGen2::LaneDataPreprocessing(IN  list<vector<point3D_t>> &lanelines,
                                             OUT int                     &matchedLane,
                                             OUT vector<point3D_t>       &leftline,
                                             OUT vector<point3D_t>       &rightline)
    {
        // check input parameters
        if (lanelines.empty() ||
            leftline.empty() || rightline.empty() ||
            (leftline.size() != rightline.size()))
        {
            return false;
        }

        // number of sample line points
        int numOfPnts = leftline.size();

        // rotate the input two lines first
        vector<point3D_t> leftRotated, leftRotatedValid;
        vector<point3D_t> rightRotated, rightRotatedValid;

        // current section rotation angle
        double theta = _secRotAngle[_curSecId - 1];
        lineRotation(lanelines.front(), -theta, leftRotated); // left line
        lineRotation(lanelines.back(), -theta, rightRotated); // right line

        // get paint information for re-sample lines
        getLinePaintInfo(leftRotated, leftline);
        getLinePaintInfo(rightRotated, rightline);

        // lane number estimation
        laneNumberEst(leftline, rightline, matchedLane);

#if VISUALIZATION_ON
        list<vector<point3D_t>> rotated;
        rotated.push_back(leftRotated);
        rotated.push_back(rightRotated);
        sprintf_s(IMAGE_NAME_STR2, "section_%d_lane_%d_merging_%d_rotated.png",
            _curSecId, matchedLane, MERGED_TIMES);
        showImage(rotated, Scalar(0, 255, 0), IMAGE_NAME_STR2);
#endif

        if (INVALID_LANE_NUM == matchedLane)
        {
            return false;
        }

#if VISUALIZATION_ON
        sprintf_s(IMAGE_NAME_STR2, "fg_%d_section_%d_lane_%d_merging_%d_source.png",
                  FG_MERGED_NUM, _curSecId, matchedLane, MERGED_TIMES);
        showImage(lanelines, Scalar(0, 255, 0), IMAGE_NAME_STR2);
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

        // data fitting or interpolation re-sample
        vector<point3D_t> leftpola = leftline, leftpoly = leftline,
                          rightpola = rightline, rightpoly = rightline;

        interpolationSample(leftRotatedValid,  leftpola);
        interpolationSample(rightRotatedValid, rightpola);

        polynomialFitting(leftRotatedValid,  leftpoly);
        polynomialFitting(rightRotatedValid, rightpoly);

#if VISUALIZATION_ON
        list<vector<point3D_t>> intersampled;
        intersampled.push_back(leftpola);
        intersampled.push_back(rightpola);
        sprintf_s(IMAGE_NAME_STR2, "section_%d_lane_%d_merging_%d_interpolate_sample.png",
                  _curSecId, matchedLane, MERGED_TIMES);
        showImage(intersampled, Scalar(0, 0, 255), IMAGE_NAME_STR2);

        list<vector<point3D_t>> polysampled;
        polysampled.push_back(leftpoly);
        polysampled.push_back(rightpoly);
        sprintf_s(IMAGE_NAME_STR2, "section_%d_lane_%d_merging_%d_polyval_sample.png",
                  _curSecId, matchedLane, MERGED_TIMES);
        showImage(polysampled, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif

        // according to mean and standard derivation to choose interpolation or
        // polynomial fitting for input data
        vector<double> dy0, dy1;
        double meanY0 = 0.0, stdY0 = 0.0, meanY1 = 0.0, stdY1 = 0.0, tmp = 0.0;
        for (int i = 0; i < numOfPnts; i++)
        {
            // interpolation
            tmp = leftpola[i].lat - rightpola[i].lat;
            dy0.push_back(tmp);
            meanY0 += tmp;

            // polynomial fitting
            tmp = leftpoly[i].lat - rightpoly[i].lat;
            dy1.push_back(tmp);
            meanY1 += tmp;
        }

        // mean and standard derivation
        meanY0 /= numOfPnts;
        meanY1 /= numOfPnts;
        for (int i = 0; i < numOfPnts; i++)
        {
            stdY0 += (dy0[i] - meanY0) * (dy0[i] - meanY0);
            stdY1 += (dy1[i] - meanY1) * (dy1[i] - meanY1);
        }
        stdY0 = sqrt(stdY0 / numOfPnts);
        stdY1 = sqrt(stdY1 / numOfPnts);

        // which re-sample method to use
        bool cond00 = 3.0 < abs(meanY0) && abs(meanY0) < 6.0;
        bool cond01 = stdY0 < 3;
        bool cond10 = 3.0 < abs(meanY1) && abs(meanY1) < 6.0;
        bool cond11 = stdY1 < 3;

        RESAMPLE_METHOD smpltype = USE_INTERPOLATION;
        if (cond00 && cond01 && cond10 && cond11)
        {
            if (stdY0 < stdY1)
            {
                // interpolation
                smpltype = USE_INTERPOLATION;
            }
            else
            {
                // polynomial
                smpltype = USE_POLYNOMIALFIT;
            }
        }
        else if (cond00 && cond01)
        {
            // interpolation
            smpltype = USE_INTERPOLATION;
        }
        else if (cond10 && cond11)
        {
            // polynomial
            smpltype = USE_POLYNOMIALFIT;
        }
        else
        {
#if VISUALIZATION_ON || SAVE_DATA_ON
            printf("section %d matchedLane %d merging %d is not valid\n",
                   _curSecId, matchedLane, MERGED_TIMES);
#endif
            return false;
        }

        if (USE_INTERPOLATION == smpltype)
        {
            interpolationSample(leftRotatedValid,  leftline);
            interpolationSample(rightRotatedValid, rightline);
#if VISUALIZATION_ON
            list<vector<point3D_t>> intersampled_;
            intersampled_.push_back(leftpola);
            intersampled_.push_back(rightpola);
            sprintf_s(IMAGE_NAME_STR2, "section_%d_lane_%d_merging_%d_interpolate_sample_.png",
                _curSecId, matchedLane, MERGED_TIMES);
            showImage(intersampled_, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
        }
        else
        {
            polynomialFitting(leftRotatedValid,  leftline);
            polynomialFitting(rightRotatedValid, rightline);
#if VISUALIZATION_ON
            list<vector<point3D_t>> polysampled_;
            polysampled_.push_back(leftpoly);
            polysampled_.push_back(rightpoly);
            sprintf_s(IMAGE_NAME_STR2, "section_%d_lane_%d_merging_%d_polyval_sample_.png",
                _curSecId, matchedLane, MERGED_TIMES);
            showImage(polysampled_, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
        }

        return true;
    }


    void CRoadVecGen2::laneNumberEst(IN  vector<point3D_t> &leftline,
                                     IN  vector<point3D_t> &rightline,
                                     OUT int32             &matchedLane)
    {
        // checkout input data
        if (leftline.empty() || rightline.empty())
        {
            return;
        }

        // get dash line start / end index for left and right lines
        vector<int> stInd, edInd;
        int numOfBlks = 0, indSt = 0, indEd = 0;
        double mL = 0.0, sL = 0.0, mR = 0.0, sR = 0.0, tmp = 0.0;
        vector<double> lenL, lenR;

        // left one
        dotLineBlockIndex(leftline, stInd, edInd);

        // get line data with section body
        numOfBlks = stInd.size();
        for (int i = 0; i < numOfBlks; i++)
        {
            // max of body start index
            indSt = max(stInd[i], _secBodyStInd[_curSecId - 1]);
            // min of body end index
            indEd = min(edInd[i], _secBodyEdInd[_curSecId - 1]);

            if ((indSt < indEd) &&
                (indSt >= _secBodyStInd[_curSecId - 1]) &&
                (indEd <= _secBodyEdInd[_curSecId - 1]))
            {
                tmp = abs(indEd - indSt);
                lenL.push_back(tmp);
                mL += tmp;
            }
        }

        // right one
        stInd.clear();
        edInd.clear();
        dotLineBlockIndex(rightline, stInd, edInd);

        // get line data with section body
        numOfBlks = stInd.size();
        for (int i = 0; i < numOfBlks; i++)
        {
            // max of body start index
            indSt = max(stInd[i], _secBodyStInd[_curSecId - 1]);
            // min of body end index
            indEd = min(edInd[i], _secBodyEdInd[_curSecId - 1]);

            if ((indSt < indEd) &&
                (indSt >= _secBodyStInd[_curSecId - 1]) &&
                (indEd <= _secBodyEdInd[_curSecId - 1]))
            {
                tmp = abs(indEd - indSt);
                lenR.push_back(tmp);
                mR += tmp;
            }
        }

        if (lenL.empty() || lenR.empty())
        {
            matchedLane = INVALID_LANE_NUM;
        }

        // mean length and standard derivation
        mL = mL / lenL.size();
        mR = mR / lenR.size();
        for (uint32 i = 0; i < lenL.size(); i++)
        {
            sL += (lenL[i] - mL) * (lenL[i] - mL);
        }
        sL = sqrt(sL / lenL.size());

        for (uint32 i = 0; i < lenR.size(); i++)
        {
            sR += (lenR[i] - mR) * (lenR[i] - mR);
        }
        sR = sqrt(sR / lenR.size());

        // estimation value
        double LL = mL + sL;
        double RR = mR + sR;

        // lane type estimation
        double threshold =  LINE_TYPE_TH; // LINE_TYPE_THRESHOLD[_curSecId];
        int lanetype = SOLID_SOLID;
        if (LL > RR)
        {
            if (LL / RR < 2)
            {
                if (RR > threshold)
                {
                    lanetype = SOLID_SOLID;
                }
                else
                {
                    lanetype = DASH_DASH;
                }
            }
            else if (LL < threshold)
            {
                lanetype = DASH_DASH;
            }
            else if (RR > 2 * threshold)
            {
                lanetype = SOLID_SOLID;
            }
            else
            {
                lanetype = SOLID_DASH;
            }
        }
        else
        {
            if (RR / LL < 2)
            {
                if (LL > threshold)
                {
                    lanetype = SOLID_SOLID;
                }
                else
                {
                    lanetype = DASH_DASH;
                }
            }
            else if (RR < threshold)
            {
                lanetype = DASH_DASH;
            }
            else if (LL > 2 * threshold)
            {
                lanetype = SOLID_SOLID;
            }
            else
            {
                lanetype = DASH_SOLID;
            }
        }

        // matched lane number
        int numOfLanes = _secLaneNum[_curSecId - 1];
        if (SINGLE_LANE == numOfLanes)
        {
            matchedLane = SOLID_SOLID;
        }
        if (DOUBLE_LANE == numOfLanes)
        {
            matchedLane = (SOLID_DASH == lanetype) ? SOLID_DASH : DOUBLE_LANE - 1;
        }
        if (MAX_SUPPORTED_LANES == numOfLanes)
        {
            matchedLane = lanetype;
        }
    }


} // end of namespace ns_database


