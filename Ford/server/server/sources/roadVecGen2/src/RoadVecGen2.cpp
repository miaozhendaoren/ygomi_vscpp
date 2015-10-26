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
#include "configure.h"

#if 1 // VISUALIZATION_ON || SAVE_DATA_ON
#include "VisualizationApis.h"

extern uint32 PREVIOUS_SEGID;
extern uint32 FG_MERGED_NUM;

extern uint32 MERGED_TIMES[100];
extern char IMAGE_NAME_STR2[MAX_PATH];

using namespace std;
#endif


namespace ns_database
{
    // number of section configuration points
#define SEG_CFG_PNT_NUM           4

#define MINDIST                   10

#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
#define LINE_TYPE_TH              46
#else
#define LINE_TYPE_TH              40
#endif

#define SAMPLE_SPACE              0.1
#define SINGLE_LANE               1
#define DOUBLE_LANE               2
#define TRIPLE_LANE               3
#define MAX_SUPPORTED_LANES       4
#define INVALID_LANE_FLAG        -1
#define VALID_LANE_FLAG           1
#define DASH_DIST_DIFF            2.5
#define INVALID_LANE_NUM         -1
#define SOLID_DASH                0
#define DASH_DASH                 1
#define DASH_SOLID                2
#define SOLID_SOLID               0
#define MATCHED_LINE_FLAG         1
#define UNMATCHED_LINE_FLAG       -1

#define SAFEARR_DELETE(p) if (p) { delete [] (p); p = nullptr; }

enum RESAMPLE_METHOD
{
    USE_INTERPOLATION = 0,
    USE_POLYNOMIALFIT = 1,
};

CRoadVecGen2::CRoadVecGen2(void)
{
    _configPath = "";
    _curSecId = 0;

    _bHasRevDirData = false;
    _bHasChanged = false;

    // create mutex
    _hMutexMerging = CreateMutex(NULL, FALSE, NULL);
    ReleaseMutex(_hMutexMerging);
}


CRoadVecGen2::CRoadVecGen2(string configFilePath)
{
    _configPath = configFilePath;

    _bHasRevDirData = false;
    _bHasChanged = false;

    // read section configuration file
    list<segAttributes_t> segConfigList;
    readSecConfig(segConfigList);

    // initialize database
    initDatabase();

    // calculate section rotation angle and X data range
    calcRotAngleAndRange();

    // check whether road is a circle
    checkCircleRoad();

    // create mutex
    _hMutexMerging = CreateMutex(NULL, FALSE, NULL);
    ReleaseMutex(_hMutexMerging);
}


CRoadVecGen2::~CRoadVecGen2(void)
{
    _configPath = "";

    // release list or vector data
    _secLaneNum.clear();
    _secRotAngle.clear();
    _secBodyStInd.clear();
    _secBodyEdInd.clear();
    _secLeftData.clear();
    _secRightData.clear();
    _secLaneType.clear();
    _secLaneConn.clear();
    _segConfigList.clear();
    _bgDatabaseList.clear();
    _bgRevDirList.clear();
    _fgDatabaseList.clear();
    _fgRevDirList.clear();
    _fgAllDirList.clear();
    _fgOutputList.clear();

    // close mutex
    CloseHandle(_hMutexMerging);
}


bool CRoadVecGen2::roadSectionsGen(IN  list<list<vector<point3D_t>>> &rptData,
    IN list<vector<point3D_t>> &gpsData,
    OUT list<list<vector<point3D_t>>> &fgData,
    OUT list<uint32> &modifiedSectionId)
{
    if (rptData.empty())
    {
        return false;
    }

    // release data first
    if (!fgData.empty())
    {
        fgData.clear();
    }

    // pre-processing reported lane data
    // preprocessRptData(rptData, gpsData);

    // suppose output is list<reportSectionData> rptData;
    list<reportSectionData> secData;

    // section partition, input is rptData
    if(!_segConfigList.empty())
    {
        uint32 sampleInterval = 20;
        _secRptDataObj.segMultiRptData(rptData, sampleInterval, _segConfigList, secData);
    }
#if 1 // SAVE_DATA_ON
    //saveData(rptData);
    //saveData(secData);
    saveData(rptData, gpsData);

    FG_MERGED_NUM++;

    return false;
#endif

    WaitForSingleObject(_hMutexMerging, INFINITE);

    bool bCommLinesMerged = false;
#if defined(_DE_LEHRE_VIDEO)
    bCommLinesMerged = true;
#endif

    // iterate each section
    list<reportSectionData>::iterator secItor = secData.begin();
    while (secItor != secData.end())
    {
        // current reported section ID
        _curSecId = secItor->sectionId;

        modifiedSectionId.push_back(_curSecId);

        // merge new data with database
        mergeSectionLane(*secItor);

        // stitch background database to generate foreground data
        stitchSectionLanes();
        stitchSectionLanes(true);

        // merge two direction shared line
        stitchSharedLines(bCommLinesMerged);

        // remove section overlap
        removeOverlap();

        secItor++;
    }

    jointProcessing(fgData, bCommLinesMerged);

#if VISUALIZATION_ON || SAVE_DATA_ON
    saveData(fgData, true);
#endif // end of visualization or save data

    ReleaseMutex(_hMutexMerging);

    return true;
}


bool CRoadVecGen2::roadSectionsGen(OUT list<list<vector<point3D_t>>> &fgData)
{
    WaitForSingleObject(_hMutexMerging, INFINITE);

    bool bCommLinesMerged = false;
#if defined(_DE_LEHRE_VIDEO)
    bCommLinesMerged = true;
#endif

    // generate foreground database data
    list<backgroundSectionData>::iterator bgSecItor = _bgDatabaseList.begin();
    while (bgSecItor != _bgDatabaseList.end())
    {
        _curSecId = bgSecItor->sectionId;

        stitchSectionLanes();
        stitchSectionLanes(true);

        // merge two direction shared line
        stitchSharedLines(bCommLinesMerged);

        // remove section overlap and output data
        removeOverlap();

        bgSecItor++;
    }

    jointProcessing(fgData, bCommLinesMerged);

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

    // check whether road is a circle
    checkCircleRoad();
}

bool CRoadVecGen2::loadDefaultSegData(IN uint32 segId, IN string filename)
{
    // load default background data for specified section
    FILE *fp = nullptr;
    errno_t err = fopen_s(&fp, filename.c_str(), "r");
    if (0 == err)
    {
        WaitForSingleObject(_hMutexMerging, INFINITE);

        backgroundSectionData *bgSegData = nullptr;

        // get background database pointer
        _curSecId = segId;
        getBgDatabaseData(&bgSegData);
        bgSegData->bgSectionData.clear();

        vector<point3D_t> leftline, rightline;
        point3D_t leftpnt = { 0 }, rightpnt = { 0 };

        while (!feof(fp))
        {
            fscanf_s(fp, "%lf, %lf, %lf, %lf, %lf, %lf\n",
                &leftpnt.lon, &leftpnt.lat, &leftpnt.paintFlag,
                &rightpnt.lon, &rightpnt.lat, &rightpnt.paintFlag);

            leftpnt.paintFlag = 1;
            leftpnt.count = 1;
            rightpnt.paintFlag = 1;
            rightpnt.count = 1;

            leftline.push_back(leftpnt);
            rightline.push_back(rightpnt);
        }
        list<vector<point3D_t>> lane;
        lane.push_back(leftline);
        lane.push_back(rightline);

        bgSegData->bgSectionData.push_back(lane);

        ReleaseMutex(_hMutexMerging);
        fclose(fp);

        return true;
    }

    return false;
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

    list<backgroundSectionData>::iterator bgRevDirItor = _bgRevDirList.begin();
    while (bgRevDirItor != _bgRevDirList.end())
    {
        bgRevDirItor->bgSectionData.clear();

        bgRevDirItor++;
    }

    list<foregroundSectionData>::iterator fgItor = _fgDatabaseList.begin();
    while (fgItor != _fgDatabaseList.end())
    {
        fgItor->fgSectionData.clear();

        fgItor++;
    }

    list<foregroundSectionData>::iterator fgRevDirItor = _fgRevDirList.begin();
    while (fgRevDirItor != _fgRevDirList.end())
    {
        fgRevDirItor->fgSectionData.clear();

        fgRevDirItor++;
    }

    list<foregroundSectionData>::iterator fgAllDirItor = _fgAllDirList.begin();
    while (fgAllDirItor != _fgAllDirList.end())
    {
        fgAllDirItor->fgSectionData.clear();

        fgAllDirItor++;
    }

    list<foregroundSectionData>::iterator fgOutItor = _fgOutputList.begin();
    while (fgOutItor != _fgOutputList.end())
    {
        fgOutItor->fgSectionData.clear();

        fgOutItor++;
    }

    ReleaseMutex(_hMutexMerging);
}

void CRoadVecGen2::getBgRoadVec(OUT list<backgroundSectionData> &bgVecOut,
    IN bool bRevDir/* = false*/)
{
    WaitForSingleObject(_hMutexMerging, INFINITE);
    if (bRevDir)
    {
        bgVecOut = _bgRevDirList;
    }
    else
    {
        bgVecOut = _bgDatabaseList;
    }

    ReleaseMutex(_hMutexMerging);
}

void CRoadVecGen2::setBgRoadVec(IN list<backgroundSectionData> &bgVecIn,
    IN bool bRevDir/* = false*/)
{
    WaitForSingleObject(_hMutexMerging, INFINITE);

    if (bRevDir)
    {
        _bgRevDirList.clear();
        _bgRevDirList = bgVecIn;
    }
    else
    {
        _bgDatabaseList.clear();
        _bgDatabaseList = bgVecIn;
    }

    ReleaseMutex(_hMutexMerging);
}

void CRoadVecGen2::readSecConfig(OUT list<segAttributes_t> &segCfgList)
{
    int sectionNum = 0;
    int sectionID  = 0;
    int laneNum = 0;
    int a = 0, b = 0, c = 0, d = 0, e = 0;

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

    memset(segmentElement.ports, 0, MAX_NUM_PORT * sizeof(point3D_t));

    FILE *fp = nullptr;
    errno_t err = fopen_s(&fp, _configPath.c_str(), "rt");
    if(0 != err)
    {
        return;
    }

    while (!feof(fp))
    {
        fscanf_s(fp, "segId: %d,laneNum: %d\n", &sectionID, &laneNum);
        segmentElement.segId_used = 1;
        segmentElement.segId      = sectionID;
        segmentElement.uiLaneNum_used = 1;
        segmentElement.uiLaneNum      = laneNum;

        vector<int> laneType, laneConn;
        switch (laneNum)
        {
        case 1:
            fscanf_s(fp, "lineType: %d,%d\n", &a, &b);
            laneType.push_back(a);
            laneType.push_back(b);
            fscanf_s(fp, "connectInfo: %d\n", &a);
            laneConn.push_back(a);
            break;
        case 2:
            fscanf_s(fp, "lineType: %d,%d,%d\n", &a, &b, &c);
            laneType.push_back(a);
            laneType.push_back(b);
            laneType.push_back(c);
            fscanf_s(fp, "connectInfo: %d,%d\n", &a, &b);
            laneConn.push_back(a);
            laneConn.push_back(b);
            break;
        case 3:
            fscanf_s(fp, "lineType: %d,%d,%d,%d\n", &a, &b, &c, &d);
            laneType.push_back(a);
            laneType.push_back(b);
            laneType.push_back(c);
            laneType.push_back(d);
            fscanf_s(fp, "connectInfo: %d,%d,%d\n", &a, &b, &c);
            laneConn.push_back(a);
            laneConn.push_back(b);
            laneConn.push_back(c);
            break;
        case 4:
            fscanf_s(fp, "lineType: %d,%d,%d,%d,%d\n", &a, &b, &c, &d, &e);
            laneType.push_back(a);
            laneType.push_back(b);
            laneType.push_back(c);
            laneType.push_back(d);
            laneType.push_back(e);
            fscanf_s(fp, "connectInfo: %d,%d,%d,%d\n", &a, &b, &c, &d);
            laneConn.push_back(a);
            laneConn.push_back(b);
            laneConn.push_back(c);
            laneConn.push_back(d);
            break;
        }

        fscanf_s(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
            &segmentElement.ports[0].lon, &segmentElement.ports[0].lat,
            &segmentElement.ports[1].lon, &segmentElement.ports[1].lat,
            &segmentElement.ports[2].lon, &segmentElement.ports[2].lat,
            &segmentElement.ports[3].lon, &segmentElement.ports[3].lat);

        _secLaneNum.push_back(laneNum);
        _segConfigList.push_back(segmentElement);
        _secLaneType.push_back(laneType);
        _secLaneConn.push_back(laneConn);
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
        _bgRevDirList.push_back(bgData);

        foregroundSectionData fgData;
        fgData.sectionId = configItor->segId;
        _fgDatabaseList.push_back(fgData);
        _fgRevDirList.push_back(fgData);
        _fgAllDirList.push_back(fgData);
        _fgOutputList.push_back(fgData);

        configItor++;
    }
}


void CRoadVecGen2::interpolationSample(IN    vector<point3D_t> &sourceLine,
    INOUT vector<point3D_t> &sampledLine)
{
    if(sourceLine.empty() || sampledLine.empty())
    {
#if VISUALIZATION_ON
        printf("ERROR: interpolationSample : Input empty. \n");
#endif
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
        pointZ.count     = sourceLine[i].count;
        pointZ.paintLength = sourceLine[i].paintLength;

        rotatedLine.push_back(pointZ);
    }
}


bool CRoadVecGen2::mergeSectionLane(IN    reportSectionData     &reportData)
{
    // check section Id first
    if (_curSecId != reportData.sectionId)
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
    list<rptSecData_t>::iterator grpItor;

    // iterate for each group data
    grpItor = reportData.rptSecData.begin();
    while (grpItor != reportData.rptSecData.end())
    {
#if VISUALIZATION_ON
        MERGED_TIMES[_curSecId - 1]++;
#endif

        if (!grpItor->rptLaneData.empty())
        {
            // reset Y and paint for sample vector
            for (int i = 0; i < numOfSplPnts; i++)
            {
                leftSample->at(i).lat          = 0.0;
                leftSample->at(i).paintFlag    = 0.0;
                leftSample->at(i).paintLength  = 0.0;
                rightSample->at(i).lat         = 0.0;
                rightSample->at(i).paintFlag   = 0.0;
                rightSample->at(i).paintLength = 0.0;
            }

            // data preprocessing
            bValid = LaneDataPreprocessing(grpItor->rptLaneData, matchedLane,
                *leftSample, *rightSample);

            // if valid, merge with database lane data
            if (bValid)
            {
                // get background database data or reversed database data
                // according to direction flag
                backgroundSectionData *bgDatabaseData = nullptr;
                getBgDatabaseData(&bgDatabaseData, grpItor->revDirFlag);

                if (!_bHasChanged && grpItor->revDirFlag)
                {
                    _bHasRevDirData = true;
                    _bHasChanged = true;
                }

                // if it is empty, add it directly. otherwise merge with it
                if (bgDatabaseData->bgSectionData.empty())
                {
                    for (uint32 i = 0; i < numOfLanes; i++)
                    {
                        if (i == matchedLane)
                        {
                            // increase merged times, just use the first point
                            leftSample->at(0).count++;
                            rightSample->at(0).count++;

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
                                // increase merged times, just use the first point
                                leftSample->at(0).count++;
                                rightSample->at(0).count++;

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
                                    float weight = (float)((0.0 < bgLaneItor->front().at(i).paintLength && 0.0 < leftSample->at(i).paintLength) ? 0.5 : 1.0);
                                    bgLaneItor->front().at(i).lat = (0.75 * bgLaneItor->front().at(i).lat + 0.25 * leftSample->at(i).lat);
                                    bgLaneItor->front().at(i).paintFlag = (float)(bgLaneItor->front().at(i).paintFlag + leftSample->at(i).paintFlag);
                                    bgLaneItor->front().at(i).paintLength = weight * bgLaneItor->front().at(i).paintLength + 
                                        weight * leftSample->at(i).paintLength;

                                    weight = (float)((0.0 < bgLaneItor->back().at(i).paintLength && 0.0 < rightSample->at(i).paintLength) ? 0.5 : 1.0);
                                    bgLaneItor->back().at(i).lat = (0.75 * bgLaneItor->back().at(i).lat + 0.25 * rightSample->at(i).lat);
                                    bgLaneItor->back().at(i).paintFlag = (float)(bgLaneItor->back().at(i).paintFlag + rightSample->at(i).paintFlag);
                                    bgLaneItor->back().at(i).paintLength = weight * bgLaneItor->back().at(i).paintLength + 
                                        weight * rightSample->at(i).paintLength;
                                }

                                // increase merged times, just use the first point
                                bgLaneItor->front().at(0).count++;
                                bgLaneItor->back().at(0).count++;
                            }
                        } // end of matched lane

                        bgLaneItor++;

                    } // end of lane iteration
                } // end of db not empty

#if VISUALIZATION_ON
                saveData(_bgDatabaseList, _curSecId);
                saveData(_bgRevDirList, _curSecId, true);
#endif

            } // end of valid processing

        } // end of lane iterator of each group

        grpItor++;
    } // end of reported data group iterator

    return true;
}


bool CRoadVecGen2::stitchSectionLanes(IN bool bRevDir/* = false*/)
{
    // check database data
    if (_segConfigList.empty())
    {
        return false;
    }

    // number of sections
    int numOfSeg = _segConfigList.size();
    int numOfBgSeg = _bgDatabaseList.size();
    int numOfBgRevSeg = _bgRevDirList.size();

    if ((numOfBgSeg != numOfSeg) ||
        (numOfSeg != numOfBgRevSeg) || (numOfBgSeg != numOfBgRevSeg))
    {
        return false;
    }

    // iterate each section in background database
    int numOfLanes = 0;
    double theta = 0.0;

    backgroundSectionData *bgSegData = nullptr;
    getBgDatabaseData(&bgSegData, bRevDir);

    foregroundSectionData *fgSegData = nullptr;
    getFgDatabaseData(&fgSegData, bRevDir);

    // none empty sections
    if (bgSegData && !bgSegData->bgSectionData.empty() && fgSegData)
    {
        fgSegData->fgSectionData.clear();

        if (!_bHasChanged && bRevDir)
        {
            _bHasRevDirData = true;
            _bHasChanged = true;
        }

        theta = _secRotAngle[_curSecId - 1];

        // number of lanes in current section
        numOfLanes = _secLaneNum[_curSecId - 1];

        if (0 < numOfLanes && numOfLanes <= MAX_SUPPORTED_LANES)
        {
            // calculate the valid lanes number, and record the number
            // of start lane
            vector<int> validLaneInd;
            int numOfValidLanes = numOfLanes;

            list<list<vector<point3D_t>>>::iterator laneItor = bgSegData->bgSectionData.begin();
            while (laneItor != bgSegData->bgSectionData.end())
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
                laneItor = bgSegData->bgSectionData.begin();

                while (laneItor != bgSegData->bgSectionData.end())
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
                            curPnt.paintLength = laneItor->front().at(i).paintLength;
                            leftSample.push_back(curPnt);

                            curPnt.lon = laneItor->back().at(i).lon;
                            curPnt.paintFlag = laneItor->back().at(i).paintFlag;
                            curPnt.paintLength = laneItor->back().at(i).paintLength;
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
                if (bRevDir)
                {
                    sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_shared_lines_rev.png",
                        FG_MERGED_NUM, _curSecId);
                }
                else
                {
                    sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_shared_lines.png",
                        FG_MERGED_NUM, _curSecId);
                }
                showImage(dblines,  Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif

                // number of dblines should be double of valid lanes
                if (2 * numOfValidLanes != dblines.size())
                {
#if VISUALIZATION_ON
                    printf("number of lines not match with number of lanes\n");
#endif
                    return false;
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

                            return false;
                        }

                        for (uint32 i = 0; i < lineOne.size(); i++)
                        {
                            curPnt.lon = lineOne[i].lon;
                            curPnt.lat = (lineOne[i].lat + lineTwo[i].lat) / 2;
                            curPnt.paintFlag = lineOne[i].paintFlag; // + lineTwo[i].paintFlag;
                            curPnt.paintLength = lineOne[i].paintLength;

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

#if SAVE_DATA_ON
                    if (bRevDir)
                    {
                        sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_mergedmiddle_lines_rev.txt",
                            FG_MERGED_NUM, _curSecId);
                    }
                    else
                    {
                        sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_mergedmiddle_lines.txt",
                            FG_MERGED_NUM, _curSecId);
                    }
                    saveListVec(midlines, IMAGE_NAME_STR2);
#endif
#if VISUALIZATION_ON
                    if (bRevDir)
                    {
                        sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_mergedmiddle_lines_rev.png",
                            FG_MERGED_NUM, _curSecId);
                    }
                    else
                    {
                        sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_mergedmiddle_lines.png",
                            FG_MERGED_NUM, _curSecId);
                    }
                    showImage(midlines,  Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
                }

                // add lines to foreground database
                if (SINGLE_LANE == numOfValidLanes)
                {
                    // two lines
                    fgSegData->fgSectionData.push_back(dblines.front());
                    fgSegData->fgSectionData.push_back(dblines.back());
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
                        curPnt.paintLength = dblines.front().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

                    // 2nd line
                    fgSegData->fgSectionData.push_back(midlines.front());

                    // 3rd line
                    for (uint32 i = 0; i < dblines.back().size(); i++)
                    {
                        curPnt.lon = dblines.back().at(i).lon;
                        curPnt.lat = dblines.back().at(i).lat + distlines.back().at(i);
                        curPnt.paintFlag = dblines.back().at(i).paintFlag;
                        curPnt.paintLength = dblines.back().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
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
                        curPnt.paintLength = dblines.front().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

                    // 2nd line
                    for (uint32 i = 0; i < midlines.front().size(); i++)
                    {
                        curPnt.lon = midlines.front().at(i).lon;
                        curPnt.lat = midlines.front().at(i).lat + d2[i];
                        curPnt.paintFlag = midlines.front().at(i).paintFlag;
                        curPnt.paintLength = midlines.front().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

                    // 3rd line
                    for (uint32 i = 0; i < midlines.back().size(); i++)
                    {
                        curPnt.lon = midlines.back().at(i).lon;
                        curPnt.lat = midlines.back().at(i).lat + d1[i];
                        curPnt.paintFlag = midlines.back().at(i).paintFlag;
                        curPnt.paintLength = midlines.back().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

                    // 4th line
                    for (uint32 i = 0; i < dblines.back().size(); i++)
                    {
                        curPnt.lon = dblines.back().at(i).lon;
                        curPnt.lat = dblines.back().at(i).lat + d1[i] + d3[i];
                        curPnt.paintFlag = dblines.back().at(i).paintFlag;
                        curPnt.paintLength = dblines.back().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();
                } // 3 lanes

#if VISUALIZATION_ON
                if (bRevDir)
                {
                    sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_rotated_lines_rev.png",
                        FG_MERGED_NUM, _curSecId);
                }
                else
                {
                    sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_rotated_lines.png",
                        FG_MERGED_NUM, _curSecId);
                }
                showImage(fgSegData->fgSectionData, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif

            } // end of none valid lanes
        }
        else
        {
            printf("number of lanes in section %d is not valid, %d\n",
                bgSegData->sectionId, numOfLanes);
        } // end of number of lanes
    } // end of merging valid section

#if SAVE_DATA_ON
    if (bRevDir)
    {
        sprintf_s(IMAGE_NAME_STR2, MAX_PATH - 1, "fg_%d_sec_%d_merged_rev.txt",
            FG_MERGED_NUM, fgSegData->sectionId);
    }
    else
    {
        sprintf_s(IMAGE_NAME_STR2, MAX_PATH - 1, "fg_%d_sec_%d_merged.txt",
            FG_MERGED_NUM, fgSegData->sectionId);
    }
    saveListVec(fgSegData->fgSectionData, IMAGE_NAME_STR2);
#endif

    return true;
}


bool CRoadVecGen2::stitchSharedLines(IN bool bNeedMerge/* = false*/)
{
    // get forward and backward direction database data
    backgroundSectionData *bgSegData = nullptr;
    getBgDatabaseData(&bgSegData);

    backgroundSectionData *bgSegRevData = nullptr;
    getBgDatabaseData(&bgSegRevData, true);

    foregroundSectionData *fgSegData = nullptr;
    getFgDatabaseData(&fgSegData);

    foregroundSectionData *fgSegRevData = nullptr;
    getFgDatabaseData(&fgSegRevData, true);

    foregroundSectionData *fgSegAllData = nullptr;
    getFgAllDatabaseData(&fgSegAllData);

    // only handle non-empty forward and backward database
    // merge backward direction right most with forward direction left most line
    if (((fgSegData && !fgSegData->fgSectionData.empty()) ||
         (fgSegRevData && !fgSegRevData->fgSectionData.empty())) &&
         fgSegAllData)
    {
        fgSegAllData->fgSectionData.clear();

        bool bHasSharedLine = false;
        vector<double> d1, d2;
        vector<point3D_t> mergedline;

        // backward direction
        if (!bgSegRevData->bgSectionData.empty())
        {
            int numOfRevUsedLanes = 0, numOfRevUsedLines = 0;
            list<list<vector<point3D_t>>>::iterator laneRevIt = bgSegRevData->bgSectionData.begin();
            while (laneRevIt->empty())
            {
                vector<point3D_t> line;
                fgSegAllData->fgSectionData.push_back(line);

                numOfRevUsedLanes++;
                laneRevIt++;
            }

            // check valid foreground lines
            int numOfRevLines = fgSegRevData->fgSectionData.size();
            if (numOfRevUsedLanes + numOfRevLines - 1 == _secLaneNum[_curSecId - 1])
            {
                // only valid backward right most lane and forward left most line
                if (bNeedMerge &&
                    (bgSegData && !bgSegData->bgSectionData.empty() &&
                    !bgSegData->bgSectionData.front().empty()) &&
                    (bgSegRevData && !bgSegRevData->bgSectionData.empty() &&
                    !bgSegRevData->bgSectionData.back().empty()))
                {
                    vector<point3D_t> line1 = fgSegRevData->fgSectionData.back();
                    vector<point3D_t> line2 = fgSegData->fgSectionData.front();
                    int numOfPnt1 = line1.size();
                    int numOfPnt2 = line2.size();

                    // size should be the same
                    if (numOfPnt1 == numOfPnt2)
                    {
                        point3D_t pnt = { 0 };

                        for (int i = 0; i < numOfPnt1; i++)
                        {
                            pnt.lon = line1.at(i).lon;
                            pnt.lat = 0.5 * line1.at(i).lat + 0.5 * line2.at(i).lat;
                            pnt.paintFlag = line2.at(i).paintFlag;
                            pnt.paintLength = line2.at(i).paintLength;

                            mergedline.push_back(pnt);
                            d1.push_back(pnt.lat - line1.at(i).lat);
                            d2.push_back(pnt.lat - line2.at(i).lat);
                        }

                        bHasSharedLine = true;
                    }
                    else
                    {
                        printf("ERROR: stitchSharedLines size not equal. \n");
                    }
                }
            }

            // add backward direction database lines
            list<vector<point3D_t>>::iterator fgRevLineIt = fgSegRevData->fgSectionData.begin();
            for (int ii = 0; ii < numOfRevLines - 1; ii++)
            {
                if (bHasSharedLine)
                {
                    int numOfPnts = fgRevLineIt->size();
                    vector<point3D_t> line;
                    for (int jj = 0; jj < numOfPnts; jj++)
                    {
                        point3D_t pnt = fgRevLineIt->at(jj);
                        pnt.lat += d1[jj];

                        line.push_back(pnt);
                    }

                    fgSegAllData->fgSectionData.push_back(line);
                }
                else
                {
                    fgSegAllData->fgSectionData.push_back(*fgRevLineIt);
                }

                numOfRevUsedLines++;
                fgRevLineIt++;
            }

            // for last line
            if (bHasSharedLine)
            {
                fgSegAllData->fgSectionData.push_back(mergedline);
                numOfRevUsedLines++;
            }
            else
            {
                fgSegAllData->fgSectionData.push_back(fgSegRevData->fgSectionData.back());
                numOfRevUsedLines++;
            }

            // for remaining empty lanes
            while (numOfRevUsedLanes + numOfRevUsedLines - 1 < _secLaneNum[_curSecId - 1])
            {
                vector<point3D_t> line;
                fgSegAllData->fgSectionData.push_back(line);

                numOfRevUsedLanes++;
            }
        }

        // the shared line and forward direction lines
        if (bHasSharedLine)
        {
            // forward direction
            int numOfLines = fgSegData->fgSectionData.size();
            list<vector<point3D_t>>::iterator fgLineIt = fgSegData->fgSectionData.begin();
            for (int ii = 0; ii < numOfLines - 1; ii++)
            {
                fgLineIt++;

                int numOfPnts = fgLineIt->size();
                vector<point3D_t> line;
                for (int jj = 0; jj < numOfPnts; jj++)
                {
                    point3D_t pnt = fgLineIt->at(jj);
                    pnt.lat += d2[jj];

                    line.push_back(pnt);
                }

                fgSegAllData->fgSectionData.push_back(line);
            }

            // for empty lane
            int numOfUsedLanes = numOfLines - 1;
            while (numOfUsedLanes < _secLaneNum[_curSecId - 1])
            {
                vector<point3D_t> line;
                fgSegAllData->fgSectionData.push_back(line);

                numOfUsedLanes++;
            }
        }
        else
        {
            // if has backward direction database, push empty lanes
            if (_bHasRevDirData && bgSegRevData->bgSectionData.empty())
            {
                int numOfRevUsedLanes = 0;
                while (numOfRevUsedLanes < _secLaneNum[_curSecId - 1])
                {
                    vector<point3D_t> line;
                    fgSegAllData->fgSectionData.push_back(line);

                    numOfRevUsedLanes++;
                }
            }

            // forward direction
            if (!bgSegData->bgSectionData.empty())
            {
                int numOfUsedLanes = 0;
                list<list<vector<point3D_t>>>::iterator laneIt = bgSegData->bgSectionData.begin();
                while (laneIt->empty())
                {
                    vector<point3D_t> line;
                    fgSegAllData->fgSectionData.push_back(line);

                    numOfUsedLanes++;
                    laneIt++;
                }

                int numOfLines = fgSegData->fgSectionData.size();
                list<vector<point3D_t>>::iterator fgLineIt = fgSegData->fgSectionData.begin();
                for (int ii = 0; ii < numOfLines; ii++)
                {
                    fgSegAllData->fgSectionData.push_back(*fgLineIt);

                    fgLineIt++;
                }

                // for empty lane
                while (numOfUsedLanes + numOfLines - 1 < _secLaneNum[_curSecId - 1])
                {
                    vector<point3D_t> line;
                    fgSegAllData->fgSectionData.push_back(line);

                    numOfUsedLanes++;
                }
            }
        }
    }

    return true;
}


void CRoadVecGen2::removeOverlap(OUT list<list<vector<point3D_t>>> &fgData)
{
    if (!_fgDatabaseList.empty())
    {
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
                fgData.push_back(fglines);

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

                    lItor++;
                }

                fgData.push_back(fglines);

#if VISUALIZATION_ON
                sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_output_lines.png",
                    FG_MERGED_NUM, _curSecId);
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

void CRoadVecGen2::removeOverlap()
{
    foregroundSectionData *fgSecData = nullptr;
    getFgAllDatabaseData(&fgSecData);

    foregroundSectionData *fgOutSecData = nullptr;
    getFgOutDatabaseData(&fgOutSecData);

    // remove overlap for current section
    if (fgSecData && !fgSecData->fgSectionData.empty() && fgOutSecData)
    {
        int numOfFGLines = 0, numOfUsedLanes = 0, ind = 0, numOfValidLines = 0;
        double *py11 = nullptr, *py12 = nullptr,
            *py21 = nullptr, *py22 = nullptr;
        double step = 0.0, stx = 0.0, edx = 0.0;
        double my1 = 0.0, my2 = 0.0, sy1 = 0.0, sy2 = 0.0;

        vector<point3D_t> line1, line2, fgl, rotline;

        list<vector<point3D_t>>::iterator lItor;
        list<list<vector<point3D_t>>>::iterator laneItor;

        fgOutSecData->fgSectionData.clear();

        // valid foreground line data
        numOfFGLines = fgSecData->fgSectionData.size();
        py11 = new double[numOfFGLines];
        py12 = new double[numOfFGLines];
        py21 = new double[numOfFGLines];
        py22 = new double[numOfFGLines];

        // adjacent Y value of two body start/end points
        if (py11 && py12 && py21 && py22)
        {
            ind = 0;
            my1 = 0.0; my2 = 0.0; sy1 = 0.0; sy2 = 0.0;

            lItor = fgSecData->fgSectionData.begin();

            while (lItor != fgSecData->fgSectionData.end())
            {
                if (!lItor->empty())
                {
                    step = lItor->at(1).lon - lItor->at(0).lon;
                    stx  = lItor->at(_secBodyStInd[_curSecId - 1]).lon;
                    edx  = lItor->at(_secBodyEdInd[_curSecId - 1]).lon;

                    py11[ind] = lItor->at(_secBodyStInd[_curSecId - 1]).lat;
                    py12[ind] = lItor->at(_secBodyStInd[_curSecId - 1] + 1).lat;
                    my1 += (py11[ind] - py12[ind]);
                    sy1 += py11[ind];

                    py21[ind] = lItor->at(_secBodyEdInd[_curSecId - 1] - 1).lat;
                    py22[ind] = lItor->at(_secBodyEdInd[_curSecId - 1]).lat;
                    my2 += (py21[ind] - py22[ind]);
                    sy2 += py21[ind];

                    numOfValidLines++;
                }

                ind++;
                lItor++;
            }

            double c1 = sy1 / numOfValidLines;
            double c2 = sy2 / numOfValidLines;

            double angle1 = atan(my1 / (numOfValidLines * step));
            double angle2 = atan(my2 / (numOfValidLines * step));

            // start point
            complex<double> theta1(0, angle1);
            complex<double> yj1(0, c1);
            double cox1 = real((stx + yj1) * exp(theta1));

            // end point
            complex<double> theta2(0, angle2);
            complex<double> yj2(0, c2);
            double cox2 = real((edx + yj2) * exp(theta2));

            lItor = fgSecData->fgSectionData.begin();
            while (lItor != fgSecData->fgSectionData.end())
            {
                if (!lItor->empty())
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
                    fgOutSecData->fgSectionData.push_back(rotline);
                }
                else
                {
                    vector<point3D_t> line;
                    fgOutSecData->fgSectionData.push_back(line);
                }

                lItor++;
            }

#if VISUALIZATION_ON
            sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_output_lines.png",
                FG_MERGED_NUM, _curSecId);
            showImage(fgOutSecData->fgSectionData, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
        }

        SAFEARR_DELETE(py11);
        SAFEARR_DELETE(py12);
        SAFEARR_DELETE(py21);
        SAFEARR_DELETE(py22);

    } // end of remove overlap for current section
}


void CRoadVecGen2::jointProcessing(OUT list<list<vector<point3D_t>>> &fgData,
    IN bool bCommLinesMerged)
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
    list<foregroundSectionData>::iterator prevSecItor = _fgOutputList.begin();
    list<foregroundSectionData>::iterator currSecItor = _fgOutputList.begin();
    foregroundSectionData currData, prevData;

    // background database section iterator used to get merged information
    list<backgroundSectionData>::iterator bgSecItor = _bgDatabaseList.begin();
    list<backgroundSectionData>::iterator bgSecRevItor = _bgRevDirList.begin();
    list<list<vector<point3D_t>>>::iterator laneItor, laneRevItor;

    for (int i = 0; i < numOfSegs; i++)
    {
        currData = *currSecItor;
        currSecItor++;

        // if current section is the first section and circle exists, then
        // the previous section is the last one
        if (i == 0)
        {
            if (_bCircleRoad)
            {
                prevData = _fgOutputList.back();
            }
        }
        else
        {
            prevData = *prevSecItor;
            prevSecItor++;
        }

        // current section Id
        _curSecId = currData.sectionId;

        // current section data is empty, push empty lines to match lane number
        if (currData.fgSectionData.empty())
        {
            bgSecItor++; bgSecRevItor++;

            list<vector<point3D_t>> fglines;

            fgData.push_back(fglines);
            continue;
        }

        // previous section data is empty
        if (prevData.fgSectionData.empty())
        {
            list<vector<point3D_t>> fglines;
            list<vector<point3D_t>>::iterator lineItor = currData.fgSectionData.begin();

            int numOfUsedLines = 0;
            laneItor = bgSecItor->bgSectionData.begin();
            laneRevItor = bgSecRevItor->bgSectionData.begin();
            while (lineItor != currData.fgSectionData.end())
            {
                if (numOfUsedLines < _secLaneNum[_curSecId - 1] &&
                    !bgSecRevItor->bgSectionData.empty())
                {
                    if (laneRevItor != bgSecRevItor->bgSectionData.end())
                    {
                        if (!laneRevItor->empty() && !lineItor->empty())
                        {
                            lineItor->at(0).count = laneRevItor->front().at(0).count;
                        }

                        laneRevItor++;
                    }
                }
                else
                {
                    if (!bCommLinesMerged && numOfUsedLines == _secLaneNum[_curSecId - 1])
                    {
                        if (!lineItor->empty())
                        {
                            lineItor->at(0).count = 0;
                        }
                    }
                    else
                    {
                        if (!bgSecItor->bgSectionData.empty())
                        {
                            if (laneItor != bgSecItor->bgSectionData.end())
                            {
                                if (!laneItor->empty() && !lineItor->empty())
                                {
                                    lineItor->at(0).count = laneItor->front().at(0).count;
                                }

                                laneItor++;
                            }
                        }
                    }
                }

                if (!lineItor->empty())
                {
                    fglines.push_back(*lineItor);
                }

                numOfUsedLines++;
                lineItor++;
            }

            bgSecItor++; bgSecRevItor++;

            fgData.push_back(fglines);
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
            vector<int> matchedInd;
            vector<point3D_t> stPnts;
            vector<point3D_t> edPnts;

            // get matched line index
            getMatchedLineInd(matchedInd, prevData.sectionId, _bHasRevDirData, bCommLinesMerged);

            int numOfMatchedLines = matchedInd.size();
            for (int i = 0; i < numOfMatchedLines; i++)
            {
                if (UNMATCHED_LINE_FLAG != matchedInd[i] &&
                    i < currlines && matchedInd[i] < prevlines &&
                    (!ppCurrLines[i]->empty()) &&
                    (!ppPrevLines[matchedInd[i]]->empty()))
                {
                    stPnts.push_back(ppCurrLines[i]->front());
                    edPnts.push_back(ppPrevLines[matchedInd[i]]->back());
                }
                else
                {
                    matchedInd[i] = UNMATCHED_LINE_FLAG;
                }
            }

            // if exist matched lines, size of stPnts and edPnts should be
            // the same
            if (!stPnts.empty() && !edPnts.empty())
            {
                int matchedCnt = stPnts.size();
                int index = 0;
                double dx = 0.0, dy = 0.0, ddx = 0.0, ddy = 0.0;

                for (int i = 0; i < currlines; i++)
                {
                    if (UNMATCHED_LINE_FLAG == matchedInd[i])
                    {
                        int ind = (i < matchedCnt) ? 0 : (matchedCnt - 1);

                        dx = edPnts[ind].lon - stPnts[ind].lon;
                        dy = edPnts[ind].lat - stPnts[ind].lat;

                    }
                    else
                    {
                        dx = edPnts[index].lon - stPnts[index].lon;
                        dy = edPnts[index].lat - stPnts[index].lat;
                        index++;
                    }

                    int numOfHalfPnts = ppCurrLines[i]->size() / 2;

                    double ddx = dx / numOfHalfPnts;
                    double ddy = dy / numOfHalfPnts;

                    // for matched lines
                    for (int pi = 0; pi < numOfHalfPnts; pi++)
                    {
                        ppCurrLines[i]->at(pi).lon += (dx - pi * ddx);
                        ppCurrLines[i]->at(pi).lat += (dy - pi * ddy);
                    }
                }
            }

            // push current section lines to fgData
            laneItor = bgSecItor->bgSectionData.begin();
            laneRevItor = bgSecRevItor->bgSectionData.begin();

            list<vector<point3D_t>> fglines;
            for (int i = 0; i < currlines; i++)
            {
                // get merged information
                if (_bHasRevDirData && (i < _secLaneNum[_curSecId - 1]))
                {
                    if (laneRevItor != bgSecRevItor->bgSectionData.end())
                    {
                        if (!laneRevItor->empty() && !ppCurrLines[i]->empty())
                        {
                            ppCurrLines[i]->at(0).count = laneRevItor->front().at(0).count;
                        }

                        laneRevItor++;
                    }
                }
                else
                {
                    if (!bCommLinesMerged && i == _secLaneNum[_curSecId - 1])
                    {
                        if (!ppCurrLines[i]->empty())
                        {
                            ppCurrLines[i]->at(0).count = 0;
                        }
                    }
                    else
                    {
                        if (laneItor != bgSecItor->bgSectionData.end())
                        {
                            if (!laneItor->empty() && !ppCurrLines[i]->empty())
                            {
                                ppCurrLines[i]->at(0).count = laneItor->front().at(0).count;
                            }

                            laneItor++;
                        }
                    }
                }

                if (!ppCurrLines[i]->empty())
                {
                    fglines.push_back(*(ppCurrLines[i]));
                }
            }

            fgData.push_back(fglines);
        }

        bgSecItor++; bgSecRevItor++;
    }
}


void CRoadVecGen2::getBgDatabaseData(OUT backgroundSectionData **bgSegData,
    IN bool revDirFlag/* = false*/)
{
    if (_segConfigList.empty() || nullptr == bgSegData)
    {
        return;
    }

    if (!revDirFlag && _bgDatabaseList.empty())
    {
        return;
    }
    if (revDirFlag && _bgRevDirList.empty())
    {
        return;
    }

    list<backgroundSectionData> *bgDbList = revDirFlag ? &_bgRevDirList : &_bgDatabaseList;

    // get background database data
    list<backgroundSectionData>::iterator dbItor = bgDbList->begin();
    while (dbItor != bgDbList->end())
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


void CRoadVecGen2::getFgDatabaseData(OUT foregroundSectionData **fgSegData,
    IN  bool revDirFlag/* = false*/)
{
    if (_segConfigList.empty() || nullptr == fgSegData)
    {
        return;
    }

    if (!revDirFlag && _fgDatabaseList.empty())
    {
        return;
    }
    if (revDirFlag && _fgRevDirList.empty())
    {
        return;
    }

    list<foregroundSectionData> *fgDbList = revDirFlag ? &_fgRevDirList : &_fgDatabaseList;

    // get foreground database data (with overlap)
    list<foregroundSectionData>::iterator dbItor = fgDbList->begin();
    while (dbItor != fgDbList->end())
    {
        if (_curSecId == dbItor->sectionId)
        {
            *fgSegData = &(*dbItor);
            break;
        }
        dbItor++;
    }

    return;
}


void CRoadVecGen2::getFgAllDatabaseData(OUT foregroundSectionData **fgSegData)
{
    if (_segConfigList.empty() || nullptr == fgSegData)
    {
        return;
    }

    // get foreground all database data (with overlap)
    list<foregroundSectionData>::iterator dbItor = _fgAllDirList.begin();
    while (dbItor != _fgAllDirList.end())
    {
        if (_curSecId == dbItor->sectionId)
        {
            *fgSegData = &(*dbItor);
            break;
        }
        dbItor++;
    }

    return;
}


void CRoadVecGen2::getFgOutDatabaseData(OUT foregroundSectionData **fgOutSegData)
{
    if (_fgOutputList.empty() || nullptr == fgOutSegData)
    {
        return;
    }

    // get foreground output database data(without overlap)
    list<foregroundSectionData>::iterator dbItor = _fgOutputList.begin();
    while (dbItor != _fgOutputList.end())
    {
        if (_curSecId == dbItor->sectionId)
        {
            *fgOutSegData = &(*dbItor);
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
        dx.push_back(abs(lineData[i].lon - lineData[i - 1].lon));
        dy.push_back(abs(lineData[i].lat - lineData[i - 1].lat));

        dd.push_back(dx[i - 1] + dy[i - 1]);
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

    if (leftSmpSt[0] > leftSmpEd[0])
    {
        reverse(leftSmpSt.begin(), leftSmpSt.end());
        reverse(leftSmpEd.begin(), leftSmpEd.end());

        vector<int> tmp = leftSmpSt;
        leftSmpSt = leftSmpEd;
        leftSmpEd = tmp;
    }

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
                leftline[i].paintLength = (float)(leftSmpEd[blkIndStEd] - leftSmpSt[blkIndStEd] + 1);
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
    int numOfPnts = fgline.size();
    float maxPaint = 1.0;
    for (int i = 0; i < numOfPnts; i++)
    {
        if (fgline[i].paintFlag > maxPaint)
        {
            maxPaint = fgline[i].paintFlag;
        }
    }

    // get valid paint block start and end index
    bool bPaintChanged = false;
    vector<int> stInd, edInd;
    vector<point3D_t> tmpline = fgline;
    for (int i = 0; i < numOfPnts; i++)
    {
        tmpline[i].paintFlag /= maxPaint;

        if (0.5 <= tmpline[i].paintFlag)
        {
            if (!bPaintChanged)
            {
                stInd.push_back(i);
            }
            bPaintChanged = true;
        }
        else
        {
            if (bPaintChanged)
            {
                edInd.push_back(i - 1);
                bPaintChanged = false;
            }
        }
    }

    // for end point is painted
    if (bPaintChanged && (stInd.size() > edInd.size()))
    {
        edInd.push_back(numOfPnts - 1);
    }

    // size of stInd and edInd should be the same
    int numOfBlks = stInd.size();
    for (int i = 0; i < numOfBlks; i++)
    {
        int start = 0, end = 0;
        float startFlag = 0.0, endFlag = 0.0;
        int numOfElems = edInd[i] - stInd[i] + 1;

        // get max merged start and end index
        for (int jj = 0; jj < numOfElems; jj++)
        {
            if (tmpline[stInd[i] + jj].paintFlag > startFlag)
            {
                start = stInd[i] + jj;
                startFlag = tmpline[stInd[i] + jj].paintFlag;
            }
            tmpline[stInd[i] + jj].paintFlag = 0.0;

            if (tmpline[edInd[i] - jj].paintFlag > endFlag)
            {
                end = edInd[i] - jj;
                endFlag = tmpline[edInd[i] - jj].paintFlag;
            }
            tmpline[edInd[i] - jj].paintFlag = 0.0;
        }

        // center point
        int middle = (start + end) / 2;
        int paintLength = (int)(max(tmpline[start].paintLength, tmpline[end].paintLength) + 1);
        start = middle - paintLength / 2;
        end = middle + paintLength / 2;

        // limit the index to be within the 0.5 range
        start = (start >= 0) ? start : 0;
        end = (end < numOfPnts) ? end : (numOfPnts - 1);
        start = (start >= stInd[i]) ? start : stInd[i];
        end = (end <= edInd[i]) ? end : edInd[i];

        for (int jj = start; jj <= end; jj++)
        {
            tmpline[jj].paintFlag = 1.0;
        }
    }

    // normalize paint information
    for (uint32 i = 1; i < fgline.size(); i++)
    {
        fgline[i].paintFlag = tmpline[i].paintFlag;
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
    sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_rotated.png",
        FG_MERGED_NUM, _curSecId, matchedLane, MERGED_TIMES[_curSecId - 1]);
    showImage(rotated, Scalar(0, 255, 0), IMAGE_NAME_STR2);
#endif

    if (INVALID_LANE_NUM == matchedLane)
    {
        return false;
    }

#if VISUALIZATION_ON
    sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_source.png",
        FG_MERGED_NUM, _curSecId, matchedLane, MERGED_TIMES[_curSecId - 1]);
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
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
    bool cond00 = 3.0 < abs(meanY0) && abs(meanY0) < 8.0;
    bool cond01 = stdY0 < 5;
    bool cond10 = 3.0 < abs(meanY1) && abs(meanY1) < 8.0;
    bool cond11 = stdY1 < 5;
#else
    bool cond00 = 3.0 < abs(meanY0) && abs(meanY0) < 6.0;
    bool cond01 = stdY0 < 3;
    bool cond10 = 3.0 < abs(meanY1) && abs(meanY1) < 6.0;
    bool cond11 = stdY1 < 3;
#endif

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
            _curSecId, matchedLane, MERGED_TIMES[_curSecId - 1]);
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
        sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_interpolate_sample_.png",
            FG_MERGED_NUM, _curSecId, matchedLane, MERGED_TIMES[_curSecId - 1]);
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
        sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_polyval_sample_.png",
            FG_MERGED_NUM, _curSecId, matchedLane, MERGED_TIMES[_curSecId - 1]);
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
    double threshold =  LINE_TYPE_TH;
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
    if (TRIPLE_LANE == numOfLanes)
    {
        matchedLane = lanetype;
    }
}

void CRoadVecGen2::checkCircleRoad()
{
    _bCircleRoad = true;

    if (!_secLaneConn.empty())
    {
        int curSegId = 0, valSum = 0;
        int numOfSegs = _secLaneConn.size();

        list<vector<int>>::iterator connIt = _secLaneConn.begin();
        while (connIt != _secLaneConn.end())
        {
            valSum = 0;

            if (connIt->size() == _secLaneNum[curSegId])
            {
                for (int i = 0; i < _secLaneNum[curSegId]; i++)
                {
                    valSum += connIt->at(i);
                }

                if (valSum == -1 * _secLaneNum[curSegId])
                {
                    _bCircleRoad = false;
                    break;
                }
            }

            curSegId++;
            connIt++;
        }
    }
}


void CRoadVecGen2::getMatchedLineInd(OUT vector<int> &matchedInd,
    IN uint32 segId,
    IN bool   bHasRevData,
    IN bool   bCommLinesMerged)
{
    matchedInd.clear();

    if (_secLaneConn.empty())
    {
        return;
    }

    int currSegId = 1;
    int index[MAX_SUPPORTED_LANES + 1] = {UNMATCHED_LINE_FLAG,
        UNMATCHED_LINE_FLAG, UNMATCHED_LINE_FLAG,
        UNMATCHED_LINE_FLAG, UNMATCHED_LINE_FLAG};
    list<vector<int>>::iterator connIt = _secLaneConn.begin();
    while (connIt != _secLaneConn.end())
    {
        if (currSegId == segId)
        {
            int numOfLanes = connIt->size();
            for (int i = 0; i < numOfLanes; i++)
            {
                if (-1 != connIt->at(i))
                {
                    // for connected lane
                    index[i]     = connIt->at(i) - 1;
                    index[i + 1] = connIt->at(i);
                }
            }

            break;
        }

        currSegId++;
        connIt++;
    }

    // matched line index for current section
    int numOfLines = _secLaneNum[segId % _secLaneNum.size()] + 1;
    for (int i = 0; i < numOfLines; i++)
    {
        matchedInd.push_back((UNMATCHED_LINE_FLAG != index[i]) ? index[i] : UNMATCHED_LINE_FLAG);
    }

    if (bHasRevData)
    {
        int numOfRevLines = numOfLines;
        if (bCommLinesMerged)
        {
            numOfRevLines = numOfLines - 1;
        }

        for (int i = 0; i < numOfRevLines; i++)
        {
            matchedInd.push_back(matchedInd[i] + numOfLines);
        }
    }
}


void CRoadVecGen2::preprocessRptData(INOUT list<list<vector<point3D_t>>>& rptData,
    IN list<vector<point3D_t>> &gpsData)
{
    int numOfLanes = rptData.size();
    int numOfTracks = gpsData.size();

    // number of lanes should be the same with number of tracks
    if (numOfLanes == numOfTracks)
    {
        list<list<vector<point3D_t>>>::iterator laneIt = rptData.begin();
        list<vector<point3D_t>>::iterator trackIt = gpsData.begin();
        while (laneIt != rptData.end() && trackIt != gpsData.end())
        {
            int numOfVecPnts = laneIt->front().size();
            int numOfGpsPnts = trackIt->size();
            if (numOfGpsPnts == numOfVecPnts)
            {
                // mean relative distance of left and right line
                int lCnt = 0, rCnt = 0;
                double lDist = 0.0, rDist = 0.0;
                for (int i = 0; i < numOfVecPnts; ++i)
                {
                    if (0.005 >= abs(laneIt->front().at(i).paintFlag - 1.0))
                    {
                        double lat = laneIt->front().at(i).lat - trackIt->at(i).lat;
                        double lon = laneIt->front().at(i).lon - trackIt->at(i).lon;
                        lDist = lat * lat + lon * lon;
                        lCnt++;
                    }

                    if (0.005 >= abs(laneIt->back().at(i).paintFlag - 1.0))
                    {
                        double lat = laneIt->back().at(i).lat - trackIt->at(i).lat;
                        double lon = laneIt->back().at(i).lon - trackIt->at(i).lon;
                        rDist = lat * lat + lon * lon;
                        rCnt++;
                    }
                }

                // add data for undetected points
                if (0 < lCnt)
                {
                    double mLDist = lDist / lCnt;
                }

                if (0 < rCnt)
                {
                    double mRDist = rDist / rCnt;
                }
            }

            ++laneIt;
            ++trackIt;
        }
    }
}


} // end of namespace ns_database


