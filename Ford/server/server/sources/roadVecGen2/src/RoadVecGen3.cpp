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
*      2015/11/3        Xin  Shao         Modify
*******************************************************************************
*/

#include <complex>

#include "AppInitCommon.h"
#include "apiDataStruct.h"
#include "polynomialFit.h"
#include "RoadVecGen3.h"
#include "configure.h"
#include "RoadSeg.h"

#if VISUALIZATION_ON || SAVE_DATA_ON
#include "VisualizationApis.h"

extern uint32 PREVIOUS_SEGID;
extern uint32 FG_MERGED_NUM;

extern uint32 MERGED_TIMES[100];
extern char IMAGE_NAME_STR2[MAX_PATH];

using namespace std;
#endif

using namespace ns_roadsegment;

namespace ns_database
{
    // this macro is used to do uniform paint for solid lines
#define UNIFORM_SOLID_PAINT       1
#define UNIFORM_SOLID_WEIGHT      0.8
#define LEFT_ROAD_LINE            0
#define RIGHT_ROAD_LINE           1
#define LEFT_RIGHT_ROAD_LINE      2

    // number of section configuration points
#define SEG_CFG_PNT_NUM           4

#define MINDIST                   10

#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
#define LINE_TYPE_TH              46
#elif(RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT_LARGE)
#define LINE_TYPE_TH              75
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
#define MATCHED_LINE_FLAG         1
#define UNMATCHED_LINE_FLAG       -1

#define SAFEARR_DELETE(p) if (p) { delete [] (p); p = nullptr; }

CRoadVecGen3::CRoadVecGen3(void)
{
    //_configPath = "";
    _curSecId = 0;
	_startSegId = 0;

    _bHasRevDirData = false;
    _bHasChanged = false;
	
    // create mutex
    _hMutexMerging = CreateMutex(NULL, FALSE, NULL);
    ReleaseMutex(_hMutexMerging);
}

void CRoadVecGen3::InitRoadVecGen()
{
	//initialize database
	initDatabase();

	// calculate section rotation angle and X data range
    calcRotAngleAndRange();

    vector<secCfgInfo_t> secCfgInfo;
	vector<secPointInfo_t> secPointInfo;
	vector<point3D_t> bodyPts3D;

	roadSegConfig_gp->getSecCfgInfo(secCfgInfo);
	roadSegConfig_gp->getsecPointInfo(secPointInfo);
	roadSegConfig_gp->getsecPointVector(bodyPts3D);

	_secRptDataObj.initCfg(secCfgInfo,secPointInfo,bodyPts3D);
}

/*
CRoadVecGen3::CRoadVecGen3(string configFilePath)
{
    //_configPath = configFilePath;

    _bHasRevDirData = false;
    _bHasChanged = false;

    // read section configuration file
    //list<segAttributes_t> segConfigList;
    //readSecConfig(segConfigList);

    // initialize database
    initDatabase();

    // calculate section rotation angle and X data range
    calcRotAngleAndRange();

    // check whether road is a circle
    //checkCircleRoad();

    // create mutex
    _hMutexMerging = CreateMutex(NULL, FALSE, NULL);
    ReleaseMutex(_hMutexMerging);
}
*/

CRoadVecGen3::~CRoadVecGen3(void)
{
    //_configPath = "";

    // release list or vector data
    //_secLaneNum.clear();
    _secRotAngle.clear();
    _secBodyStInd.clear();
    _secBodyEdInd.clear();
    _secLeftData.clear();
    _secRightData.clear();
    //_secLaneType.clear();
    //_secLaneConn.clear();
    //_segConfigList.clear();
    _bgDatabaseList.clear();
    _bgRevDirList.clear();
    _fgDatabaseList.clear();
    _fgRevDirList.clear();
    _fgAllDirList.clear();
    _fgOutputList.clear();

    // close mutex
    CloseHandle(_hMutexMerging);
}

bool CRoadVecGen3::getSideLane(IN  list<list<vector<point3D_t>>> &rptData,
	OUT list<list<vector<point3D_t>>> &mainRptData, OUT list<list<vector<point3D_t>>> &sideRptData)
{
	mainRptData.push_back(rptData.front());
	if(2 == rptData.size())
	{
		sideRptData.push_back(rptData.back());
		return true;
	}
	return false;
}


bool CRoadVecGen3::getSideLaneGps(IN list<vector<point3D_t>> &gpsData,
    OUT list<vector<point3D_t>> &mainGpsData,
    OUT list<vector<point3D_t>> &sideGpsData)
{
    if (gpsData.empty())
    {
        return false;
    }

    mainGpsData.push_back(gpsData.front());
    if (2 == gpsData.size())
    {
        sideGpsData.push_back(gpsData.back());
        return true;
    }

    return false;
}

bool CRoadVecGen3::findSameSectionId(IN  vector<uint32> &allId, IN uint32 segId)
{
	if(allId.empty())
	{
		return false;
	}

	for(int i = 0; i < allId.size(); i++)
	{
		if(segId == allId[i])
		{
			return true;
		}
	}
	return false;
}

void CRoadVecGen3::prevAndNextSegmentInquireLoop(IN int segId, INOUT vector<uint32> &allId)
{
	if(false == findSameSectionId(allId, segId))
	{
		allId.push_back(segId);
	}

	segment_type_e segType = NORMAL_E;
	if(false == roadSegConfig_gp->getSegmentType(segId, segType) || T_ROAD_CROSS != segType)
	{
		return;
	}	

	vector<uint32> preSedId, nextSedId;
	if(true == roadSegConfig_gp->getPrevSegId(segId, preSedId))
	{
		for(int i = 0; i < preSedId.size(); i++)
		{
			if(true == findSameSectionId(allId, preSedId[i]))
			{
				continue;	
			}
			prevAndNextSegmentInquireLoop(preSedId[i], allId);

		}			
	}

	if(true == roadSegConfig_gp->getNextSegId(segId, nextSedId))
	{
		for(int i = 0; i < nextSedId.size(); i++)
		{
			if(true == findSameSectionId(allId, nextSedId[i]))
			{
				continue;
			}
			
			prevAndNextSegmentInquireLoop(nextSedId[i], allId);
		}			
	}
	return;
}

void CRoadVecGen3::getModifiedSectionId(IN  list<reportSectionData> &secData,
    IN list<list<vector<point3D_t>>> &fgData,
    OUT list<uint32> &modifiedSectionId)
{
	vector<uint32> allId;
	list<reportSectionData>::iterator secDataItor = secData.begin();
	while (secDataItor != secData.end())
	{	
		if(false == findSameSectionId(allId, secDataItor->sectionId))
		{
			allId.push_back(secDataItor->sectionId);
		}

		vector<uint32> preSedId, nextSedId;
		if(true == roadSegConfig_gp->getPrevSegId(secDataItor->sectionId, preSedId))
		{
			for(int i = 0; i < preSedId.size(); i++)
			{
				prevAndNextSegmentInquireLoop(preSedId[i], allId);
			}			
		}

		if(true == roadSegConfig_gp->getNextSegId(secDataItor->sectionId, nextSedId))
		{
			for(int i = 0; i < nextSedId.size(); i++)
			{
				prevAndNextSegmentInquireLoop(nextSedId[i], allId);
			}			
		}

		secDataItor++;
	}

	for (int index = 0; index < allId.size(); index++)
	{
		uint32 secItem = 1;
		list<list<vector<point3D_t>>>::iterator fgSecItor = fgData.begin();
		while(fgSecItor != fgData.end())
		{
			if((allId[index] == secItem) && (!fgSecItor->empty()))
			{
				modifiedSectionId.push_back(secItem);			
			} 		
			secItem++;
			fgSecItor++;
		}
	}
}

bool CRoadVecGen3::roadSectionsGen(IN  list<list<vector<point3D_t>>> &rptData,
        IN  list<vector<point3D_t>> &gpsData,
        OUT list<list<vector<point3D_t>>> &fgData,
        OUT list<uint32> &modifiedSectionId)
{
    if (rptData.empty())
    {
        return false;
    }

    WaitForSingleObject(_hMutexMerging, INFINITE);

    // release data first
    if (!fgData.empty())
    {
        fgData.clear();
    }

	bool hasSideLane = false;
	list<list<vector<point3D_t>>> mainRptData, sideRptData;
	hasSideLane = getSideLane(rptData, mainRptData, sideRptData);

    bool hasSideGps = false;
    list<vector<point3D_t>> mainGpsData, sideGpsData;
    hasSideGps = getSideLaneGps(gpsData, mainGpsData, sideGpsData);

    // suppose output is list<reportSectionData> rptData;
    list<reportSectionData> secData, sideSecData;
	sideSecData.clear();

    // section partition, input is rptData
    //if(!_segConfigList.empty())
	{

#if SAVE_DATA_ON
		saveData(mainRptData);
#endif

        // pre-processing reported lane data
        preprocessRptData(mainRptData, mainGpsData);

		uint32 sampleInterval = 20;
		_secRptDataObj.segMultiRptData(mainRptData,sampleInterval,secData);
		if(hasSideLane)
		{
			_secRptDataObj.segMultiRptData(sideRptData, sampleInterval, sideSecData);
		}

#if SAVE_DATA_ON
		saveData(secData);
#endif

	}

	vector<rptLaneLink> laneLink;
	laneLink.clear();

    // iterate each section
    list<reportSectionData>::iterator secItor = secData.begin();
    while (secItor != secData.end())
    {
        // current reported section ID
        _curSecId = secItor->sectionId;

		bool bCommLinesMerged = roadSegConfig_gp->getBothSideMergeFlag(_curSecId);

        // merge new data with database
		mergeSectionLane(*secItor, sideSecData, laneLink);
        
        // stitch background database to generate foreground data
        stitchSectionLanes();
        stitchSectionLanes(true);

        // merge two direction shared line
        stitchSharedLines(bCommLinesMerged);

        // remove section overlap
        removeOverlap();

        secItor++;
    }

    jointProcessing(fgData);

	getModifiedSectionId(secData, fgData, modifiedSectionId);

#if VISUALIZATION_ON || SAVE_DATA_ON
    saveData(fgData, true);
#endif // end of visualization or save data

    ReleaseMutex(_hMutexMerging);

    return true;
}


bool CRoadVecGen3::roadSectionsGen(OUT list<list<vector<point3D_t>>> &fgData)
{
    WaitForSingleObject(_hMutexMerging, INFINITE);

    //bool bCommLinesMerged = false;
//#if defined(_DE_LEHRE_VIDEO)
//    bCommLinesMerged = true;
//#endif

    // generate foreground database data
    list<backgroundSectionData>::iterator bgSecItor = _bgDatabaseList.begin();
    while (bgSecItor != _bgDatabaseList.end())
    {
        _curSecId = bgSecItor->sectionId;
		bool bCommLinesMerged = roadSegConfig_gp->getBothSideMergeFlag(_curSecId);
		
        stitchSectionLanes();
        stitchSectionLanes(true);

        // merge two direction shared line
        stitchSharedLines(bCommLinesMerged);

        // remove section overlap and output data
        removeOverlap();

        bgSecItor++;
    }

    jointProcessing(fgData);

    ReleaseMutex(_hMutexMerging);
    return true;
}

/*
void CRoadVecGen3::setSectionConfigPath(IN string                  filename,
    OUT list<segAttributes_t> &segConfigList)
{
    //_configPath = filename;

    // read section configuration file
    //readSecConfig(segConfigList);

    // initialize database
    initDatabase();

    // calculate section rotation angle and X data range
    calcRotAngleAndRange();

    // check whether road is a circle
    checkCircleRoad();
}
*/
bool CRoadVecGen3::loadDefaultSegData(IN uint32 segId, IN string filename)
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


uint32 CRoadVecGen3::getSectionId(IN point3D_t p)
{


    return 0;
}


void CRoadVecGen3::getShitDist(OUT vector<double> &dist)
{

}


void CRoadVecGen3::resetDatabase()
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

void CRoadVecGen3::getBgRoadVec(OUT list<backgroundSectionData> &bgVecOut,
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

void CRoadVecGen3::setBgRoadVec(IN list<backgroundSectionData> &bgVecIn,
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

/*
void CRoadVecGen3::readSecConfig(OUT list<segAttributes_t> &segCfgList)
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
*/

void CRoadVecGen3::initDatabase()
{
    // when initialing database should be empty, so add structure directly
    //get all section Id
	vector<int> segId;
	bool getFlag = roadSegConfig_gp->getAllSegIdOrder(segId);
	
	vector<int>::iterator segItor = segId.begin();
	if(getFlag)
	{
		_startSegId = *segItor;
		while (segItor != segId.end())
		{
			backgroundSectionData bgData;
			bgData.sectionId = *segItor;
			_bgDatabaseList.push_back(bgData);
			_bgRevDirList.push_back(bgData);

			foregroundSectionData fgData;
			fgData.sectionId = *segItor;
			_fgDatabaseList.push_back(fgData);
			_fgRevDirList.push_back(fgData);
			_fgAllDirList.push_back(fgData);
			_fgOutputList.push_back(fgData);

			segItor++;
		}
	}
}


void CRoadVecGen3::interpolationSample(IN    vector<point3D_t> &sourceLine,
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
    vector<int> validPntIdx;

    for (int i = 0; i < srcPointCnt; ++i)
    {
        if (1e-5 < abs(sourceLine.at(i).lon) || 1e-5 < abs(sourceLine.at(i).lat))
        {
            validPntIdx.push_back(i);
        }
    }

    if (!validPntIdx.empty())
    {
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

                for(uint32 i = validPntIdx.front(); i < validPntIdx.back(); i++)
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
                    if(abs(sampledLine[index].lon - sourceLine[validPntIdx.front()].lon) <
                        abs(sampledLine[index].lon - sourceLine[validPntIdx.back()].lon))
                    {
                        x0 = validPntIdx.front();
                        x1 = validPntIdx.front();
                    }
                    else
                    {
                        // more than 2 points, use last three points to generate
                        // points not in source line, otherwise use last two points
                        x0 = validPntIdx.back() - 2;
                        x1 = validPntIdx.back() - 1;


                        if(srcPointCnt <= 2)
                        {
                            x0 = validPntIdx.front();
                            x1 = validPntIdx.back();
                        }
                    }
                }

                if (x1 == validPntIdx.front() || x1 == validPntIdx.back())
                {
                    x0 = x1;
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
    }// end !validPntIdx.empty()
}


void CRoadVecGen3::calcRotAngleAndRange()
{
    // check section configuration list
    //if (_segConfigList.empty())
    //{
    //   return;
    //}
	list<vector<point3D_t>> rangPoint;
	roadSegConfig_gp->getAllSegRangPointOrder(rangPoint);

    double x0(0.0), x1(0.0), y0(0.0), y1(0.0), X0Temp(0.0);
    double theta = 0.0;
    double xlimits[SEG_CFG_PNT_NUM] = {0.0, 0.0, 0.0, 0.0};

    // re-sample variables
    point3D_t pointSpl = { 0 };
    int pOrder = 1, numOfSplPnts = 0, stInd = 0, edInd = 0;
    vector<point3D_t> leftSample, rightSample;

    // calculate each rotation angle and X range
    //list<segAttributes_t>::iterator segItor = _segConfigList.begin();
	list<vector<point3D_t>>::iterator segItor = rangPoint.begin();
	
    while (segItor != rangPoint.end())
    {
        if (segItor->empty())
        {
            segItor++;
            continue;
        }

        // clear internal variables
        leftSample.clear();
        rightSample.clear();

        //x0 = segItor->ports[2].lon;
        //x1 = segItor->ports[3].lon;
          x0 = (*segItor)[0].lon;
		  x1 = (*segItor)[3].lon;
        //y0 = segItor->ports[2].lat;
        //y1 = segItor->ports[3].lat;
		  y0 = (*segItor)[0].lat;
		  y1 = (*segItor)[3].lat;

        theta = atan2((y0 - y1), (x0 - x1));

        // push rotation angle to _secRotAngle
        _secRotAngle.push_back(theta);

        complex<double> thetaj(0, -1 * theta);

        for (int i = 0; i < SEG_CFG_PNT_NUM; i++)
        {
            complex<double> yi(0, (*segItor)[i].lat);
            xlimits[i] = real(((*segItor)[i].lon + yi) * exp(thetaj));
        }


        // calculate re-sample points
        numOfSplPnts = (int)((xlimits[3] - xlimits[0]) / SAMPLE_SPACE);
        if (xlimits[0] >= xlimits[3])
        {
            pOrder = -1;
            numOfSplPnts = (int)((xlimits[0] - xlimits[3]) / SAMPLE_SPACE) + 1;
        }

        for (int i = 0; i < numOfSplPnts; i++)
        {
            pointSpl.lon = xlimits[0] + i * pOrder * SAMPLE_SPACE;

            leftSample.push_back(pointSpl);
            rightSample.push_back(pointSpl);
        }

        _secLeftData.push_back(leftSample);
        _secRightData.push_back(rightSample);

        // section body start and end index of sample points
        stInd = (int)(floor((xlimits[1] - xlimits[0]) / (pOrder * SAMPLE_SPACE)));
        edInd = (int)(floor((xlimits[2] - xlimits[0]) / (pOrder * SAMPLE_SPACE))) ;
        if((stInd < 0) ||(edInd > (numOfSplPnts-1)))
		{
			printf("seg start or end Index error\n");
		}

		_secBodyStInd.push_back(stInd);
        _secBodyEdInd.push_back(edInd);

        segItor++;
    }
}

void CRoadVecGen3::lineRotation(IN  vector<point3D_t> &sourceLine,
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

bool CRoadVecGen3::getPrevSegLaneOfTrack(IN vector<rptLaneLink> &laneLink,
	IN rptLaneLink &currSegLane, OUT rptLaneLink &prevSegLane)
{
	prevSegLane = laneLink.back();
	return true;
}

void CRoadVecGen3::judgeDiversion(IN vector<rptLaneLink> &laneLink,
	INOUT rptLaneLink &currSegLane, IN list<vector<point3D_t>> &twoLines, IN bool revDirect)
{
	if (twoLines.empty() || twoLines.front().empty() || twoLines.back().empty() || false == currSegLane.bHasChangeLane)
	{
		currSegLane.turnTo = 0;
		return;
	}

	if (false == revDirect)
	{
		currSegLane.fPointL = twoLines.front().front();
		currSegLane.fPointR = twoLines.back().front();
		currSegLane.bPointL = twoLines.front().back();
		currSegLane.bPointR = twoLines.back().back();
	} 
	else
	{
		currSegLane.fPointL = twoLines.back().back();
		currSegLane.fPointR = twoLines.front().back();
		currSegLane.bPointL = twoLines.back().front();
		currSegLane.bPointR = twoLines.front().front();
	}

	if (laneLink.empty())
	{
		currSegLane.turnTo = 0;
		return;
	}

	rptLaneLink prevSegLane = laneLink.back();

	double turnLeftDist = _secRptDataObj.getLength(prevSegLane.bPointL, currSegLane.fPointR);
	double turnRightDist = _secRptDataObj.getLength(prevSegLane.bPointR, currSegLane.fPointL);

	if(1e-6 > abs(turnLeftDist - turnRightDist))
	{
		currSegLane.turnTo = 0;
	} 
	else
	{
		currSegLane.turnTo = turnLeftDist < turnRightDist ? 1 : 2;
	}

	return;
}

bool CRoadVecGen3::ContinuousLaneLinkTrack(IN vector<rptLaneLink> &laneLink,
	INOUT rptLaneLink &currSegLane, IN bool revDirect, IN vector<int> &matchLaneIdx)
{

	if(1 == matchLaneIdx.size())
	{
		if(true == roadSegConfig_gp->getLineIdOfCurrentLane(currSegLane.segId, matchLaneIdx.front(), 
			revDirect, currSegLane.LlineId, currSegLane.RlineId))
		{
			currSegLane.laneIndex = matchLaneIdx.front();
			currSegLane.ValidFlag = true;
			return true;
		}
		else
		{
			return false;
		}
	}

	if(laneLink.empty())
	{
		return false;
	}

	rptLaneLink prevSegLane = laneLink.back();
	if(false == prevSegLane.ValidFlag)
	{
		return false;
	}

	if (1 == currSegLane.turnTo)
	{
		if(false == roadSegConfig_gp->getLeftOrRightneighbourLineId(prevSegLane.segId, 
			prevSegLane.LlineId, currSegLane.segId, currSegLane.turnTo, 
			currSegLane.LlineId, currSegLane.RlineId))
		{
			return false;
		}
	}
	else if (2 == currSegLane.turnTo)
	{
		if(false == roadSegConfig_gp->getLeftOrRightneighbourLineId(prevSegLane.segId, 
			prevSegLane.RlineId, currSegLane.segId, currSegLane.turnTo, 
			currSegLane.LlineId, currSegLane.RlineId))
		{
			return false;
		}
	}
	else
	{
		if(false == roadSegConfig_gp->getConnectedLineId(prevSegLane.segId, 
			prevSegLane.LlineId, prevSegLane.RlineId, currSegLane.segId, 
			currSegLane.LlineId, currSegLane.RlineId))
		{
			return false;
		}
	}

	if(false == roadSegConfig_gp->getMatchedLaneIdx(currSegLane.segId, 
			currSegLane.LlineId, currSegLane.RlineId, revDirect, currSegLane.laneIndex))
	{
		return false;
	}

	currSegLane.ValidFlag = true;
	matchLaneIdx.push_back(currSegLane.laneIndex);
	return true;
}

bool CRoadVecGen3::mergeSectionLane(IN reportSectionData &reportData, IN list<reportSectionData> &sideSecData,
	IN vector<rptLaneLink> &laneLink)
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
	while (splLItor != _secLeftData.end() && splRItor != _secRightData.end())
	{
		if (curItem  == _curSecId - 1)
		{
			leftSample  = &(*splLItor);
			rightSample = &(*splRItor);
            break;
		}
		splLItor++; splRItor++; curItem++;
	}
	
    if (NULL == leftSample || NULL == rightSample)
    {
        return false;
    }

	// matched lane number
    int numOfSplPnts = leftSample->size();
	vector<int> sourcelaneType, laneType;
	int neigbourLaneType = INVALID_INVALID;
	bool bValid = false;
	bool bHasChangeLane = false;
	list<list<vector<point3D_t>>> sourceTwolines, twolines;

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
		int32 numOfLanes = roadSegConfig_gp->getLaneNumInSeg(reportData.sectionId, grpItor->revDirFlag);
		if(0 == numOfLanes)
		{
			//printf("ERROR: section ID:%d number of lanes is 0.\n", reportData.sectionId);
		}

		if (!grpItor->rptLaneData.empty())
		{
			// reset Y and paint for sample vector
			for (int i = 0; i < numOfSplPnts; i++)
			{
				leftSample->at(i).lat          = 0.0;
				leftSample->at(i).paintFlag    = 0.0;
				leftSample->at(i).paintLength  = 0.0;
				leftSample->at(i).count        = 0;
				rightSample->at(i).lat         = 0.0;
				rightSample->at(i).paintFlag   = 0.0;
				rightSample->at(i).paintLength = 0.0;
				rightSample->at(i).count = 0;
			}

			// data preprocessing
			bValid = LaneDataPreprocessing(grpItor->rptLaneData, *leftSample, *rightSample, grpItor->revDirFlag, sourcelaneType,
				sourceTwolines, bHasChangeLane);

			list<vector<int>> allMatchLaneIdx;

			if(sourcelaneType.size() != sourceTwolines.size() || 0 == sourcelaneType.size())
			{
				//printf("Error: change lane number(%d) is not equal to lane data number(%d).\n", sourcelaneType.size(), sourceTwolines.size());
				rptLaneLink currSegLane;
				currSegLane.bHasChangeLane = bHasChangeLane;
				currSegLane.revDirect = grpItor->revDirFlag;
				currSegLane.segId = reportData.sectionId;
				currSegLane.ValidFlag = false;
				laneLink.push_back(currSegLane);
				return false;
			}

			if (true == grpItor->revDirFlag)
			{
				list<list<vector<point3D_t>>>::reverse_iterator twoLinesItor = sourceTwolines.rbegin();
				while (twoLinesItor != sourceTwolines.rend())
				{
					twolines.push_back(*twoLinesItor);
					twoLinesItor++;
				}
				vector<int>::reverse_iterator TypeItor = sourcelaneType.rbegin();
				while (TypeItor != sourcelaneType.rend())
				{
					laneType.push_back(*TypeItor);
					TypeItor++;
				}
			}
			else
			{
				twolines = sourceTwolines;
				laneType = sourcelaneType;
			}

			list<list<vector<point3D_t>>>::iterator changeLaneItor = twolines.begin();
			int idx = 0;
			vector<int> validIdx;
			vector<int>::iterator laneTypeItor = laneType.begin();
			while (laneTypeItor != laneType.end())
			{
				vector<int> matchLaneIdx;
				if(true == bHasChangeLane || false == getNeighborLaneType(reportData.sectionId, sideSecData, 
					neigbourLaneType, *leftSample, *rightSample))
				{
					 neigbourLaneType = INVALID_INVALID;
				}

				if(false == roadSegConfig_gp->getMatchedLaneIdx(reportData.sectionId, 
					(LINETYPE_MIX)*laneTypeItor, (LINETYPE_MIX)neigbourLaneType, grpItor->revDirFlag, matchLaneIdx))
				{
					//printf("Error: get exact matched lane Failed: segId %d, matched number %d\n", _curSecId, matchLaneIdx.size());
				}

				rptLaneLink currSegLane;
				currSegLane.bHasChangeLane = bHasChangeLane;
				currSegLane.revDirect = grpItor->revDirFlag;
				currSegLane.segId = reportData.sectionId;
				currSegLane.ValidFlag = false;
				judgeDiversion(laneLink, currSegLane, *changeLaneItor, grpItor->revDirFlag);
				if(false == ContinuousLaneLinkTrack(laneLink, currSegLane, grpItor->revDirFlag, matchLaneIdx))
				{
					//printf("Error: get continuous track Failed: segId %d, matched number %d\n", _curSecId, matchLaneIdx.size());
				}
				laneLink.push_back(currSegLane);

				if(1 == matchLaneIdx.size())
				{
					//printf("matched only one lane: segId %d\n", _curSecId);
					validIdx.push_back(idx);
				}

				allMatchLaneIdx.push_back(matchLaneIdx);
				laneTypeItor++;
				changeLaneItor++;
				idx++;
			}

			// if valid, merge with database lane data
			if (bValid && !validIdx.empty() && laneType.size() == twolines.size())
			{
				vector<int> matchedLane;
				list<vector<int>>::iterator allMatchLaneIdxItor = allMatchLaneIdx.begin();
				while (allMatchLaneIdxItor != allMatchLaneIdx.end())
				{
					if (allMatchLaneIdxItor->empty())
					{
						matchedLane.push_back(-1);
					}
					else
					{
						matchedLane.push_back(allMatchLaneIdxItor->front());
					}
					allMatchLaneIdxItor++;
				}

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
						if (false == bHasChangeLane)
						{
							if (i == matchedLane.at(validIdx[0]))
							{
								// increase merged times, just use the first point
								//twolines.front().front().at(0).count = _curSecId;
								//twolines.front().back().at(0).count = _curSecId;
								twolines.front().front().at(0).count++;
								twolines.front().back().at(0).count++;
								SET_LANE_DIR_BIT((twolines.front().front().at(0).count), (grpItor->revDirFlag));
								SET_LANE_DIR_BIT((twolines.front().back().at(0).count), (grpItor->revDirFlag));

								list<vector<point3D_t>> lines;
								lines.push_back(twolines.front().front());
								lines.push_back(twolines.front().back());
								bgDatabaseData->bgSectionData.push_back(lines);
							}
							else
							{
								list<vector<point3D_t>> lines;
								bgDatabaseData->bgSectionData.push_back(lines);
							}
						}
					} // end of lane match iteration
				}
				else
				{
					// if the matched lane number is empty, add it directly
					bgLaneItor = bgDatabaseData->bgSectionData.begin();
					for (uint32 i = 0; i < numOfLanes; i++)
					{
						if(false == bHasChangeLane) // no changing lane area
						{
							if (i == matchedLane.at(validIdx[0]))
							{
								if (bgLaneItor->empty())
								{
									// increase merged times, just use the first point
									//twolines.front().front().at(0).count = _curSecId;
									//twolines.front().back().at(0).count = _curSecId;
									twolines.front().front().at(0).count++;
									twolines.front().back().at(0).count++;
									SET_LANE_DIR_BIT((twolines.front().front().at(0).count), (grpItor->revDirFlag));
									SET_LANE_DIR_BIT((twolines.front().back().at(0).count), (grpItor->revDirFlag));

									list<vector<point3D_t>> lines;
									lines.push_back(twolines.front().front());
									lines.push_back(twolines.front().back());
									*bgLaneItor = lines;
									break;
								}
								else
								{
									// merge data whose paintFlags are not -1
									// other points are processed by translation
									MergForChangLane(twolines.front(), *bgLaneItor);

									// increase merged times, just use the first point
									//bgLaneItor->front().at(0).count = _curSecId;
									//bgLaneItor->back().at(0).count = _curSecId;
									bgLaneItor->front().at(0).count++;
									bgLaneItor->back().at(0).count++;
									SET_LANE_DIR_BIT((bgLaneItor->front().at(0).count), (grpItor->revDirFlag));
									SET_LANE_DIR_BIT((bgLaneItor->back().at(0).count), (grpItor->revDirFlag));
								}
							} // end of matched lane
						}
						else // changing lane areas exist
						{
							int sizeOfValidIdx = validIdx.size();
							list<list<vector<point3D_t>>>::iterator twolinesItor = twolines.begin();
							for(int j = 0; j < sizeOfValidIdx; j++)
							{
								if (i == matchedLane.at(validIdx[j]))
								{
									advance(twolinesItor, validIdx[j]);
                                    if (!bgLaneItor->empty())
									{
										// merge data whose paintFlags are not -1
										// other points are processed by translation
										MergForChangLane(*twolinesItor, *bgLaneItor);

										// increase merged times, just use the first point
										//bgLaneItor->front().at(0).count = _curSecId;
										//bgLaneItor->back().at(0).count = _curSecId;
										bgLaneItor->front().at(0).count++;
										bgLaneItor->back().at(0).count++;
										SET_LANE_DIR_BIT((bgLaneItor->front().at(0).count), (grpItor->revDirFlag));
										SET_LANE_DIR_BIT((bgLaneItor->back().at(0).count), (grpItor->revDirFlag));
									}
								}
								twolinesItor = twolines.begin();
							}
						}

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

bool CRoadVecGen3::mergeSectionLane(IN    reportSectionData     &reportData)
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
    //uint32 numOfLanes = _secLaneNum[_curSecId - 1];

    // matched lane number
    int matchedLane = 0;
    vector<int> laneType;
    bool bValid = false;
    bool bHasChangeLane = false;
    list<list<vector<point3D_t>>> twolines;

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
        int32 numOfLanes = roadSegConfig_gp->getLaneNumInSeg(reportData.sectionId, grpItor->revDirFlag);
		if(0 == numOfLanes)
		{
			//printf("ERROR: section ID:%d number of lanes is 0.\n", reportData.sectionId);
		}

        if (!grpItor->rptLaneData.empty())
        {
            // reset Y and paint for sample vector
            for (int i = 0; i < numOfSplPnts; i++)
            {
                leftSample->at(i).lat          = 0.0;
                leftSample->at(i).paintFlag    = 0.0;
                leftSample->at(i).paintLength  = 0.0;
				leftSample->at(i).count        = 0;
                rightSample->at(i).lat         = 0.0;
                rightSample->at(i).paintFlag   = 0.0;
                rightSample->at(i).paintLength = 0.0;
				rightSample->at(i).count = 0;
            }

            // data preprocessing
            bValid = LaneDataPreprocessing(grpItor->rptLaneData, *leftSample, *rightSample, grpItor->revDirFlag, laneType,
                twolines, bHasChangeLane);

            list<vector<int>> allMatchLaneIdx;

            int idx = 0;
            vector<int> validIdx;
            vector<int>::iterator laneTypeItor = laneType.begin();
            while (laneTypeItor != laneType.end())
            {
                vector<int> matchLaneIdx;
                if(!roadSegConfig_gp->getMatchedLaneIdx(reportData.sectionId, 
                    (LINETYPE_MIX)*laneTypeItor, grpItor->revDirFlag,matchLaneIdx))
                {
                    printf("error: didn't find any matched lanes: segId %d\n", _curSecId);
                }

                if(matchLaneIdx.size() > 1)
                {
                    printf("matched more than one lanes: segId %d\n", _curSecId);
                }

                if(matchLaneIdx.size() == 1)
                {
                    printf("matched only one lane: segId %d\n", _curSecId);
                    validIdx.push_back(idx);
                }
                allMatchLaneIdx.push_back(matchLaneIdx);
                laneTypeItor++;
                idx++;
            }

            // if valid, merge with database lane data
            if (bValid && !validIdx.empty() && laneType.size() == twolines.size())
            {
                vector<int> matchedLane;
                list<vector<int>>::iterator allMatchLaneIdxItor = allMatchLaneIdx.begin();
                while (allMatchLaneIdxItor != allMatchLaneIdx.end())
                {
                    if (allMatchLaneIdxItor->empty())
                    {
                        matchedLane.push_back(-1);
                    }
                    else
                    {
                        matchedLane.push_back(allMatchLaneIdxItor->front());
                    }
                    allMatchLaneIdxItor++;
                }

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
                        if (false == bHasChangeLane)
                        {
                            if (i == matchedLane.at(validIdx[0]))
                            {
                                // increase merged times, just use the first point
                                twolines.front().front().at(0).count = _curSecId;
                                twolines.front().back().at(0).count = _curSecId;
                                //twolines.front().front().at(0).count++;
                                //twolines.front().back().at(0).count++;
                                SET_LANE_DIR_BIT((twolines.front().front().at(0).count), (grpItor->revDirFlag));
                                SET_LANE_DIR_BIT((twolines.front().back().at(0).count), (grpItor->revDirFlag));

                                list<vector<point3D_t>> lines;
                                lines.push_back(twolines.front().front());
                                lines.push_back(twolines.front().back());
                                bgDatabaseData->bgSectionData.push_back(lines);
                            }
                            else
                            {
                                list<vector<point3D_t>> lines;
                                bgDatabaseData->bgSectionData.push_back(lines);
                            }
                        }
                    } // end of lane match iteration
                }
                else
                {
                    // if the matched lane number is empty, add it directly
                    bgLaneItor = bgDatabaseData->bgSectionData.begin();
                    for (uint32 i = 0; i < numOfLanes; i++)
                    {
                        if(false == bHasChangeLane) // no changing lane area
                        {
                            if (i == matchedLane.at(validIdx[0]))
                            {
                                if (bgLaneItor->empty())
                                {
                                    // increase merged times, just use the first point
                                    twolines.front().front().at(0).count = _curSecId;
                                    twolines.front().back().at(0).count = _curSecId;
                                    //twolines.front().front().at(0).count++;
                                    //twolines.front().back().at(0).count++;
                                    SET_LANE_DIR_BIT((twolines.front().front().at(0).count), (grpItor->revDirFlag));
                                    SET_LANE_DIR_BIT((twolines.front().back().at(0).count), (grpItor->revDirFlag));

                                    list<vector<point3D_t>> lines;
                                    lines.push_back(twolines.front().front());
                                    lines.push_back(twolines.front().back());
                                    *bgLaneItor = lines;
                                    break;
                                }
                                else
                                {
                                    // merge data whose paintFlags are not -1
                                    // other points are processed by translation
                                    MergForChangLane(twolines.front(), *bgLaneItor);

                                    // increase merged times, just use the first point
                                    bgLaneItor->front().at(0).count = _curSecId;
                                    bgLaneItor->back().at(0).count = _curSecId;
									//bgLaneItor->front().at(0).count++;
									//bgLaneItor->back().at(0).count++;
                                    SET_LANE_DIR_BIT((bgLaneItor->front().at(0).count), (grpItor->revDirFlag));
                                    SET_LANE_DIR_BIT((bgLaneItor->back().at(0).count), (grpItor->revDirFlag));
                                }
                            } // end of matched lane
                        }
                        else // changing lane areas exist
                        {
                            int sizeOfValidIdx = validIdx.size();
                            list<list<vector<point3D_t>>>::iterator twolinesItor = twolines.begin();
                            for(int j = 0; j < sizeOfValidIdx; j++)
                            {
                                if (i == matchedLane.at(validIdx[j]))
                                {
                                    if (!bgLaneItor->empty())
                                    {
                                        // merge data whose paintFlags are not -1
                                        // other points are processed by translation
                                        MergForChangLane(*twolinesItor, *bgLaneItor);

                                        // increase merged times, just use the first point
								bgLaneItor->front().at(0).count = _curSecId;
								bgLaneItor->back().at(0).count = _curSecId;
                                //bgLaneItor->front().at(0).count++;
                                //bgLaneItor->back().at(0).count++;
                                        SET_LANE_DIR_BIT((bgLaneItor->front().at(0).count), (grpItor->revDirFlag));
                                        SET_LANE_DIR_BIT((bgLaneItor->back().at(0).count), (grpItor->revDirFlag));
                                    }
                                }
                                twolinesItor++;
                            }
                        }

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

int CRoadVecGen3::getContinousValidIndex(INOUT vector<int> &validLaneInd)
{
	bool findFirstValid = false;
	int count = 0, i = 0;
	for(i = 0; i < validLaneInd.size(); i++)
	{
		if(VALID_LANE_FLAG == validLaneInd[i])	
		{
			if(false == findFirstValid)
			{
				findFirstValid = true;
            }
			count++;
		}
		else
		{
			if(true == findFirstValid)
			{
				break;	
			}			
		}
	}
	
	for( ; i < validLaneInd.size(); i++)
	{
		validLaneInd[i] = INVALID_LANE_FLAG;		
	}
	return count;
}

bool CRoadVecGen3::stitchSectionLanes(IN bool bRevDir/* = false*/)
{
    // number of sections
    //int numOfSeg = _segConfigList.size();
	int numOfSeg = roadSegConfig_gp->getSegNum();
    int numOfBgSeg = _bgDatabaseList.size();
    int numOfBgRevSeg = _bgRevDirList.size();

    //if ((numOfBgSeg != numOfSeg) ||
    //    (numOfSeg != numOfBgRevSeg) || (numOfBgSeg != numOfBgRevSeg))
    //{
    //    return false;
    //}

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
		numOfLanes = roadSegConfig_gp->getLaneNumInSeg(_curSecId, bRevDir);
		
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
				/*
                // current section has max supported lanes(3)
                // if lane 1 and 3 are valid, keep lane 1 and discard lane 3
                if ((MAX_SUPPORTED_LANES == validLaneInd.size()) &&
                    (MAX_SUPPORTED_LANES - 1 == numOfValidLanes) &&
                    (INVALID_LANE_FLAG == validLaneInd[DASH_DASH]))
                {
                    numOfValidLanes = 1;
                    validLaneInd[MAX_SUPPORTED_LANES - 1] = INVALID_LANE_FLAG;
                }
			    */
				
				numOfValidLanes = getContinousValidIndex(validLaneInd);
				if(0 >= numOfValidLanes)
				{
					return false;	
				}
				
                // iterate valid lane to merge common lines
                list<vector<point3D_t>> dblines;
                vector<int>::iterator laneIndItor = validLaneInd.begin();
                laneItor = bgSegData->bgSectionData.begin();

                while (laneItor != bgSegData->bgSectionData.end())
                {
                    if (VALID_LANE_FLAG == *laneIndItor)
                    {
                        vector<point3D_t> leftline = laneItor->front();
                        vector<point3D_t> rightline = laneItor->back();

                        polynomialFitting(leftline, leftline, DEFAULT_DEGREE);
                        polynomialFitting(rightline, rightline, DEFAULT_DEGREE);

                        dblines.push_back(leftline);
                        dblines.push_back(rightline);
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
                else if (3 == numOfValidLanes)
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
				else
				{
					// five lines
                    list<vector<double>>::iterator distItor = distlines.begin();
                    vector<double> d0 = *distItor ++;
                    vector<double> d1 = *distItor ++;
                    vector<double> d2 = *distItor ++;
                    vector<double> d3 = *distItor ++;
                    vector<double> d4 = *distItor ++;
                    vector<double> d5 = *distItor ++;
					list<vector<point3D_t>>::iterator midItor = midlines.begin();
                    vector<point3D_t> rotline;
                    vector<point3D_t> linein;

                    // 1st line
                    for (uint32 i = 0; i < dblines.front().size(); i++)
                    {
                        curPnt.lon = dblines.front().at(i).lon;
                        curPnt.lat = dblines.front().at(i).lat + d0[i] + d2[i] + d4[i];
                        curPnt.paintFlag = dblines.front().at(i).paintFlag;
                        curPnt.paintLength = dblines.front().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

                    // 2nd line
                    for (uint32 i = 0; midItor != midlines.end() && i < (*midItor).size(); i++)
                    {
                        curPnt.lon = midItor->at(i).lon;
                        curPnt.lat = midItor->at(i).lat + d2[i] + d4[i];
                        curPnt.paintFlag = midItor->at(i).paintFlag;
                        curPnt.paintLength = midItor->at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

                    // 3rd line
					midItor++;
                    for (uint32 i = 0; midItor != midlines.end() && i < (*midItor).size(); i++)
                    {
                        curPnt.lon = midItor->at(i).lon;
                        curPnt.lat = midItor->at(i).lat + d1[i] + d4[i];
                        curPnt.paintFlag = midItor->at(i).paintFlag;
                        curPnt.paintLength = midItor->at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

                    // 4th line
					midItor++;
                    for (uint32 i = 0; midItor != midlines.end() && i < (*midItor).size(); i++)
                    {
                        curPnt.lon = midItor->at(i).lon;
                        curPnt.lat = midItor->at(i).lat + d1[i] + d3[i];
                        curPnt.paintFlag = midItor->at(i).paintFlag;
                        curPnt.paintLength = midItor->at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();

					// 5th line
                    for (uint32 i = 0; i < dblines.back().size(); i++)
                    {
                        curPnt.lon = dblines.back().at(i).lon;
                        curPnt.lat = dblines.back().at(i).lat + d1[i] + d3[i] + d5[i];
                        curPnt.paintFlag = dblines.back().at(i).paintFlag;
                        curPnt.paintLength = dblines.back().at(i).paintLength;

                        linein.push_back(curPnt);
                    }
                    fgSegData->fgSectionData.push_back(linein);
                    linein.clear();					
				}// 4 lanes

#if UNIFORM_SOLID_PAINT
                unifySolidPaint(*bgSegData, *fgSegData, numOfLanes, bRevDir);
#endif

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

int CRoadVecGen3::getNumOfValidLanesInCurrBgSeg(IN backgroundSectionData &bgSegData)
{
	int numOfValidLanes = 0;

	list<list<vector<point3D_t>>>::iterator laneItor = bgSegData.bgSectionData.begin();
	while (laneItor != bgSegData.bgSectionData.end())
	{
		if (!laneItor->empty())
		{
			numOfValidLanes++;
		}

		laneItor++;
	}

	return numOfValidLanes;
}

void CRoadVecGen3::adjustForwardAndReverseGap(IN bool bNeedMerge, foregroundSectionData *fgSegAllData)
{
	int reverseLaneNum = roadSegConfig_gp->getLaneNumInSeg(_curSecId,true);
	int forwardLaneNum = roadSegConfig_gp->getLaneNumInSeg(_curSecId,false);
	if(bNeedMerge || NULL == fgSegAllData || fgSegAllData->fgSectionData.empty() || 0 == reverseLaneNum || 0 == forwardLaneNum)
	{
		return;
	}

	if((reverseLaneNum + 1 + forwardLaneNum + 1) != fgSegAllData->fgSectionData.size())
	{
		return;
	}

    int segId = fgSegAllData->sectionId;
	bool findFirst = false;
	point3D_t leftPoint, rightPoint;
	list<vector<point3D_t>>::iterator tempItor = fgSegAllData->fgSectionData.end();
	list<vector<point3D_t>>::iterator itor = fgSegAllData->fgSectionData.begin();
	while(itor != fgSegAllData->fgSectionData.end())
	{	
		if(false == findFirst && !itor->empty())
		{
			leftPoint = itor->at(_secBodyStInd[segId - 1]);
			findFirst = true;
		}

		if(true == findFirst && !itor->empty())
		{
			tempItor = itor;
		}
		itor++;
	}

	if(true == findFirst && tempItor != fgSegAllData->fgSectionData.end())
	{
		rightPoint = tempItor->at(_secBodyStInd[segId - 1]);
	}
	else
	{
		return;
	}
	
	list<vector<point3D_t>>::reverse_iterator revRightTtor = fgSegAllData->fgSectionData.rbegin();
	list<vector<point3D_t>>::iterator forwardLeftTtor = fgSegAllData->fgSectionData.begin();
	if(reverseLaneNum + 1 < fgSegAllData->fgSectionData.size() && forwardLaneNum + 1 < fgSegAllData->fgSectionData.size())
	{
		advance(revRightTtor, forwardLaneNum + 1);
		advance(forwardLeftTtor, reverseLaneNum + 1);
	}
	else
	{
		return;
	}

	//find the rigntmost non-empty line of reverse direction 
	list<vector<point3D_t>>::reverse_iterator revLineItor = revRightTtor;
	while(revLineItor != fgSegAllData->fgSectionData.rend())
	{
		if(!revLineItor->empty())
		{
			break;
		}
		revLineItor++;
	}

	//find the leftmost non-empty line of forward direction 
	list<vector<point3D_t>>::iterator forwardLineItor = forwardLeftTtor;
	while(forwardLineItor != fgSegAllData->fgSectionData.end())
	{
		if(!forwardLineItor->empty())
		{
			break;
		}
		forwardLineItor++;		
	}

	if(revLineItor == fgSegAllData->fgSectionData.rend() || forwardLineItor == fgSegAllData->fgSectionData.end() 
		|| revLineItor->size() != forwardLineItor->size())
	{
		return;
	}

	double moveDist = 0;
	if(leftPoint.lat < rightPoint.lat)
	{	
        // only check values within body
		for(int i = _secBodyStInd[segId - 1]; i <= _secBodyEdInd[segId - 1]; i++)
		{
			if((*revLineItor)[i].lat > (*forwardLineItor)[i].lat)
			{
				if(((*revLineItor)[i].lat - (*forwardLineItor)[i].lat) > moveDist)
				{
					moveDist = (*revLineItor)[i].lat - (*forwardLineItor)[i].lat;
				}
			}
		}

		if(0 == moveDist)
		{
			return;
		}
		moveDist = moveDist / (double) 2;

		//revese direction lines lat decrease
		revLineItor = revRightTtor;
		while(revLineItor != fgSegAllData->fgSectionData.rend())
		{
			for(int i = 0; i < revLineItor->size(); i++)
			{
				(*revLineItor)[i].lat -= moveDist;
			}
			revLineItor++;
		}

		//forward direction lines lat increase
		forwardLineItor = forwardLeftTtor;
		while(forwardLineItor != fgSegAllData->fgSectionData.end())
		{
			for(int i = 0; i < forwardLineItor->size(); i++)
			{
				(*forwardLineItor)[i].lat += moveDist;
			}
			forwardLineItor++;		
		}
	}
	else
	{
        // only check values within body
		for(int i = _secBodyStInd[segId - 1]; i <= _secBodyEdInd[segId - 1]; i++)
		{
			if((*revLineItor)[i].lat < (*forwardLineItor)[i].lat)
			{
				if(((*forwardLineItor)[i].lat - (*revLineItor)[i].lat) > moveDist)
				{
					moveDist = (*forwardLineItor)[i].lat - (*revLineItor)[i].lat;
				}
			}
		}

		if(0 == moveDist)
		{
			return;
		}
		moveDist = moveDist / (double) 2;

		//revese direction lines lat increase
		revLineItor = revRightTtor;
		while(revLineItor != fgSegAllData->fgSectionData.rend())
		{
			for(int i = 0; i < revLineItor->size(); i++)
			{
				(*revLineItor)[i].lat += moveDist;
			}
			revLineItor++;
		}

		//forward direction lines lat decrease
		forwardLineItor = forwardLeftTtor;
		while(forwardLineItor != fgSegAllData->fgSectionData.end())
		{
			for(int i = 0; i < forwardLineItor->size(); i++)
			{
				(*forwardLineItor)[i].lat -= moveDist;
			}
			forwardLineItor++;		
		}
	}

	return;
}

bool CRoadVecGen3::stitchSharedLines(IN bool bNeedMerge/* = false*/)
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
	
	int reverseLaneNum = roadSegConfig_gp->getLaneNumInSeg(_curSecId,true);
	int forwardLaneNum = roadSegConfig_gp->getLaneNumInSeg(_curSecId,false);
	int RevValidLanesNum = getNumOfValidLanesInCurrBgSeg(*bgSegRevData);
	int ValidLanesNum = getNumOfValidLanesInCurrBgSeg(*bgSegData);

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
        //if (!bgSegRevData->bgSectionData.empty())
		if(0 != RevValidLanesNum)
        {
            int numOfRevUsedLanes = 0, numOfRevUsedLines = 0;
			
            list<list<vector<point3D_t>>>::iterator laneRevIt = bgSegRevData->bgSectionData.begin();
            while (laneRevIt != bgSegRevData->bgSectionData.end()  && laneRevIt->empty())
            {
                vector<point3D_t> line;
                fgSegAllData->fgSectionData.push_back(line);

                numOfRevUsedLanes++;
                laneRevIt++;
            }

            // check valid foreground lines
            int numOfRevLines = fgSegRevData->fgSectionData.size();
            if (numOfRevUsedLanes + numOfRevLines - 1 == reverseLaneNum)//_secLaneNum[_curSecId - 1])
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

				// for remaining empty lanes
				while (numOfRevUsedLanes + numOfRevUsedLines - 1 < reverseLaneNum)
				{
					vector<point3D_t> line;
					fgSegAllData->fgSectionData.push_back(line);

					numOfRevUsedLanes++;
				}

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
            while (numOfUsedLanes < forwardLaneNum)
            {
                vector<point3D_t> line;
                fgSegAllData->fgSectionData.push_back(line);

                numOfUsedLanes++;
            }
        }
        else
        {
            // if has backward direction database, push empty lanes
            //if (0 != reverseLaneNum && bgSegRevData->bgSectionData.empty())
			if (0 != reverseLaneNum && 0 == RevValidLanesNum)				
            {
                int numOfRevUsedLanes = 0;
                while (numOfRevUsedLanes < reverseLaneNum + 1)
                {
                    vector<point3D_t> line;
                    fgSegAllData->fgSectionData.push_back(line);

                    numOfRevUsedLanes++;
                }
            }

			if(bNeedMerge)
			{
	            // forward direction
				//if (!bgSegData->bgSectionData.empty())
				if (0 != ValidLanesNum)
				{
					if (!bgSegData->bgSectionData.front().empty())
					{
						fgSegAllData->fgSectionData.pop_back();
						list<vector<point3D_t>>::iterator fgLineIt = fgSegData->fgSectionData.begin();
						fgSegAllData->fgSectionData.push_back(*fgLineIt);

						int numOfLines = fgSegData->fgSectionData.size();
						fgLineIt++;
						for (int ii = 0; ii < numOfLines - 1; ii++)
						{	
							fgSegAllData->fgSectionData.push_back(*fgLineIt);	
							fgLineIt++;
						}

						// for empty lane
						int numOfUsedLanes = numOfLines - 1;
						while (numOfUsedLanes < forwardLaneNum)
						{
							vector<point3D_t> line;
							fgSegAllData->fgSectionData.push_back(line);
							numOfUsedLanes++;
						}
					}
					else
					{						
						list<list<vector<point3D_t>>>::iterator laneIt = bgSegData->bgSectionData.begin();
						laneIt++;
						int numOfUsedLanes = 1;
						while (laneIt != bgSegData->bgSectionData.end() && laneIt->empty())
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
						while (numOfUsedLanes + numOfLines - 1 < forwardLaneNum)
						{
							vector<point3D_t> line;
							fgSegAllData->fgSectionData.push_back(line);
							numOfUsedLanes++;
						}					
					}
				}
				else
				{
					int numOfRevUsedLanes = 0;
					while (numOfRevUsedLanes < forwardLaneNum)
					{
						vector<point3D_t> line;
						fgSegAllData->fgSectionData.push_back(line);
						numOfRevUsedLanes++;
					}
				}					
			}
			else
			{
	            // forward direction
				//if (!bgSegData->bgSectionData.empty())
				if (0 != ValidLanesNum)
				{
						int numOfUsedLanes = 0;
						list<list<vector<point3D_t>>>::iterator laneIt = bgSegData->bgSectionData.begin();
						while (laneIt != bgSegData->bgSectionData.end() && laneIt->empty())
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
						while (numOfUsedLanes + numOfLines - 1 < forwardLaneNum)
						{
							vector<point3D_t> line;
							fgSegAllData->fgSectionData.push_back(line);
							numOfUsedLanes++;
						}
				}
				else
				{
					int numOfRevUsedLanes = 0;
					while (0 != forwardLaneNum && numOfRevUsedLanes < forwardLaneNum + 1)
					{
						vector<point3D_t> line;
						fgSegAllData->fgSectionData.push_back(line);
						numOfRevUsedLanes++;
					}
				}				
			}
        }
    }

	adjustForwardAndReverseGap(bNeedMerge, fgSegAllData);

    return true;
}


void CRoadVecGen3::removeOverlap(OUT list<list<vector<point3D_t>>> &fgData)
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

void CRoadVecGen3::removeOverlap()
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
		fgOutSecData->isNewRpt = CURRENT_RPT;

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

            double angle1 = 0.0, angle2 = 0.0, cox1 = 0.0, cox2 = 0.0;
            if (numOfValidLines > 0)
            {
                double c1 = sy1 / numOfValidLines;
                double c2 = sy2 / numOfValidLines;

                angle1 = atan(my1 / (numOfValidLines * step));
                angle2 = atan(my2 / (numOfValidLines * step));

                // start point
                complex<double> theta1(0, angle1);
                complex<double> yj1(0, c1);
                cox1 = real((stx + yj1) * exp(theta1));

                // end point
                complex<double> theta2(0, angle2);
                complex<double> yj2(0, c2);
                cox2 = real((edx + yj2) * exp(theta2));
            }

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
#if 1

bool CRoadVecGen3::getSectionFromInputFgout(IN list<foregroundSectionData> &fgData, IN int SegID, 
	OUT foregroundSectionData **pSegData)	
{
	if(fgData.empty() || NULL == pSegData)
	{
		return false;
	}
	list<foregroundSectionData>::iterator SecItor = fgData.begin();
	while(SecItor != fgData.end())
	{
		if(SegID == SecItor->sectionId)
		{
			*pSegData = &(*SecItor);
			return true;
		}
		SecItor++;
	}
	return false;
}

bool CRoadVecGen3::GetLineFromSegmentOfFgout(IN foregroundSectionData *pSegData, 
						IN int LineID, OUT vector<point3D_t> **pLine)
{
    if (NULL == pSegData || NULL == pLine)
    {
        return false;
    }
	int LineIndex = LineID - 1;
	int lineNum = pSegData->fgSectionData.size();
	if(LineIndex >= lineNum || 0 == lineNum)
	{	
		return false;				
	}

	list<vector<point3D_t>>::iterator lineItor = pSegData->fgSectionData.begin();
	for(int i = 0; i < lineNum; i++)
	{
		if(i == LineIndex && !lineItor->empty())
		{
			*pLine = &(*lineItor);
			return true;
		}
		lineItor++;
	}

	return false;
}

bool CRoadVecGen3::GetLineFromInputFgout(IN list<foregroundSectionData> &fgData, 
						IN int segId, IN int LineID, OUT vector<point3D_t> **pLine)
{	
	foregroundSectionData *pSegData = NULL;
	if(false == getSectionFromInputFgout(fgData, segId, &pSegData))
	{
		return false;
	}

	if(false == GetLineFromSegmentOfFgout(pSegData, LineID, pLine))
	{
		return false;
	}

	return true;
}
#endif
int CRoadVecGen3::getSectionFromOutputList(IN int SegID, OUT foregroundSectionData **pSegData)	
{
    if (NULL == pSegData)
    {
        return false;
    }

	list<foregroundSectionData>::iterator SecItor = _fgOutputList.begin();
	while(SecItor != _fgOutputList.end())
	{
		if(SegID == (*SecItor).sectionId)
		{
			*pSegData = &(*SecItor);
			return true;
		}
		SecItor++;
	}	
	
	return false;
}	

bool CRoadVecGen3::GetLineFromInputSegment(IN foregroundSectionData *SegData, IN int LineID, OUT vector<point3D_t> **pLine)
{
	if(NULL == SegData || SegData->fgSectionData.empty() || NULL == pLine)
	{
		return false;
	}

	int LineIndex = LineID - 1;
	int lineNum = SegData->fgSectionData.size();
	if(LineIndex >= lineNum || 0 == lineNum)
	{	
		printf("ERROR:Get line(SegID:%d LineIndex:%d LineNum:%d) from output list failed.\n", 
			SegData->sectionId, LineIndex, lineNum);
		return false;				
	}

    list<vector<point3D_t>>::iterator lineItor = SegData->fgSectionData.begin();
	for(int i = 0; i < lineNum; i++)
	{
		if(i == LineIndex && !lineItor->empty())
		{
			*pLine = &(*lineItor);
			return true;
		}
		lineItor++;
	}

	return false;
}

bool CRoadVecGen3::GetLineFromOutputList(IN int SegID, IN int LineID, OUT vector<point3D_t> **pLine)
{
	foregroundSectionData *SegData = NULL;	
	if(false == getSectionFromOutputList(SegID, &SegData))
	{
		printf("ERROR:Get section(ID:%d) from outputlist failed.\n", SegID);
		return false;
	}

	if(NULL == SegData)
	{
		printf("ERROR:section ID:%d Line ID:%d SegData is NULL.\n", SegID, LineID);
		return false;
	}
	
	if(false == GetLineFromInputSegment(SegData, LineID, pLine))
	{
		return false;
	}

	return true;
}

void CRoadVecGen3::freePointerArray(vector<point3D_t> **ppCurrLines, int currlines)
{
	if(NULL == ppCurrLines)
	{
		return;
	}
	
	for(int i = 0; i < currlines; i++)
	{
		ppCurrLines[i] = NULL;
	}
	delete [] ppCurrLines;
	ppCurrLines = NULL;	
	return;
}

int CRoadVecGen3::devideMatchLinesIntoSegGroups(IN list<vector<lineConn>> MatchedLines,
												OUT list<vector<lineConn>> MatchedSegment[],
												OUT list<list<vector<lineConn>>> &lines, OUT int &segNum)
{
	int segId[4] = {0};
	list<vector<lineConn>> :: iterator matchItor = MatchedLines.begin();
	while(matchItor != MatchedLines.end())
	{
		vector<lineConn> ::iterator conItor = (*matchItor).begin();
		while (conItor != (*matchItor).end())
		{	
			int i = 0;
			for(i = 0; i < 4; i++)
			{
				if(((*conItor).connSegID == segId[i]) && (0 != segId[i]))
				{
					break;
				}
			}

			if(i < 4)
			{
				conItor++;
				continue;
			}

			for(int i = 0; i < 4; i++)
			{			
				if(0 == segId[i])
				{
					segId[i] = (*conItor).connSegID;
					segNum++;
					break;
				}
			}
			conItor++;
		}
		matchItor++;
	}

	for(int i = 0; i < segNum; i++)
	{	
		foregroundSectionData *SegData = NULL;
		int lineNum = 0;
		if(true == getSectionFromOutputList(segId[i], &SegData))
		{
			lineNum = SegData->fgSectionData.size();
			if(0 == lineNum)
			{
				continue;
			}
		}

		for(int LineInd = 0; LineInd < lineNum; LineInd++)
		{
			vector<lineConn> tmp;
			list<vector<lineConn>> :: iterator vecItor = MatchedLines.begin();
			while(vecItor != MatchedLines.end())
			{
				vector<lineConn> ::iterator connItor = (*vecItor).begin();
				while (connItor != (*vecItor).end())
				{
					if((segId[i] == (*connItor).connSegID) && ((LineInd + 1) == (*connItor).connLineID))
					{
						tmp.push_back(*connItor);	
					}
					connItor++;
				}
				vecItor++;
			}

			MatchedSegment[i].push_back(tmp);
		}
	}

	for(int i = 0; i < segNum; i++)
	{
		lines.push_back(MatchedSegment[i]);
	}

	if(0 == segNum)
	{
		return false;
	}
	
	return true;
}

void CRoadVecGen3::adjacentEndOrStartPoint(IN foregroundSectionData *currData,
										   IN int flag,
										   OUT list<list<vector<lineConn>>> &lines)
{
	if(NULL == currData || currData->fgSectionData.empty())
	{
		return;
	}

	segment_type_e segType;
	if (false == roadSegConfig_gp->getSegmentType(currData->sectionId, segType))
	{
		return;
	}

	if((POINT_TYPE_END == flag && T_ROAD_CROSS == segType && HANDLED == currData->dealEnd)
		|| (POINT_TYPE_START == flag  && T_ROAD_CROSS == segType  && HANDLED == currData->dealStart))
	{
		//CrossTVirtualCon(currData);
		return;
	}

	//current section is not empty
	int matchMinIdx = -1;
	int currlines = 0;
    currlines = currData->fgSectionData.size();
	
	// get matched line info of all lines in current section
	list<vector<lineConn>> MatchedLines;
	if(POINT_TYPE_END == flag)
	{
		if(false == roadSegConfig_gp->getMatchedLinesInfoOfEndPoint(currData->sectionId, MatchedLines , REAL_LINE_CONNECTION))
		{
			printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
			return;						
		}		
	}
	else
	{
		if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(currData->sectionId, MatchedLines , REAL_LINE_CONNECTION))
		{
			printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
			return;						
		}		
	}

	if(MatchedLines.empty())
	{
		printf("section ID:%d has no matchedLine.\n", currData->sectionId);
		return;
	}	

	if(currlines != MatchedLines.size())
	{
		printf("ERROR: section ID:%d line number(%d) is not same with matchedLine number(%d). \n", 
				currData->sectionId, currlines, MatchedLines.size());
		return;
	}	

	int segNum = 0;
	list<vector<lineConn>> MatchedSegment[4];
	if(false == devideMatchLinesIntoSegGroups(MatchedLines, MatchedSegment, lines, segNum))
	{
		//printf("ERROR: devide matched lines into groups section by section Failed.\n");
		return;
	}

	if(1 < segNum)
	{
		if(!MatchedSegment[0].empty())
		{
			adjacentOneSeg(flag, MatchedSegment[0]);
		}
		return;
	}

	vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
	if(NULL == ppCurrLines)
	{
		printf("ERROR:NULL pointer.\n");
		return;
	}

	list<vector<point3D_t>>::iterator lineItor = currData->fgSectionData.begin();
	for (int i = 0; i < currlines; i++)
	{
		ppCurrLines[i] = &(*lineItor);
		lineItor++;
	}

	list<vector<lineConn>> firstMatchSeg;
	for(int i = 0; i < currlines; i++)
	{
		vector<lineConn> tmp;
		list<vector<lineConn>> :: iterator nextSegItor = MatchedSegment[0].begin();
		while(nextSegItor != MatchedSegment[0].end())
		{
			vector<lineConn> ::iterator vecItor = (*nextSegItor).begin();
			while (vecItor != (*nextSegItor).end())
			{
				if((*vecItor).LineID == (i + 1))
				{
					tmp.push_back(*vecItor);					
				}
				vecItor++;
			}
			nextSegItor++;
		}
		firstMatchSeg.push_back(tmp);
	}

	// get end points of current lines, and start points of next lines
	int LineIndex = 0;
	vector<point3D_t> fixPoint, movePoint;
    vector<int> matchedInd;	
	list<vector<lineConn>> :: iterator Itor = firstMatchSeg.begin();
	while(Itor != firstMatchSeg.end())
	{					
		if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
		{
			matchedInd.push_back(UNMATCHED_LINE_FLAG);
			Itor++;
			LineIndex++;
			continue;			
		}
		
		vector<point3D_t> *pNextLine = NULL;
		if(false == GetLineFromOutputList((*Itor).front().connSegID, (*Itor).front().connLineID, &pNextLine) ||
            NULL == pNextLine || pNextLine->empty())
		{
			matchedInd.push_back(UNMATCHED_LINE_FLAG);
			Itor++;
			LineIndex++;
			continue;									
		}
		
		if(STARTPOINT == (*Itor).front().connType)
		{
			fixPoint.push_back((*pNextLine).front());
		}
		else
		{
			fixPoint.push_back((*pNextLine).back());
		}

        findAndErasePointsOverMinDist(*(ppCurrLines[LineIndex]), fixPoint.back(), flag);

        if(POINT_TYPE_END == flag)
        {
            movePoint.push_back(ppCurrLines[LineIndex]->back());
        }
        else
        {
            movePoint.push_back(ppCurrLines[LineIndex]->front());
        }

			
		if(-1 == matchMinIdx)
		{
			matchMinIdx = LineIndex;
		}
			
		matchedInd.push_back(MATCHED_LINE_FLAG);
		Itor++;	
		LineIndex++;
	}

    // if exist matched lines, size of fixPoint and movePoint should be the same
    if (!fixPoint.empty() && !movePoint.empty())
    {
        int index = 0;
		double dx = 0.0, dy = 0.0, ddx = 0.0, ddy = 0.0;

        for (int LineIndex = 0; LineIndex < currlines; LineIndex++)
        {
            // for empty lines, skip it
            if(NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
            {
                continue;
            }

			if(LineIndex < matchMinIdx)
			{
				index = 0;
			}
				
			if ((MATCHED_LINE_FLAG == matchedInd[LineIndex])&&(LineIndex != matchMinIdx))
			{
				index++;
			}

            if (UNMATCHED_LINE_FLAG == matchedInd[LineIndex])
            {
                findAndErasePointsOverMinDist(*(ppCurrLines[LineIndex]), fixPoint[index], flag);
            }

            if ((0 == LineIndex || LineIndex == (currlines - 1)) ||
                (index == fixPoint.size() - 1) ||
                (MATCHED_LINE_FLAG == matchedInd[LineIndex]))
            {
                dx = fixPoint[index].lon - movePoint[index].lon;
                dy = fixPoint[index].lat - movePoint[index].lat;
            }
            else
            {
                // for unmatched lines, if it's between two matched lines
                int tmpPrevInd = LineIndex-1, tmpNextInd = LineIndex+1;
                while (tmpPrevInd >= 0 && matchedInd[tmpPrevInd] == UNMATCHED_LINE_FLAG) { tmpPrevInd--; }
                while (tmpNextInd < currlines && matchedInd[tmpNextInd] == UNMATCHED_LINE_FLAG) { tmpNextInd++; }

                int prevInd = 0, nextInd = 0;
                for (int i = 0; i < currlines; i++)
                {
                    if ((MATCHED_LINE_FLAG == matchedInd[i]))
                    {
                        if (i == tmpPrevInd)
                        {
                            break;
                        }
                        prevInd++;
                    }
                }

                for (int i = 0; i < currlines; i++)
                {
                    if ((MATCHED_LINE_FLAG == matchedInd[i]))
                    {
                        if (i == tmpNextInd)
                        {
                            break;
                        }
                        nextInd++;
                    }
                }

                int numOfPnts = fixPoint.size();
                if (prevInd < numOfPnts && nextInd < numOfPnts)
                {
                    dx = 0.5 * (fixPoint[prevInd].lon - movePoint[prevInd].lon) +
                        0.5 * (fixPoint[nextInd].lon - movePoint[nextInd].lon);
                    dy = 0.5 * (fixPoint[prevInd].lat - movePoint[prevInd].lat) +
                        0.5 * (fixPoint[nextInd].lat - movePoint[nextInd].lat);
                }
                else if (prevInd < numOfPnts)
                {
                    dx = fixPoint[prevInd].lon - movePoint[prevInd].lon;
                    dy = fixPoint[prevInd].lat - movePoint[prevInd].lat;
                }
                else if (nextInd < numOfPnts)
                {
                    dx = fixPoint[nextInd].lon - movePoint[nextInd].lon;
                    dy = fixPoint[nextInd].lat - movePoint[nextInd].lat;
                }
                else
                {
                    dx = fixPoint[index].lon - movePoint[index].lon;
                    dy = fixPoint[index].lat - movePoint[index].lat;
                }
            }

            // for matched lines
            int numOfHalfPnts = ppCurrLines[LineIndex]->size() / 2;
			int remainder = ppCurrLines[LineIndex]->size() & (0x1);
            double ddx = dx / numOfHalfPnts;
            double ddy = dy / numOfHalfPnts;
			if(POINT_TYPE_END == flag)
			{
				for (int pi = 0; pi < numOfHalfPnts; pi++)
				{
					ppCurrLines[LineIndex]->at(pi + numOfHalfPnts + remainder).lon += (pi + 1)* ddx;
					ppCurrLines[LineIndex]->at(pi + numOfHalfPnts + remainder).lat += (pi + 1)* ddy;					
				}
			}
			else
			{
				for (int pi = 0; pi < numOfHalfPnts; pi++)
				{
					ppCurrLines[LineIndex]->at(pi).lon += (dx - pi * ddx);
					ppCurrLines[LineIndex]->at(pi).lat += (dy - pi * ddy);					
				}				
			}

		}
    }
		
	freePointerArray(ppCurrLines, currlines);
	return;
}

void CRoadVecGen3::findAndErasePointsOverMinDist(IN vector<point3D_t> &line,
    IN point3D_t &fixPoint,
    IN int flag)
{
    if (!line.empty())
    {
        int numOfPnts = line.size();
        int minDistInd = 0;
        double minDist = DBL_MAX;
        for (int i = 0; i < numOfPnts; i++)
        {
            double dx = line.at(i).lon - fixPoint.lon;
            double dy = line.at(i).lat - fixPoint.lat;
            double dist = dx * dx + dy * dy;

            if (dist < minDist)
            {
                minDist = dist;
                minDistInd = i;
            }
        }

        if(POINT_TYPE_END == flag)
        {
            if (minDistInd < numOfPnts - 1)
            {
                vector<point3D_t>::iterator itorPnt = line.begin();
                advance(itorPnt, minDistInd + 1);
                line.erase(itorPnt, line.end());
            }
        }
        else
        {
            if (minDistInd > 0)
            {
                vector<point3D_t>::iterator itorPnt = line.begin();
                advance(itorPnt, minDistInd);
                line.erase(line.begin(), itorPnt);
            }
        }
    }
}


void CRoadVecGen3::CrossTVirtualCon(IN foregroundSectionData *currData)
{
    if (NULL == currData)
    {
        return;
    }

	//current section is not empty
	int currlines = 0;
	currlines = currData->fgSectionData.size();

	// get matched line info of all lines in current section
	vector<uint32> prevId, nextId;
	if(false == roadSegConfig_gp->getPrevSegId(currData->sectionId, prevId) || false == roadSegConfig_gp->getNextSegId(currData->sectionId, nextId))
	{
		return;
	}

	if ((1 >= prevId.size() && 1 >= nextId.size()) || (1 < prevId.size() && 1 < nextId.size()))
	{
		return;
	}

	list<vector<lineConn>> virtualCon;
	if (1 < nextId.size())
	{
		if(false == roadSegConfig_gp->getMatchedLinesInfoOfEndPoint(currData->sectionId, virtualCon, VIRTUAL_LINE_CONNECTION))
		{
			printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
			return;						
		}		

		if(virtualCon.empty() || currlines != virtualCon.size())
		{
			return;
		}	

		vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
		if(NULL == ppCurrLines)
		{
			printf("ERROR:NULL pointer.\n");
			return;
		}

		list<vector<point3D_t>>::iterator lineItor = currData->fgSectionData.begin();
		for (int i = 0; i < currlines; i++)
		{
			ppCurrLines[i] = &(*lineItor);
			lineItor++;
		}

		int LineIndex = 0;	
		list<vector<lineConn>> :: iterator Itor = virtualCon.begin();
		while(Itor != virtualCon.end())
		{					
			if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
			{
				Itor++;
				LineIndex++;
				continue;			
			}

			vector<point3D_t> *pNextLine = NULL;
			if(false == GetLineFromOutputList((*Itor).front().connSegID, (*Itor).front().connLineID, &pNextLine) ||
                NULL == pNextLine || pNextLine->empty())
			{
				Itor++;
				LineIndex++;
				continue;									
			}

			point3D_t currPoint, nextPoint, midPoint;
			currPoint = ppCurrLines[LineIndex]->back();
			if(STARTPOINT == (*Itor).front().connType)
			{
				nextPoint = (*pNextLine).front();
			}
			else
			{
				nextPoint = (*pNextLine).back();
			}

			midPoint.lat = (currPoint.lat + nextPoint.lat) * 0.5;
			midPoint.lon = (currPoint.lon + nextPoint.lon) * 0.5;
			midPoint.alt = 0.0;
			midPoint.count = 0.0;
			midPoint.paintFlag = 0.0;
			midPoint.paintLength = 0.0;
			ppCurrLines[LineIndex]->push_back(midPoint);
			if(STARTPOINT == (*Itor).front().connType)
			{
				(*pNextLine).insert((*pNextLine).begin(), midPoint);
			}
			else
			{
				(*pNextLine).push_back(midPoint);
			}

			Itor++;
			LineIndex++;
		}

		freePointerArray(ppCurrLines, currlines);
		return;
	}

	if (1 < prevId.size())
	{
		if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(currData->sectionId, virtualCon, VIRTUAL_LINE_CONNECTION))
		{
			printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
			return;						
		}		

		if(virtualCon.empty() || currlines != virtualCon.size())
		{
			return;
		}	

		vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
		if(NULL == ppCurrLines)
		{
			printf("ERROR:NULL pointer.\n");
			return;
		}

		list<vector<point3D_t>>::iterator lineItor = currData->fgSectionData.begin();
		for (int i = 0; i < currlines; i++)
		{
			ppCurrLines[i] = &(*lineItor);
			lineItor++;
		}

		int LineIndex = 0;	
		list<vector<lineConn>> :: iterator Itor = virtualCon.begin();
		while(Itor != virtualCon.end())
		{					
			if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
			{
				Itor++;
				LineIndex++;
				continue;			
			}

			vector<point3D_t> *pNextLine = NULL;
			if(false == GetLineFromOutputList((*Itor).front().connSegID, (*Itor).front().connLineID, &pNextLine) ||
                NULL == pNextLine || pNextLine->empty())
			{
				Itor++;
				LineIndex++;
				continue;									
			}

			point3D_t currPoint, nextPoint, midPoint;
			currPoint = ppCurrLines[LineIndex]->front();
			if(STARTPOINT == (*Itor).front().connType)
			{
				nextPoint = (*pNextLine).front();
			}
			else
			{
				nextPoint = (*pNextLine).back();
			}

			if (currPoint.lat == nextPoint.lat && currPoint.lon == nextPoint.lon)
			{
				Itor++;
				LineIndex++;
				continue;	
			}

			midPoint.lat = (currPoint.lat + nextPoint.lat) * 0.5;
			midPoint.lon = (currPoint.lon + nextPoint.lon) * 0.5;
			midPoint.alt = 0.0;
			midPoint.count = 0.0;
			midPoint.paintFlag = 0.0;
			midPoint.paintLength = 0.0;
			//ppCurrLines[LineIndex]->push_back(midPoint);
			ppCurrLines[LineIndex]->insert(ppCurrLines[LineIndex]->begin(), midPoint);
			if(STARTPOINT == (*Itor).front().connType)
			{
				(*pNextLine).insert((*pNextLine).begin(), midPoint);
			}
			else
			{
				(*pNextLine).push_back(midPoint);
			}

			Itor++;
			LineIndex++;
		}

		freePointerArray(ppCurrLines, currlines);
		return;
	}
}

void CRoadVecGen3::CrossTVirtualCon(IN int segID, IN foregroundSectionData &segData, 
			INOUT list<foregroundSectionData> &fgData, INOUT vector<int> &handledSeg)
{
	//current section is not empty
	int currlines = 0;
	currlines = segData.fgSectionData.size();

	// get matched line info of all lines in current section
	vector<uint32> prevId, nextId;
	if(false == roadSegConfig_gp->getPrevSegId(segID, prevId) || false == roadSegConfig_gp->getNextSegId(segID, nextId))
	{
		return;
	}

	if ((1 >= prevId.size() && 1 >= nextId.size()) || (1 < prevId.size() && 1 < nextId.size()))
	{
		return;
	}

	list<vector<lineConn>> virtualCon;
	if (1 < nextId.size())
	{
		if(false == roadSegConfig_gp->getMatchedLinesInfoOfEndPoint(segID, virtualCon, VIRTUAL_LINE_CONNECTION))
		{
			printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
			return;						
		}		

		if(virtualCon.empty() || currlines != virtualCon.size())
		{
			return;
		}	

		vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
		if(NULL == ppCurrLines)
		{
			printf("ERROR:NULL pointer.\n");
			return;
		}

		list<vector<point3D_t>>::iterator lineItor = segData.fgSectionData.begin();
		for (int i = 0; i < currlines; i++)
		{
			ppCurrLines[i] = &(*lineItor);
			lineItor++;
		}

		int LineIndex = 0;	
		list<vector<lineConn>> :: iterator Itor = virtualCon.begin();
		while(Itor != virtualCon.end())
		{					
			if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
			{
				Itor++;
				LineIndex++;
				continue;			
			}

			bool findSame = false;
			for(int i = 0; i < handledSeg.size(); i++)
			{
				if((*Itor).front().connSegID == handledSeg[i])
				{
					findSame = true;
					break;
				}
			}
			if(true == findSame)
			{
				Itor++;
				LineIndex++;
				continue;
			}

			vector<point3D_t> *pNextLine = NULL;
			if(false == GetLineFromInputFgout(fgData, (*Itor).front().connSegID, (*Itor).front().connLineID, &pNextLine) ||
                NULL == pNextLine || pNextLine->empty())
			{
				Itor++;
				LineIndex++;
				continue;									
			}

			point3D_t currPoint, nextPoint, midPoint;
			currPoint = ppCurrLines[LineIndex]->back();
			if(STARTPOINT == (*Itor).front().connType)
			{
				nextPoint = (*pNextLine).front();
			}
			else
			{
				nextPoint = (*pNextLine).back();
			}

			midPoint.lat = (currPoint.lat + nextPoint.lat) * 0.5;
			midPoint.lon = (currPoint.lon + nextPoint.lon) * 0.5;
			midPoint.alt = 0.0;
			midPoint.count = 0.0;
			midPoint.paintFlag = 0.0;
			midPoint.paintLength = 0.0;
			ppCurrLines[LineIndex]->push_back(midPoint);
			if(STARTPOINT == (*Itor).front().connType)
			{
				(*pNextLine).insert((*pNextLine).begin(), midPoint);
			}
			else
			{
				(*pNextLine).push_back(midPoint);
			}

			Itor++;
			LineIndex++;
		}

		freePointerArray(ppCurrLines, currlines);
		return;
	}

	if (1 < prevId.size())
	{
		if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(segID, virtualCon, VIRTUAL_LINE_CONNECTION))
		{
			printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
			return;						
		}		

		if(virtualCon.empty() || currlines != virtualCon.size())
		{
			return;
		}	

		vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
		if(NULL == ppCurrLines)
		{
			printf("ERROR:NULL pointer.\n");
			return;
		}

		list<vector<point3D_t>>::iterator lineItor = segData.fgSectionData.begin();
		for (int i = 0; i < currlines; i++)
		{
			ppCurrLines[i] = &(*lineItor);
			lineItor++;
		}

		int LineIndex = 0;	
		list<vector<lineConn>> :: iterator Itor = virtualCon.begin();
		while(Itor != virtualCon.end())
		{					
			if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
			{
				Itor++;
				LineIndex++;
				continue;			
			}

			bool findSame = false;
			for(int i = 0; i < handledSeg.size(); i++)
			{
				if((*Itor).front().connSegID == handledSeg[i])
				{
					findSame = true;
					break;
				}
			}
			if(true == findSame)
			{
				Itor++;
				LineIndex++;
				continue;
			}

			vector<point3D_t> *pNextLine = NULL;
			if(false == GetLineFromInputFgout(fgData, (*Itor).front().connSegID, (*Itor).front().connLineID, &pNextLine) ||
                NULL == pNextLine || pNextLine->empty())
			{
				Itor++;
				LineIndex++;
				continue;									
			}

			point3D_t currPoint, nextPoint, midPoint;
			currPoint = ppCurrLines[LineIndex]->front();
			if(STARTPOINT == (*Itor).front().connType)
			{
				nextPoint = (*pNextLine).front();
			}
			else
			{
				nextPoint = (*pNextLine).back();
			}

			if (currPoint.lat == nextPoint.lat && currPoint.lon == nextPoint.lon)
			{
				Itor++;
				LineIndex++;
				continue;	
			}

			midPoint.lat = (currPoint.lat + nextPoint.lat) * 0.5;
			midPoint.lon = (currPoint.lon + nextPoint.lon) * 0.5;
			midPoint.alt = 0.0;
			midPoint.count = 0.0;
			midPoint.paintFlag = 0.0;
			midPoint.paintLength = 0.0;
			ppCurrLines[LineIndex]->insert(ppCurrLines[LineIndex]->begin(), midPoint);
			if(STARTPOINT == (*Itor).front().connType)
			{
				(*pNextLine).insert((*pNextLine).begin(), midPoint);
			}
			else
			{
				(*pNextLine).push_back(midPoint);
			}

			Itor++;
			LineIndex++;
		}

		freePointerArray(ppCurrLines, currlines);
		return;
	}
}

bool CRoadVecGen3::findIndexPairOfMinDistancePoints(IN vector<point3D_t> &currLine, 
	IN vector<point3D_t> &nextLine, OUT int &minCurrInd, OUT int &minNextInd)
{
	if (currLine.empty() || nextLine.empty())
	{
		return false;
	}
	map<double, pair<int, int>> DistToIndex;
	for (int currPntInd = 0; currPntInd < currLine.size(); currPntInd++)
	{
		pair<int, int> indexPair;
		vector<double> distance(nextLine.size());
		for (int nextPntInd = 0; nextPntInd < nextLine.size(); nextPntInd++)
		{			
			distance[nextPntInd] = _secRptDataObj.getLength(currLine[currPntInd], nextLine[nextPntInd]);
		}
		uint32 nextMinPos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin());
		indexPair.first = currPntInd;
		indexPair.second = nextMinPos;
		DistToIndex.insert(make_pair<double, pair<int, int>>(distance[nextMinPos], indexPair));
	}
	minCurrInd = DistToIndex.begin()->second.first;
	minNextInd =  DistToIndex.begin()->second.second;

	return true;
}

bool CRoadVecGen3::fitTwoSerialLinesToOne(IN int currSegId, IN vector<point3D_t> &currLine, 
	IN int nextSegId, IN vector<point3D_t> &nextLine, IN int degree, OUT vector<point3D_t> &wholeLine)
{
	wholeLine.clear();
	if (currLine.empty() || nextLine.empty())
	{
		return false;
	}

	wholeLine = currLine;
	wholeLine.insert(wholeLine.end(), nextLine.begin(), nextLine.end());

	vector<point3D_t> lineTemp;
	vector<point3D_t> Rotated, leftRotatedValid;
	double x0(0.0), x1(0.0), y0(0.0), y1(0.0), xlimit0(0.0), xlimit1(0.0);
	x0 = wholeLine.front().lon;
	y0 = wholeLine.front().lat;
	x1 = wholeLine.back().lon;
	y1 = wholeLine.back().lat;

	// calculate current section rotation angle, rotate the whole line first
	double theta = atan2((y0 - y1), (x0 - x1));
	lineRotation(wholeLine, -theta, Rotated);

	// calculate re-sample points
	int numOfSplPnts = 0, pOrder = 1;
	xlimit0 = Rotated.front().lon;
	xlimit1 = Rotated.back().lon;
	numOfSplPnts = (int)((xlimit1 - xlimit0) / SAMPLE_SPACE);
	if (xlimit0 >= xlimit1)
	{
		pOrder = -1;
		numOfSplPnts = (int)((xlimit0 - xlimit1) / SAMPLE_SPACE) + 1;
	}
	point3D_t pointSpl = { 0 };
	for (int i = 0; i < numOfSplPnts; i++)
	{
		pointSpl.lon = xlimit0 + i * pOrder * SAMPLE_SPACE;

		lineTemp.push_back(pointSpl);
	}

	// get paint information for re-sample line
	getLinePaintInfo(Rotated, lineTemp);
	polynomialFitting(Rotated,  lineTemp, degree);
	//interpolationSample(Rotated,  lineTemp);
	wholeLine.clear();
	lineRotation(lineTemp, theta, wholeLine);

	return true;
}

bool CRoadVecGen3::cutFittedCurveToTwoLinesByBody(IN vector<point3D_t> &wholeLine, IN int currSegId, 
	IN int currConType, OUT vector<point3D_t> &currLine, OUT vector<point3D_t> &nextLine)
{
	if (2 > wholeLine.size())
	{
		return false;
	}

	point3D_t point;
	roadSegConfig_gp->getBodyPoint(currSegId, currConType, point);
	vector<double> distance(wholeLine.size());
	for (int i = 0;i < distance.size();i++)
	{
		distance[i] = _secRptDataObj.getLength(point, wholeLine[i]);
	}
	uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

	vector<point3D_t>::iterator midItor = wholeLine.begin();
	advance(midItor, min_pos);
	//advance(midItor, currLine.size() - 1);
	currLine.clear();
	nextLine.clear();
	currLine.insert(currLine.end(), wholeLine.begin(), midItor + 1);
	nextLine.insert(nextLine.end(), midItor, wholeLine.end());

	if (currLine.empty() || nextLine.empty())
	{
		currLine.clear();
		nextLine.clear();
		return false;
	}

	return true;
}

void CRoadVecGen3::preprocessVirtualConnection(IN foregroundSectionData *currData, INOUT vector<point3D_t> **ppCurrLines)
{
	if(NULL == currData || currData->fgSectionData.empty() || NULL == ppCurrLines)
	{
		return;
	}

	int currlines = 0;
	currlines = currData->fgSectionData.size();
	list<vector<lineConn>> virtualCon;
	if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(currData->sectionId, virtualCon, VIRTUAL_LINE_CONNECTION))
	{
		return;						
	}		

	if(virtualCon.empty() || currlines != virtualCon.size())
	{
		return;
	}

	int LineIndex = 0;
	list<vector<lineConn>> :: iterator Itor = virtualCon.begin();
	while(Itor != virtualCon.end())
	{	
		if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
		{
			Itor++;
			LineIndex++;
			continue;
		}

		vector<point3D_t> *pNextLine = NULL;
		vector<point3D_t> currLineTmp;
		if(false == GetLineFromOutputList((*Itor).front().connSegID, (*Itor).front().connLineID, &pNextLine) ||
            NULL == pNextLine || pNextLine->empty())
		{
			Itor++;
			LineIndex++;
			continue;
		}

		currLineTmp = (*ppCurrLines[LineIndex]);
		point3D_t currFixPoint = currLineTmp.back();
		point3D_t nextFixPoint;
		if(STARTPOINT == (*Itor).front().connType)
		{
			nextFixPoint = (*pNextLine).front();
		}
		else
		{
			nextFixPoint = (*pNextLine).back();
		}

		double dx = 0.0, dy = 0.0;
		dx = (nextFixPoint.lon - currFixPoint.lon);
		dy = (nextFixPoint.lat - currFixPoint.lat);
		int numOfPnts = currLineTmp.size();
		if(0 < (numOfPnts - 1))
		{
			double ddx = dx / (numOfPnts - 1);
			double ddy = dy / (numOfPnts - 1);      

			for (int pi = 0; pi < numOfPnts - 1; pi++)
			{
				currLineTmp.at(pi).lon = (numOfPnts - 1 - pi)* ddx + currFixPoint.lon;
				currLineTmp.at(pi).lat = (numOfPnts - 1 - pi)* ddy + currFixPoint.lat;					
			}

			(*ppCurrLines[LineIndex]).clear();
			*ppCurrLines[LineIndex] = currLineTmp;
		}
		Itor++;
		LineIndex++;
	}

	return;
}

#if 0
//T cross merging version 0.1
void CRoadVecGen3::crossTHandle(IN foregroundSectionData *currData)
{
	if(NULL == currData || currData->fgSectionData.empty())
	{
		return;
	}

	// get matched line info of all lines in current section
	vector<uint32> prevId, nextId;
	if(false == roadSegConfig_gp->getPrevSegId(currData->sectionId, prevId) || false == roadSegConfig_gp->getNextSegId(currData->sectionId, nextId))
	{
		return;
	}
	
	if ((1 >= prevId.size() && 1 >= nextId.size()) || (1 < prevId.size() && 1 < nextId.size()))
	{
		return;
	}

	int currlines = 0;
	currlines = currData->fgSectionData.size();
	vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
	if(NULL == ppCurrLines)
	{
		printf("ERROR:NULL pointer.\n");
		return;
	}

	list<vector<point3D_t>>::iterator lineItor = currData->fgSectionData.begin();
	for (int i = 0; i < currlines; i++)
	{
		ppCurrLines[i] = &(*lineItor);
		lineItor++;
	}

	preprocessVirtualConnection(currData, ppCurrLines);

	do
	{
		list<vector<lineConn>> MatchedLines;
		int degree = 10;
		if (1 < nextId.size())
		{
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfEndPoint(currData->sectionId, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}		
		
			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				printf("section ID:%d matchedLines size is wrong.\n", currData->sectionId);
				break;
			}	

			// get end points of current lines, and start points of next lines
			int LineIndex = 0;
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if(ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}
				else
				{
					point3D_t point;
					roadSegConfig_gp->getBodyPoint(currData->sectionId, ENDPOINT, point);
					vector<double> distance((*ppCurrLines[LineIndex]).size());
					for (int i = 0;i < distance.size();i++)
					{
						distance[i] = _secRptDataObj.getLength(point, (*ppCurrLines[LineIndex])[i]);
					}
					uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

					vector<point3D_t>::iterator midItor = (*ppCurrLines[LineIndex]).begin();
					advance(midItor, min_pos);	
					midItor++;
					(*ppCurrLines[LineIndex]).erase(midItor, (*ppCurrLines[LineIndex]).end());				
				}

				if((*Itor).empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				foregroundSectionData *SegData = NULL;
				if(false == getSectionFromOutputList((*Itor).front().connSegID, &SegData)
					|| NULL == SegData || SegData->fgSectionData.empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				if((STARTPOINT == (*Itor).front().connType && HANDLED == SegData->dealStart)
					|| (ENDPOINT == (*Itor).front().connType && HANDLED == SegData->dealEnd))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				vector<point3D_t> nextLineTmp, currLineTmp, wholeLine;
				if(false == GetLineFromInputSegment(SegData, (*Itor).front().connLineID, &pNextLine))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				currLineTmp = (*ppCurrLines[LineIndex]);
				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					vector<point3D_t>::reverse_iterator lineTmpItor = (*pNextLine).rbegin();
					while (lineTmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*lineTmpItor);
						lineTmpItor++;
					}
				}

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(minDistItor + 1, currLineTmp.end());

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				double curTheta = _secRotAngle[currData->sectionId - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(STARTPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (acuteAngle > 45.0 && acuteAngle < 135.0)
				{
					double dx = (currLineTmp.back().lon - nextLineTmp.front().lon) / (double)2;
					double dy = (currLineTmp.back().lat - nextLineTmp.front().lat) / (double)2;
					for(int i = 0; i < currLineTmp.size(); i++)
					{
						currLineTmp[i].lon -= dx;
						currLineTmp[i].lat -= dy;
					}

					for(int i = 0; i < nextLineTmp.size(); i++)
					{
						nextLineTmp[i].lon += dx;
						nextLineTmp[i].lat += dy;
					}
				}
				else
				{
					vector<point3D_t> currRotated, nextRotated;
					
#if 0
					int currHalfPnts = currLineTmp.size() / 2;
					vector<point3D_t> currRotated, nextRotated;
					double currAverage = 0.0, nextAverage = 0.0;
					int currHalfPnts = currLineTmp.size() / 2;

					lineRotation(nextLineTmp, -nextTheta, nextRotated);
					int nextrHalfPnts = nextRotated.size() / 2;
					for(int i = nextrHalfPnts; i < nextRotated.size(); i++)
					{
						nextAverage += nextRotated[i].lat;
					}
					nextAverage = nextAverage / (double)(nextrHalfPnts);
					double latFix = nextRotated[nextrHalfPnts].lat;
					double latDValue = nextAverage - latFix;
					double latDDValue = latDValue / nextrHalfPnts;
					for (int pi = 0; pi < nextrHalfPnts; pi++)
					{
						nextRotated[pi].lat = (latDValue - pi * latDDValue) + latFix;					
					}

					nextLineTmp.clear();
					lineRotation(nextRotated, nextTheta, nextLineTmp);
		
					double dx = 0.0, dy = 0.0;
					point3D_t fixPoint = currLineTmp[currHalfPnts - 1];
					dx = nextLineTmp.front().lon - fixPoint.lon;
					dy = nextLineTmp.front().lat - fixPoint.lat;
					int numOfHalfPnts = currLineTmp.size() / 2;
					int remainder = currLineTmp.size() & (0x1);
					double ddx = dx / numOfHalfPnts;
					double ddy = dy / numOfHalfPnts;      

					for (int pi = 0; pi < numOfHalfPnts; pi++)
					{
						currLineTmp.at(pi + numOfHalfPnts + remainder).lon = (pi + 1)* ddx + fixPoint.lon;
						currLineTmp.at(pi + numOfHalfPnts + remainder).lat = (pi + 1)* ddy + fixPoint.lat;					
					}
#endif	
					double dx = 0.0, dy = 0.0;
					point3D_t fixPoint = currLineTmp[0];
					dx = nextLineTmp.front().lon - fixPoint.lon;
					dy = nextLineTmp.front().lat - fixPoint.lat;
					int numOfPnts = currLineTmp.size();
					int remainder = currLineTmp.size() & (0x1);
					double ddx = dx / (numOfPnts - 1);
					double ddy = dy / (numOfPnts - 1);      

					for (int pi = 1; pi < numOfPnts; pi++)
					{
						currLineTmp.at(pi).lon = (pi)* ddx + fixPoint.lon;
						currLineTmp.at(pi).lat = (pi)* ddy + fixPoint.lat;					
					}
				}

				(*(ppCurrLines[LineIndex])).clear();		
				(*pNextLine).clear();
				(*(ppCurrLines[LineIndex])) = currLineTmp;

				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					vector<point3D_t>::reverse_iterator tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}

				Itor++;	
				LineIndex++;
			}

			currData->dealEnd = HANDLED;
			break;
		}
		
		if (1 < prevId.size())
		{
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(currData->sectionId, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}
		
			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				printf("section ID:%d matchedLines size is wrong.\n", currData->sectionId);
				break;
			}	

			// get end points of current lines, and start points of next lines
			int LineIndex = 0;
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if(ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}
				else
				{
					point3D_t point;
					roadSegConfig_gp->getBodyPoint(currData->sectionId, STARTPOINT, point);
					vector<double> distance((*ppCurrLines[LineIndex]).size());
					for (int i = 0;i < distance.size();i++)
					{
						distance[i] = _secRptDataObj.getLength(point, (*ppCurrLines[LineIndex])[i]);
					}
					uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

					vector<point3D_t>::iterator midItor = (*ppCurrLines[LineIndex]).begin();
					advance(midItor, min_pos);	
					(*ppCurrLines[LineIndex]).erase((*ppCurrLines[LineIndex]).begin(), midItor);
				}

				if((*Itor).empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				foregroundSectionData *SegData = NULL;
				if(false == getSectionFromOutputList((*Itor).front().connSegID, &SegData)
					|| NULL == SegData || SegData->fgSectionData.empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				if((STARTPOINT == (*Itor).front().connType && HANDLED == SegData->dealStart)
					|| (ENDPOINT == (*Itor).front().connType && HANDLED == SegData->dealEnd))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				vector<point3D_t> nextLineTmp, currLineTmp, wholeLine;
				if(false == GetLineFromInputSegment(SegData, (*Itor).front().connLineID, &pNextLine))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t>::reverse_iterator tmpItor = (*(ppCurrLines[LineIndex])).rbegin();
				while (tmpItor != (*(ppCurrLines[LineIndex])).rend())
				{
					currLineTmp.push_back(*tmpItor);
					tmpItor++;
				}

				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					tmpItor = (*pNextLine).rbegin();
					while (tmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*tmpItor);
						tmpItor++;
					}
				}

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(minDistItor + 1, currLineTmp.end());

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				double curTheta = _secRotAngle[currData->sectionId - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(ENDPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (acuteAngle > 45.0 && acuteAngle < 135.0)
				{
					double dx = (currLineTmp.back().lon - nextLineTmp.front().lon) / (double)2;
					double dy = (currLineTmp.back().lat - nextLineTmp.front().lat) / (double)2;
					for(int i = 0; i < currLineTmp.size(); i++)
					{
						currLineTmp[i].lon -= dx;
						currLineTmp[i].lat -= dy;
					}

					for(int i = 0; i < nextLineTmp.size(); i++)
					{
						nextLineTmp[i].lon += dx;
						nextLineTmp[i].lat += dy;
					}
				}
				else
				{
					vector<point3D_t> currRotated, nextRotated;
					
#if 0
					int currHalfPnts = currLineTmp.size() / 2;
					double currAverage = 0.0, nextAverage = 0.0;
					
					lineRotation(nextLineTmp, -nextTheta, nextRotated);
					int nextrHalfPnts = nextRotated.size() / 2;
					for(int i = nextrHalfPnts; i < nextRotated.size(); i++)
					{
						nextAverage += nextRotated[i].lat;
					}
					nextAverage = nextAverage / (double)(nextrHalfPnts);
					double latFix = nextRotated[nextrHalfPnts].lat;
					double latDValue = nextAverage - latFix;
					double latDDValue = latDValue / nextrHalfPnts;
					for (int pi = 0; pi < nextrHalfPnts; pi++)
					{
						nextRotated[pi].lat = (latDValue - pi * latDDValue) + latFix;					
					}

					nextLineTmp.clear();
					lineRotation(nextRotated, nextTheta, nextLineTmp);
	
					double dx = 0.0, dy = 0.0;
					point3D_t fixPoint = currLineTmp[currHalfPnts - 1];
					dx = nextLineTmp.front().lon - fixPoint.lon;
					dy = nextLineTmp.front().lat - fixPoint.lat;
					int numOfHalfPnts = currLineTmp.size() / 2;
					int remainder = currLineTmp.size() & (0x1);
					double ddx = dx / numOfHalfPnts;
					double ddy = dy / numOfHalfPnts;      

					for (int pi = 0; pi < numOfHalfPnts; pi++)
					{
						currLineTmp.at(pi + numOfHalfPnts + remainder).lon = (pi + 1)* ddx + fixPoint.lon;
						currLineTmp.at(pi + numOfHalfPnts + remainder).lat = (pi + 1)* ddy + fixPoint.lat;					
					}
	
					double dx = 0.0, dy = 0.0;
					point3D_t fixPoint = currLineTmp[0];
					dx = nextLineTmp.front().lon - fixPoint.lon;
					dy = nextLineTmp.front().lat - fixPoint.lat;
					int numOfPnts = currLineTmp.size();
					int remainder = currLineTmp.size() & (0x1);
					double ddx = dx / (numOfPnts - 1);
					double ddy = dy / (numOfPnts - 1);      

					for (int pi = 1; pi < numOfPnts; pi++)
					{
						currLineTmp.at(pi).lon = (pi)* ddx + fixPoint.lon;
						currLineTmp.at(pi).lat = (pi)* ddy + fixPoint.lat;					
					}
#endif
					double dx = 0.0, dy = 0.0;
					point3D_t fixPoint = currLineTmp[0];
					dx = nextLineTmp.front().lon - fixPoint.lon;
					dy = nextLineTmp.front().lat - fixPoint.lat;
					int numOfPnts = currLineTmp.size();
					int remainder = currLineTmp.size() & (0x1);
					double ddx = dx / (numOfPnts - 1);
					double ddy = dy / (numOfPnts - 1);      

					for (int pi = 1; pi < numOfPnts; pi++)
					{
						currLineTmp.at(pi).lon = (pi)* ddx + fixPoint.lon;
						currLineTmp.at(pi).lat = (pi)* ddy + fixPoint.lat;					
					}
				}

				(*(ppCurrLines[LineIndex])).clear();		
				(*pNextLine).clear();
				tmpItor = currLineTmp.rbegin();
				while (tmpItor != currLineTmp.rend())
				{
					(*(ppCurrLines[LineIndex])).push_back(*tmpItor);
					tmpItor++;
				}

				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}

				Itor++;	
				LineIndex++;
			}

			currData->dealStart = HANDLED;
			break;
		}	
	}while(0);

	freePointerArray(ppCurrLines, currlines);
	return;
}
#endif

#if 1
//T cross merging version 0.0
void CRoadVecGen3::crossTHandle(IN foregroundSectionData *currData)
{
	if(NULL == currData || currData->fgSectionData.empty())
	{
		return;
	}

	// get matched line info of all lines in current section
	vector<uint32> prevId, nextId;
	if(false == roadSegConfig_gp->getPrevSegId(currData->sectionId, prevId) || false == roadSegConfig_gp->getNextSegId(currData->sectionId, nextId))
	{
		return;
	}
	
	if ((1 >= prevId.size() && 1 >= nextId.size()) || (1 < prevId.size() && 1 < nextId.size()))
	{
		return;
	}

	int currlines = 0;
	currlines = currData->fgSectionData.size();
	vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
	if(NULL == ppCurrLines)
	{
		printf("ERROR:NULL pointer.\n");
		return;
	}

	list<vector<point3D_t>>::iterator lineItor = currData->fgSectionData.begin();
	for (int i = 0; i < currlines; i++)
	{
		ppCurrLines[i] = &(*lineItor);
		lineItor++;
	}

	do
	{
		list<vector<lineConn>> MatchedLines;
		if (1 < nextId.size())
		{
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfEndPoint(currData->sectionId, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}		
		
			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				printf("section ID:%d matchedLines size is wrong.\n", currData->sectionId);
				break;
			}	

			// get end points of current lines, and start points of next lines
			int LineIndex = 0;
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if(NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}

				if((*Itor).empty() && !ppCurrLines[LineIndex]->empty())
				{
					point3D_t point;
					roadSegConfig_gp->getBodyPoint(currData->sectionId, ENDPOINT, point);
					vector<double> distance((*ppCurrLines[LineIndex]).size());
					for (int i = 0;i < distance.size();i++)
					{
						distance[i] = _secRptDataObj.getLength(point, (*ppCurrLines[LineIndex])[i]);
					}
					uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

					vector<point3D_t>::iterator midItor = (*ppCurrLines[LineIndex]).begin();
					advance(midItor, min_pos);	
					midItor++;
					(*ppCurrLines[LineIndex]).erase(midItor, (*ppCurrLines[LineIndex]).end());
					Itor++;
					LineIndex++;
					continue;
				}

				foregroundSectionData *SegData = NULL;
				if(false == getSectionFromOutputList((*Itor).front().connSegID, &SegData)
					|| NULL == SegData || SegData->fgSectionData.empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				if((STARTPOINT == (*Itor).front().connType && HANDLED == SegData->dealStart)
					|| (ENDPOINT == (*Itor).front().connType && HANDLED == SegData->dealEnd))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				vector<point3D_t> nextLineTmp, currLineTmp, wholeLine;
				if(false == GetLineFromInputSegment(SegData, (*Itor).front().connLineID, &pNextLine) ||
                    NULL == pNextLine || pNextLine->empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				currLineTmp = (*ppCurrLines[LineIndex]);
				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					vector<point3D_t>::reverse_iterator lineTmpItor = (*pNextLine).rbegin();
					while (lineTmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*lineTmpItor);
						lineTmpItor++;
					}
				}

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(minDistItor + 1, currLineTmp.end());

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				double curTheta = _secRotAngle[currData->sectionId - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(STARTPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (acuteAngle > 45.0 && acuteAngle < 135.0)
				{
					double dx = (currLineTmp.back().lon - nextLineTmp.front().lon) / (double)2;
					double dy = (currLineTmp.back().lat - nextLineTmp.front().lat) / (double)2;
					for(int i = 0; i < currLineTmp.size(); i++)
					{
						currLineTmp[i].lon -= dx;
						currLineTmp[i].lat -= dy;
					}

					for(int i = 0; i < nextLineTmp.size(); i++)
					{
						nextLineTmp[i].lon += dx;
						nextLineTmp[i].lat += dy;
					}
				}
				else
				{
					vector<point3D_t> nextRotated;
					double nextAverage = 0.0;					
					lineRotation(nextLineTmp, -nextTheta, nextRotated);
					int nextStartPnts =  1 * nextRotated.size() / 5;
					int nextEndPnts = nextRotated.size();
					for(int i = nextStartPnts; i < nextEndPnts; i++)
					{
						nextAverage += nextRotated[i].lat;
					}
					nextAverage = nextAverage / (double)(nextEndPnts - nextStartPnts);
					double latFix = nextRotated[nextStartPnts].lat;
					double latDValue = nextAverage - latFix;
					double latDDValue = latDValue / nextStartPnts;
					for (int pi = 0; pi < nextStartPnts; pi++)
					{
						nextRotated[pi].lat = (latDValue - pi * latDDValue) + latFix;					
					}

					nextLineTmp.clear();
					lineRotation(nextRotated, nextTheta, nextLineTmp);
			
					double dx = 0.0, dy = 0.0;
					int currHalfPnts = currLineTmp.size() / 2;
					point3D_t fixPoint = currLineTmp[currHalfPnts - 1];
					dx = nextLineTmp.front().lon - fixPoint.lon;
					dy = nextLineTmp.front().lat - fixPoint.lat;
					int numOfHalfPnts = currLineTmp.size() / 2;
					int remainder = currLineTmp.size() & (0x1);
					double ddx = dx / numOfHalfPnts;
					double ddy = dy / numOfHalfPnts;      

					for (int pi = 0; pi < numOfHalfPnts; pi++)
					{
						currLineTmp.at(pi + numOfHalfPnts + remainder).lon = (pi + 1)* ddx + fixPoint.lon;
						currLineTmp.at(pi + numOfHalfPnts + remainder).lat = (pi + 1)* ddy + fixPoint.lat;					
					}
				}

				(*(ppCurrLines[LineIndex])).clear();		
				(*pNextLine).clear();
				(*(ppCurrLines[LineIndex])) = currLineTmp;

				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					vector<point3D_t>::reverse_iterator tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}

				Itor++;	
				LineIndex++;
			}

			currData->dealEnd = HANDLED;
			break;
		}
		
		if (1 < prevId.size())
		{
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(currData->sectionId, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}
		
			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				printf("section ID:%d matchedLines size is wrong.\n", currData->sectionId);
				break;
			}	

			// get end points of current lines, and start points of next lines
			int LineIndex = 0;
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if(NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}

				if((*Itor).empty() && !ppCurrLines[LineIndex]->empty())
				{
					point3D_t point;
					roadSegConfig_gp->getBodyPoint(currData->sectionId, STARTPOINT, point);
					vector<double> distance((*ppCurrLines[LineIndex]).size());
					for (int i = 0;i < distance.size();i++)
					{
						distance[i] = _secRptDataObj.getLength(point, (*ppCurrLines[LineIndex])[i]);
					}
					uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

					vector<point3D_t>::iterator midItor = (*ppCurrLines[LineIndex]).begin();
					advance(midItor, min_pos);	
					(*ppCurrLines[LineIndex]).erase((*ppCurrLines[LineIndex]).begin(), midItor);
					Itor++;
					LineIndex++;
					continue;
				}

				foregroundSectionData *SegData = NULL;
				if(false == getSectionFromOutputList((*Itor).front().connSegID, &SegData)
					|| NULL == SegData || SegData->fgSectionData.empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				if((STARTPOINT == (*Itor).front().connType && HANDLED == SegData->dealStart)
					|| (ENDPOINT == (*Itor).front().connType && HANDLED == SegData->dealEnd))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				vector<point3D_t> nextLineTmp, currLineTmp;
				if(false == GetLineFromInputSegment(SegData, (*Itor).front().connLineID, &pNextLine) ||
                    NULL == pNextLine || pNextLine->empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t>::reverse_iterator tmpItor = (*(ppCurrLines[LineIndex])).rbegin();
				while (tmpItor != (*(ppCurrLines[LineIndex])).rend())
				{
					currLineTmp.push_back(*tmpItor);
					tmpItor++;
				}

				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					tmpItor = (*pNextLine).rbegin();
					while (tmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*tmpItor);
						tmpItor++;
					}
				}

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(minDistItor + 1, currLineTmp.end());

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				double curTheta = _secRotAngle[currData->sectionId - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(ENDPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (acuteAngle > 45.0 && acuteAngle < 135.0)
				{
					double dx = (currLineTmp.back().lon - nextLineTmp.front().lon) / (double)2;
					double dy = (currLineTmp.back().lat - nextLineTmp.front().lat) / (double)2;
					for(int i = 0; i < currLineTmp.size(); i++)
					{
						currLineTmp[i].lon -= dx;
						currLineTmp[i].lat -= dy;
					}

					for(int i = 0; i < nextLineTmp.size(); i++)
					{
						nextLineTmp[i].lon += dx;
						nextLineTmp[i].lat += dy;
					}
				}
				else
				{
					vector<point3D_t> nextRotated;
					double nextAverage = 0.0;					
					lineRotation(nextLineTmp, -nextTheta, nextRotated);
					int nextStartPnts =  1 * nextRotated.size() / 3;
					int nextEndPnts =  2 * nextRotated.size() / 3;
					for(int i = nextStartPnts; i < nextEndPnts; i++)
					{
						nextAverage += nextRotated[i].lat;
					}
					nextAverage = nextAverage / (double)(nextEndPnts - nextStartPnts);
					double latFix = nextRotated[nextStartPnts].lat;
					double latDValue = nextAverage - latFix;
					double latDDValue = latDValue / nextStartPnts;
					for (int pi = 0; pi < nextStartPnts; pi++)
					{
						nextRotated[pi].lat = (latDValue - pi * latDDValue) + latFix;					
					}

					nextLineTmp.clear();
					lineRotation(nextRotated, nextTheta, nextLineTmp);
			
					double dx = 0.0, dy = 0.0;
					int currHalfPnts = currLineTmp.size() / 2;
					point3D_t fixPoint = currLineTmp[currHalfPnts - 1];
					dx = nextLineTmp.front().lon - fixPoint.lon;
					dy = nextLineTmp.front().lat - fixPoint.lat;
					int numOfHalfPnts = currLineTmp.size() / 2;
					int remainder = currLineTmp.size() & (0x1);
					double ddx = dx / numOfHalfPnts;
					double ddy = dy / numOfHalfPnts;      

					for (int pi = 0; pi < numOfHalfPnts; pi++)
					{
						currLineTmp.at(pi + numOfHalfPnts + remainder).lon = (pi + 1)* ddx + fixPoint.lon;
						currLineTmp.at(pi + numOfHalfPnts + remainder).lat = (pi + 1)* ddy + fixPoint.lat;					
					}
				}

				(*(ppCurrLines[LineIndex])).clear();		
				(*pNextLine).clear();
				tmpItor = currLineTmp.rbegin();
				while (tmpItor != currLineTmp.rend())
				{
					(*(ppCurrLines[LineIndex])).push_back(*tmpItor);
					tmpItor++;
				}

				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}

				Itor++;	
				LineIndex++;
			}

			currData->dealStart = HANDLED;
			break;
		}	
	}while(0);

	freePointerArray(ppCurrLines, currlines);
	return;
}
#endif

#if 0
//T cross merging version 0
void CRoadVecGen3::crossTHandle(IN foregroundSectionData *currData)
{
	if(NULL == currData || currData->fgSectionData.empty())
	{
		return;
	}

	// get matched line info of all lines in current section
	vector<uint32> prevId, nextId;
	if(false == roadSegConfig_gp->getPrevSegId(currData->sectionId, prevId) || false == roadSegConfig_gp->getNextSegId(currData->sectionId, nextId))
	{
		return;
	}
	
	if ((1 >= prevId.size() && 1 >= nextId.size()) || (1 < prevId.size() && 1 < nextId.size()))
	{
		return;
	}

	int currlines = 0;
	currlines = currData->fgSectionData.size();
	vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
	if(NULL == ppCurrLines)
	{
		printf("ERROR:NULL pointer.\n");
		return;
	}

	list<vector<point3D_t>>::iterator lineItor = currData->fgSectionData.begin();
	for (int i = 0; i < currlines; i++)
	{
		ppCurrLines[i] = &(*lineItor);
		lineItor++;
	}

	do
	{
		list<vector<lineConn>> MatchedLines;
		int degree = 10;
		if (1 < nextId.size())
		{
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfEndPoint(currData->sectionId, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}		
		
			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				printf("section ID:%d matchedLines size is wrong.\n", currData->sectionId);
				break;
			}	

			// get end points of current lines, and start points of next lines
			int LineIndex = 0;
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if(ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}

				if((*Itor).empty() && !ppCurrLines[LineIndex]->empty())
				{
					point3D_t point;
					roadSegConfig_gp->getBodyPoint(currData->sectionId, ENDPOINT, point);
					vector<double> distance((*ppCurrLines[LineIndex]).size());
					for (int i = 0;i < distance.size();i++)
					{
						distance[i] = _secRptDataObj.getLength(point, (*ppCurrLines[LineIndex])[i]);
					}
					uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

					vector<point3D_t>::iterator midItor = (*ppCurrLines[LineIndex]).begin();
					advance(midItor, min_pos);	
					midItor++;
					(*ppCurrLines[LineIndex]).erase(midItor, (*ppCurrLines[LineIndex]).end());
					Itor++;
					LineIndex++;
					continue;
				}

				foregroundSectionData *SegData = NULL;
				if(false == getSectionFromOutputList((*Itor).front().connSegID, &SegData)
					|| NULL == SegData || SegData->fgSectionData.empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				if((STARTPOINT == (*Itor).front().connType && HANDLED == SegData->dealStart)
					|| (ENDPOINT == (*Itor).front().connType && HANDLED == SegData->dealEnd))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				vector<point3D_t> nextLineTmp, currLineTmp, wholeLine;
				if(false == GetLineFromInputSegment(SegData, (*Itor).front().connLineID, &pNextLine))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				currLineTmp = (*ppCurrLines[LineIndex]);
				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					vector<point3D_t>::reverse_iterator lineTmpItor = (*pNextLine).rbegin();
					while (lineTmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*lineTmpItor);
						lineTmpItor++;
					}
				}

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(minDistItor + 1, currLineTmp.end());

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				double curTheta = _secRotAngle[currData->sectionId - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(STARTPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (acuteAngle > 45.0 && acuteAngle < 135.0)
				{
					degree = 13;			
				}
				else
				{
					degree = 3;
				}
				if(false == fitTwoSerialLinesToOne(currData->sectionId, currLineTmp, (*Itor).front().connSegID, nextLineTmp, degree, wholeLine))
				{
					Itor++;
					LineIndex++;
					continue;				
				}

				(*(ppCurrLines[LineIndex])).clear();		
				(*pNextLine).clear();
				currLineTmp.clear();
				nextLineTmp.clear();
				if(false == cutFittedCurveToTwoLinesByBody(wholeLine, currData->sectionId, ENDPOINT, currLineTmp, nextLineTmp))
				{
					Itor++;
					LineIndex++;
					continue;				
				}

				double eraseRatio = 0.10;
				int eraseNum = 0;
				if (CURRENT_RPT == currData->isNewRpt)
				{
					eraseNum = eraseRatio * ((double)currLineTmp.size());
					while (eraseNum > 0)
					{
						vector<point3D_t>::iterator erasedItor = currLineTmp.begin();
						if (currLineTmp.end() != erasedItor)
						{
							currLineTmp.erase(erasedItor);
						}
						eraseNum--;
					}
				}		
				(*(ppCurrLines[LineIndex])) = currLineTmp;

				if (CURRENT_RPT == SegData->isNewRpt)
				{		
					eraseNum = eraseRatio * ((double)nextLineTmp.size());
					while (eraseNum > 0)
					{
						vector<point3D_t>::iterator erasedItor = nextLineTmp.end();
						if (nextLineTmp.begin() != erasedItor)
						{
							erasedItor--;
							nextLineTmp.erase(erasedItor);
						}
						eraseNum--;
					}			
				}		
				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					vector<point3D_t>::reverse_iterator tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}

				Itor++;	
				LineIndex++;
			}

			currData->dealEnd = HANDLED;
			break;
		}
		
		if (1 < prevId.size())
		{
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(currData->sectionId, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}
		
			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				printf("section ID:%d matchedLines size is wrong.\n", currData->sectionId);
				break;
			}	

			// get end points of current lines, and start points of next lines
			int LineIndex = 0;
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if(ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}

				if((*Itor).empty() && !ppCurrLines[LineIndex]->empty())
				{
					point3D_t point;
					roadSegConfig_gp->getBodyPoint(currData->sectionId, STARTPOINT, point);
					vector<double> distance((*ppCurrLines[LineIndex]).size());
					for (int i = 0;i < distance.size();i++)
					{
						distance[i] = _secRptDataObj.getLength(point, (*ppCurrLines[LineIndex])[i]);
					}
					uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );

					vector<point3D_t>::iterator midItor = (*ppCurrLines[LineIndex]).begin();
					advance(midItor, min_pos);	
					(*ppCurrLines[LineIndex]).erase((*ppCurrLines[LineIndex]).begin(), midItor);
					Itor++;
					LineIndex++;
					continue;
				}

				foregroundSectionData *SegData = NULL;
				if(false == getSectionFromOutputList((*Itor).front().connSegID, &SegData)
					|| NULL == SegData || SegData->fgSectionData.empty())
				{
					Itor++;
					LineIndex++;
					continue;
				}

				if((STARTPOINT == (*Itor).front().connType && HANDLED == SegData->dealStart)
					|| (ENDPOINT == (*Itor).front().connType && HANDLED == SegData->dealEnd))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				vector<point3D_t> nextLineTmp, currLineTmp, wholeLine;
				if(false == GetLineFromInputSegment(SegData, (*Itor).front().connLineID, &pNextLine))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t>::reverse_iterator tmpItor = (*(ppCurrLines[LineIndex])).rbegin();
				while (tmpItor != (*(ppCurrLines[LineIndex])).rend())
				{
					currLineTmp.push_back(*tmpItor);
					tmpItor++;
				}

				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					tmpItor = (*pNextLine).rbegin();
					while (tmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*tmpItor);
						tmpItor++;
					}
				}

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(minDistItor + 1, currLineTmp.end());

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				double curTheta = _secRotAngle[currData->sectionId - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(ENDPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (acuteAngle > 45.0 && acuteAngle < 135.0)
				{
					degree = 13;
				}
				else
				{
					degree = 3;
				}

				if(false == fitTwoSerialLinesToOne(currData->sectionId, currLineTmp, (*Itor).front().connSegID, nextLineTmp, degree, wholeLine))
				{
					Itor++;
					LineIndex++;
					continue;				
				}

				(*(ppCurrLines[LineIndex])).clear();		
				(*pNextLine).clear();
				currLineTmp.clear();
				nextLineTmp.clear();
				if(false == cutFittedCurveToTwoLinesByBody(wholeLine, currData->sectionId, STARTPOINT, currLineTmp, nextLineTmp))
				{
					Itor++;
					LineIndex++;
					continue;				
				}

				double eraseRatio = 0.10;
				int eraseNum = 0;
				if (CURRENT_RPT == currData->isNewRpt)
				{
					eraseNum = eraseRatio * ((double)currLineTmp.size());
					while (eraseNum > 0)
					{
						vector<point3D_t>::iterator erasedItor = currLineTmp.begin();
						if (currLineTmp.end() != erasedItor)
						{
							currLineTmp.erase(erasedItor);
						}
						eraseNum--;
					}
				}
				tmpItor = currLineTmp.rbegin();
				while (tmpItor != currLineTmp.rend())
				{
					(*(ppCurrLines[LineIndex])).push_back(*tmpItor);
					tmpItor++;
				}
	
				if (CURRENT_RPT == SegData->isNewRpt)
				{
					eraseNum = eraseRatio * ((double)nextLineTmp.size());
					while (eraseNum > 0)
					{
						vector<point3D_t>::iterator erasedItor = nextLineTmp.end();
						if (nextLineTmp.begin() != erasedItor)
						{
							erasedItor--;
							nextLineTmp.erase(erasedItor);
						}
						eraseNum--;
					}
				}
				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}

				Itor++;	
				LineIndex++;
			}

			currData->dealStart = HANDLED;
			break;
		}	
	}while(0);

	freePointerArray(ppCurrLines, currlines);
	return;
}
#endif

void CRoadVecGen3::adjacentOneSeg(IN int flag, IN list<vector<lineConn>> &segLines)
{
	foregroundSectionData *SegData = NULL;
	if(segLines.empty())
	{
		return;
	}

	list<vector<lineConn>>::iterator connItor = segLines.begin();
	while (connItor != segLines.end())
	{
		if(!connItor->empty())
		{
			break;
		}
		connItor++;
	}

	if(connItor == segLines.end())
	{
		return;
	}

	uint32 SegID = connItor->front().connSegID;

	if(false == getSectionFromOutputList(SegID, &SegData))
	{
		printf("ERROR:Get section(ID:%d) from outputlist failed.\n", SegID);
		return;
	}

	if(NULL == SegData)
	{
		printf("ERROR:section ID:%d SegData is NULL.\n", SegID);
		return;
	}

	if(SegData->fgSectionData.empty())
	{
		return;
	}

	int currlines = SegData->fgSectionData.size();
	vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
	if(NULL == ppCurrLines)
	{
		printf("ERROR:NULL pointer.\n");
		return;
	}

	list<vector<point3D_t>>::iterator lineItor = SegData->fgSectionData.begin();
	for (int i = 0; i < currlines; i++)
	{
		ppCurrLines[i] = &(*lineItor);
		lineItor++;
	}

	int LineIndex = 0;
	int matchMinIdx = -1;
	vector<point3D_t> fixPoint, movePoint;
    vector<int> matchedInd;	
	int connType = 0;
	list<vector<lineConn>> :: iterator Itor = segLines.begin();
	while(Itor != segLines.end())
	{					
		if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
		{
			matchedInd.push_back(UNMATCHED_LINE_FLAG);
			Itor++;
			LineIndex++;
			continue;			
		}

		vector<point3D_t> *pPrevLine = NULL;
		if(false == GetLineFromOutputList((*Itor).front().SegID, (*Itor).front().LineID, &pPrevLine) ||
            NULL == pPrevLine || pPrevLine->empty())
		{
			matchedInd.push_back(UNMATCHED_LINE_FLAG);
			Itor++;
			LineIndex++;
			continue;									
		}

		connType = (*Itor).front().connType;
		if(STARTPOINT == (*Itor).front().connType)
		{
			movePoint.push_back(ppCurrLines[LineIndex]->front());
		}
		else
		{
			movePoint.push_back(ppCurrLines[LineIndex]->back());
		}

		if(POINT_TYPE_END == flag)
		{
			fixPoint.push_back((*pPrevLine).back());
		}	
		else
		{
			fixPoint.push_back((*pPrevLine).front());
		}
			
		if(-1 == matchMinIdx)
		{
			matchMinIdx = LineIndex;
		}
			
		matchedInd.push_back(MATCHED_LINE_FLAG);
		Itor++;	
		LineIndex++;
	}

    // if exist matched lines, size of fixPoint and movePoint should be the same
    if (!fixPoint.empty() && !movePoint.empty())
    {
        int index = 0;
		double dx = 0.0, dy = 0.0, ddx = 0.0, ddy = 0.0;

        for (int LineIndex = 0; LineIndex < currlines; LineIndex++)
        {
			if(LineIndex < matchMinIdx)
			{
				index = 0;
			}
				
			if ((MATCHED_LINE_FLAG == matchedInd[LineIndex])&&(LineIndex != matchMinIdx))
			{
				index++;
			}
				
			dx = fixPoint[index].lon - movePoint[index].lon;
            dy = fixPoint[index].lat - movePoint[index].lat;	
				
			// for matched lines
			if(NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
			{
				continue;	
			}
            int numOfHalfPnts = ppCurrLines[LineIndex]->size() / 2;
			int remainder = ppCurrLines[LineIndex]->size() & (0x1);
            double ddx = dx / numOfHalfPnts;
            double ddy = dy / numOfHalfPnts;      

			if(ENDPOINT == connType)
			{
				for (int pi = 0; pi < numOfHalfPnts; pi++)
				{
					ppCurrLines[LineIndex]->at(pi + numOfHalfPnts + remainder).lon += (pi + 1)* ddx;
					ppCurrLines[LineIndex]->at(pi + numOfHalfPnts + remainder).lat += (pi + 1)* ddy;					
				}
			}
			else
			{
				for (int pi = 0; pi < numOfHalfPnts; pi++)
				{
					ppCurrLines[LineIndex]->at(pi).lon += (dx - pi * ddx);
					ppCurrLines[LineIndex]->at(pi).lat += (dy - pi * ddy);					
				}				
			}
		}
    }

	freePointerArray(ppCurrLines, currlines);
}

void CRoadVecGen3::adjacentMultiplePoint(IN int flag, IN list<list<vector<lineConn>>> &lines)
{
	list<vector<lineConn>> secLines;
	list<list<vector<lineConn>>> ::iterator segItor = lines.begin();
	while(segItor != lines.end())
	{
		secLines = *segItor;
		if(!secLines.empty())
		{
			adjacentOneSeg(flag, secLines);	
		}
		
		segItor++;		
	}			
}

void CRoadVecGen3::jointProcessing(OUT list<list<vector<point3D_t>>> &fgData)
{
    // if output database is empty, clear output 
    if (_fgOutputList.empty())
    {
        fgData.clear();
        return;
    }

    // iterate each adjacent sections to join together
    list<foregroundSectionData>::iterator currSec = _fgOutputList.begin();	
	while (currSec != _fgOutputList.end())
	{	
		currSec->dealEnd = NOT_HANDLED;
		currSec->dealStart = NOT_HANDLED;
		currSec++;
	}

	currSec = _fgOutputList.begin();
	foregroundSectionData *currData = nullptr;
	while (currSec != _fgOutputList.end())
	{	
		currData = &(*currSec);

		//get segment type fail, or type is not T cross, or current section is empty, skip
		segment_type_e segType;
		if (false == roadSegConfig_gp->getSegmentType(currData->sectionId, segType) || T_ROAD_CROSS != segType || currSec->fgSectionData.empty())
		{
			currSec++;
			continue;
		}
		
		//current section is not empty
		crossTHandle(currData);
		currSec++;
	}

	currSec = _fgOutputList.begin();
	currData = nullptr;
	list<list<vector<lineConn>>> stPntLines, endPntLines;
    while (currSec != _fgOutputList.end())
    {	
		currData = &(*currSec);
		currData->isNewRpt = NOT_CURRENT_RPT;

        //current section is empty
		if (currSec->fgSectionData.empty())
		{
			currSec++;
			continue;
		}
		
		//current section is not empty
		adjacentEndOrStartPoint(currData, POINT_TYPE_END, endPntLines);
		adjacentEndOrStartPoint(currData, POINT_TYPE_START, stPntLines);
		currSec++;
	}

	adjacentMultiplePoint(POINT_TYPE_END, endPntLines);
	adjacentMultiplePoint(POINT_TYPE_START, stPntLines);
	
	// copy lines from _fgOutputList into fgData
	getFgData(fgData);
	
}
#if 0
void CRoadVecGen3::getFgData(OUT list<list<vector<point3D_t>>> &fgData)
{
    // if output database is empty, clear output data
    if (_fgOutputList.empty())
    {
        fgData.clear();
        return;
    }
    fgData.clear();

    // iterate each sections, copy lines of _fgOutputList into fgData
    list<foregroundSectionData>::iterator currSecItor = _fgOutputList.begin();
    foregroundSectionData curData;

	// background database section iterator used to get merged information
	list<backgroundSectionData>::iterator bgSecItor = _bgDatabaseList.begin();
	list<backgroundSectionData>::iterator bgSecRevItor = _bgRevDirList.begin();
	list<list<vector<point3D_t>>>::iterator laneItor, laneRevItor;

    while (currSecItor != _fgOutputList.end())
    {
		curData = *currSecItor;			
		
		// current section data is empty, push empty lines to match lane number
        if (curData.fgSectionData.empty())
        {
            list<vector<point3D_t>> fglines;
            fgData.push_back(fglines);
			currSecItor++;
			bgSecItor++;
			bgSecRevItor++;
            continue;
        }

		//current section data is not empty,push non-empty lines of current section to fgData
		int reverseLaneNum = roadSegConfig_gp->getLaneNumInSeg(curData.sectionId,true);
		int forwardLaneNum = roadSegConfig_gp->getLaneNumInSeg(curData.sectionId,false);
		bool bCommLinesMerged = roadSegConfig_gp->getBothSideMergeFlag(curData.sectionId);

		int numOfUsedLines = 0;
		list<vector<point3D_t>> fglines;
		laneItor = bgSecItor->bgSectionData.begin();
		laneRevItor = bgSecRevItor->bgSectionData.begin();
		list<vector<point3D_t>>::iterator lineItor = curData.fgSectionData.begin();	
		while(lineItor != curData.fgSectionData.end())
		{
			if (numOfUsedLines < reverseLaneNum)
			{
				if(!bgSecRevItor->bgSectionData.empty())
				{
					if (laneRevItor != bgSecRevItor->bgSectionData.end())
					{
						if (!lineItor->empty())
						{
							if(!laneRevItor->empty())
							{
								lineItor->at(0).count = laneRevItor->front().at(0).count;
							}else
							{
								lineItor->at(0).count = LANE_END_LINE_FLAG;
							}
						}

						laneRevItor++;
					}
				}
			}
			else
			{
				if ((!bCommLinesMerged) && (numOfUsedLines == reverseLaneNum) && (reverseLaneNum != 0))
				{
					if (!lineItor->empty())
					{
						lineItor->at(0).count = LANE_END_LINE_FLAG;
					}
				}
				else
				{
					if (!bgSecItor->bgSectionData.empty())
					{
						if (laneItor != bgSecItor->bgSectionData.end())
						{
							if (!lineItor->empty())
							{
								if(!laneItor->empty())
								{
									lineItor->at(0).count = laneItor->front().at(0).count;
								}else
								{
									lineItor->at(0).count = LANE_END_LINE_FLAG;
								}
							}

							laneItor++;
						}else
						{
							if (!lineItor->empty())
							{
								lineItor->at(0).count = LANE_END_LINE_FLAG;
							}
						}
					}else
					{
						if (!lineItor->empty())
							{
								lineItor->at(0).count = LANE_END_LINE_FLAG;
							}
					}
				}
			}

			fglines.push_back(*(lineItor));
		
			numOfUsedLines++;
			lineItor++;
		}

		fgData.push_back(fglines);	

		currSecItor++;	
		bgSecItor++;
		bgSecRevItor++;
	}

	vector<int> handledSeg;
	int segID = 1;
	list<list<vector<point3D_t>>>::iterator fgSegItor = fgData.begin();
	while(fgSegItor != fgData.end())
	{
		segment_type_e segType;
		if (true == roadSegConfig_gp->getSegmentType(segID, segType) && T_ROAD_CROSS == segType)
		{
			for(int i = 0; i < handledSeg.size(); i++)
			{
				if(segID == handledSeg[i])
				{
					segID++;
					fgSegItor++;
					continue;
				}
			}
			crossTCurveFitting(segID, (*fgSegItor), fgData, handledSeg);
			handledSeg.push_back(segID);
		}

		segID++;
		fgSegItor++;
	}

	handledSeg.clear();
	segID = 1;
	fgSegItor = fgData.begin();
	while(fgSegItor != fgData.end())
	{
		segment_type_e segType;
		if (true == roadSegConfig_gp->getSegmentType(segID, segType) && T_ROAD_CROSS == segType)
		{
			for(int i = 0; i < handledSeg.size(); i++)
			{
				if(segID == handledSeg[i])
				{
					segID++;
					fgSegItor++;
					continue;
				}
			}
			CrossTVirtualCon(segID, *fgSegItor, fgData, handledSeg);
			handledSeg.push_back(segID);
		}
		
		recursiveDeleteEmptyLine(*fgSegItor);
		segID++;
		fgSegItor++;
	}

	return;
}
#endif

void CRoadVecGen3::getFgData(OUT list<list<vector<point3D_t>>> &fgData)
{
    // if output database is empty, clear output data
    if (_fgOutputList.empty())
    {
        fgData.clear();
        return;
    }
    fgData.clear();


    // iterate each sections, copy lines of _fgOutputList into fgData
	list<foregroundSectionData> fgOutputTemp = _fgOutputList;
	vector<int> handledSeg;
	list<foregroundSectionData>::iterator segItor = fgOutputTemp.begin();
	while(segItor != fgOutputTemp.end())
	{
		segment_type_e segType;
		if (true == roadSegConfig_gp->getSegmentType(segItor->sectionId, segType) && T_ROAD_CROSS == segType)
		{
			for(int i = 0; i < handledSeg.size(); i++)
			{
				if(segItor->sectionId == handledSeg[i])
				{
					segItor++;
					continue;
				}
			}
			crossTCurveFitting(segItor->sectionId, (*segItor), fgOutputTemp, handledSeg);
			handledSeg.push_back(segItor->sectionId);
		}

		segItor++;
	}

	handledSeg.clear();
	segItor = fgOutputTemp.begin();
	while(segItor != fgOutputTemp.end())
	{
		segment_type_e segType;
		if (true == roadSegConfig_gp->getSegmentType(segItor->sectionId, segType) && T_ROAD_CROSS == segType)
		{
			for(int i = 0; i < handledSeg.size(); i++)
			{
				if(segItor->sectionId == handledSeg[i])
				{
					segItor++;
					continue;
				}
			}
			CrossTVirtualCon(segItor->sectionId, *segItor, fgOutputTemp, handledSeg);
			handledSeg.push_back(segItor->sectionId);
		}
		segItor++;
	}

    list<foregroundSectionData>::iterator currSecItor = fgOutputTemp.begin();
    foregroundSectionData curData;

	// background database section iterator used to get merged information
	list<backgroundSectionData>::iterator bgSecItor = _bgDatabaseList.begin();
	list<backgroundSectionData>::iterator bgSecRevItor = _bgRevDirList.begin();
	list<list<vector<point3D_t>>>::iterator laneItor, laneRevItor;

    while (currSecItor != fgOutputTemp.end())
    {
		curData = *currSecItor;			
		
		// current section data is empty, push empty lines to match lane number
        if (curData.fgSectionData.empty())
        {
            list<vector<point3D_t>> fglines;
            fgData.push_back(fglines);
			currSecItor++;
			bgSecItor++;
			bgSecRevItor++;
            continue;
        }

		//current section data is not empty,push non-empty lines of current section to fgData
		int reverseLaneNum = roadSegConfig_gp->getLaneNumInSeg(curData.sectionId,true);
		int forwardLaneNum = roadSegConfig_gp->getLaneNumInSeg(curData.sectionId,false);
		bool bCommLinesMerged = roadSegConfig_gp->getBothSideMergeFlag(curData.sectionId);

		int numOfUsedLines = 0;
		list<vector<point3D_t>> fglines;
		laneItor = bgSecItor->bgSectionData.begin();
		laneRevItor = bgSecRevItor->bgSectionData.begin();
		list<vector<point3D_t>>::iterator lineItor = curData.fgSectionData.begin();	
		while(lineItor != curData.fgSectionData.end())
		{
			if (numOfUsedLines < reverseLaneNum)
			{
				if(!bgSecRevItor->bgSectionData.empty())
				{
					if (laneRevItor != bgSecRevItor->bgSectionData.end())
					{
						if (!lineItor->empty())
						{
							if(!laneRevItor->empty())
							{
								lineItor->at(0).count = laneRevItor->front().at(0).count;
							}else
							{
								lineItor->at(0).count = LANE_END_LINE_FLAG;
							}
						}

						laneRevItor++;
					}
				}
			}
			else
			{
				if ((!bCommLinesMerged) && (numOfUsedLines == reverseLaneNum) && (reverseLaneNum != 0))
				{
					if (!lineItor->empty())
					{
						lineItor->at(0).count = LANE_END_LINE_FLAG;
					}
				}
				else
				{
					if (!bgSecItor->bgSectionData.empty())
					{
						if (laneItor != bgSecItor->bgSectionData.end())
						{
							if (!lineItor->empty())
							{
								if(!laneItor->empty())
								{
									lineItor->at(0).count = laneItor->front().at(0).count;
								}else
								{
									lineItor->at(0).count = LANE_END_LINE_FLAG;
								}
							}

							laneItor++;
						}else
						{
							if (!lineItor->empty())
							{
								lineItor->at(0).count = LANE_END_LINE_FLAG;
							}
						}
					}else
					{
						if (!lineItor->empty())
							{
								lineItor->at(0).count = LANE_END_LINE_FLAG;
							}
					}
				}
			}

			if (!lineItor->empty())
			{
				fglines.push_back(*(lineItor));
			}
		
			numOfUsedLines++;
			lineItor++;
		}

		fgData.push_back(fglines);	

		currSecItor++;	
		bgSecItor++;
		bgSecRevItor++;
	}
	return;
}

bool CRoadVecGen3::distanceOfPointToLine(IN point3D_t pointA, IN list<vector<point3D_t>> &segData, 
											IN int lineIndex, IN int connType, OUT double &distanc)
{
	point3D_t pointB, pointC;
	int index = 0;
	if(0 <= (lineIndex - 1))
	{
		index = lineIndex - 1;
	}
	else
	{
		if(segData.size() > (lineIndex + 1))
		{
			index = lineIndex + 1;
		}
		else
		{
			return false;						
		}
	}
	list<vector<point3D_t>>::iterator lineItor = segData.begin();
	advance(lineItor, index);
	if((*lineItor).empty())
	{
		return false;				
	}

	if(STARTPOINT == connType)
	{
		pointB = (*lineItor)[((*lineItor).size() / 4)];
		pointC = (*lineItor)[((*lineItor).size() / 2)];		
	}
	else
	{
		pointB = (*lineItor)[(3 * (*lineItor).size() / 4)];
		pointC = (*lineItor)[((*lineItor).size() / 2)];	
	}

	double a = _secRptDataObj.getLength(pointB, pointC);
	double b = _secRptDataObj.getLength(pointC, pointA);
	double c = _secRptDataObj.getLength(pointB, pointA);
	double p = (a + b + c) / (double)2;
	double s = sqrt(long double(p * (p - a) * (p - b) * (p - c)));
	distanc = (double)2 * s / a;	

	return true;
}

bool CRoadVecGen3::crossTCurveFitting(IN int segID, INOUT foregroundSectionData &currData, 
	INOUT list<foregroundSectionData> &fgData, INOUT vector<int> &handledSeg)
{
	if(currData.fgSectionData.empty() || fgData.empty())
	{
		return false;
	}

	// get matched line info of all lines in current section
	vector<uint32> prevId, nextId;
	if(false == roadSegConfig_gp->getPrevSegId(segID, prevId) || false == roadSegConfig_gp->getNextSegId(segID, nextId))
	{
		return false;
	}
	
	if ((1 >= prevId.size() && 1 >= nextId.size()) || (1 < prevId.size() && 1 < nextId.size()))
	{
		return false;
	}

	int currlines = 0;
	currlines = currData.fgSectionData.size();
	vector<point3D_t> **ppCurrLines = new vector<point3D_t> *[currlines];
	if(NULL == ppCurrLines)
	{
		printf("ERROR:NULL pointer.\n");
		return false;
	}

	list<vector<point3D_t>>::iterator lineItor = currData.fgSectionData.begin();
	for (int i = 0; i < currlines; i++)
	{
		ppCurrLines[i] = &(*lineItor);
		lineItor++;
	}

	do
	{
		int degree = 10;
		list<vector<lineConn>> MatchedLines;
		if(1 < nextId.size())
		{	
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfEndPoint(segID, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}		
		
			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				printf("section ID:%d matchedLines size is wrong.\n", segID);
				break;
			}	

			// get end points of current lines, and start points of next lines
			int LineIndex = 0;
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}

				bool findSame = false;
				for(int i = 0; i < handledSeg.size(); i++)
				{
					if((*Itor).front().connSegID == handledSeg[i])
					{
						findSame = true;
						break;
					}
				}
				if(true == findSame)
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				foregroundSectionData *nextSegData = NULL;
				if(false == getSectionFromInputFgout(fgData, (*Itor).front().connSegID, &nextSegData) ||
                    NULL == nextSegData)
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				if(false == GetLineFromSegmentOfFgout(nextSegData, (*Itor).front().connLineID, &pNextLine) ||
                    NULL == pNextLine || pNextLine->empty())
				{
					Itor++;
					LineIndex++;
					continue;				
				}

				vector<point3D_t> nextLineTmp, currLineTmp, wholeLine;
				vector<point3D_t>::reverse_iterator tmpItor = (*ppCurrLines[LineIndex]).rbegin();
				while (tmpItor != (*ppCurrLines[LineIndex]).rend())
				{
					currLineTmp.push_back(*tmpItor);
					tmpItor++;
				}

				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					tmpItor = (*pNextLine).rbegin();
					while (tmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*tmpItor);
						tmpItor++;
					}
				}

				double curTheta = _secRotAngle[segID - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(STARTPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (!(acuteAngle > 45.0 && acuteAngle < 135.0))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				double h1 = 0.0, h2 = 0.0;
				int refIndex = currLineTmp.size() / 4;
				point3D_t pointA = currLineTmp[refIndex];
				distanceOfPointToLine(pointA, currData.fgSectionData, LineIndex, ENDPOINT, h1);

				pointA = (*pNextLine)[refIndex];
				distanceOfPointToLine(pointA, (*nextSegData).fgSectionData, (*Itor).front().connLineID - 1, (*Itor).front().connType, h2);

				int totalNum = 100;
				int currNum = totalNum * h1 / (h1 + h2);
				int nextNum = totalNum * h2 / (h1 + h2);
				point3D_t currStart = currLineTmp[currNum - 1];
				point3D_t currEnd = currLineTmp.front();
				point3D_t nextStart = nextLineTmp[nextNum - 1];
				point3D_t nextEnd = nextLineTmp.front();
#if 0
				for(int i = 1; i < currNum + 1; i++)
				{
					double valueX = (double)i * (double)i * (nextStart.lon - nextEnd.lon) / (double)(currNum * currNum);
					double valueY = (double)i * (double)i * (nextStart.lat - nextEnd.lat) / (double)(currNum * currNum);
					currLineTmp[currNum - i].lon = currLineTmp[currNum - i].lon + valueX;
					currLineTmp[currNum - i].lat = currLineTmp[currNum - i].lat + valueY;		
				}
#endif

#if 1
				for(int j = 1; j < nextNum + 1; j++)
				{
					double valueX = (double)j * (double)j * (currStart.lon - nextEnd.lon) / (double)(nextNum * nextNum);
					double valueY = (double)j * (double)j * (currStart.lat - nextEnd.lat) / (double)(nextNum * nextNum);
					nextLineTmp[nextNum - j].lon = nextLineTmp[nextNum - j].lon + valueX;
					nextLineTmp[nextNum - j].lat = nextLineTmp[nextNum - j].lat + valueY;		
				}
#endif

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(currLineTmp.begin(), minDistItor);

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				(*ppCurrLines[LineIndex]).clear();
				(*pNextLine).clear();
				tmpItor = currLineTmp.rbegin();
				while (tmpItor != currLineTmp.rend())
				{
					(*ppCurrLines[LineIndex]).push_back(*tmpItor);
					tmpItor++;
				}

				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}

				Itor++;
				LineIndex++;
				continue;	
			}
			break;
		}

		if (1 < prevId.size())
		{
			if(false == roadSegConfig_gp->getMatchedLinesInfoOfStartPoint(segID, MatchedLines, REAL_LINE_CONNECTION))
			{
				printf("ERROR: section ID:%d  get matched lines info of end point Failed.\n");
				break;						
			}		

			if(MatchedLines.empty() || currlines != MatchedLines.size())
			{
				break;
			}	

			int LineIndex = 0;	
			list<vector<lineConn>> :: iterator Itor = MatchedLines.begin();
			while(Itor != MatchedLines.end())
			{					
				if((*Itor).empty() || NULL == ppCurrLines[LineIndex] || ppCurrLines[LineIndex]->empty())
				{
					Itor++;
					LineIndex++;
					continue;			
				}

				bool findSame = false;
				for(int i = 0; i < handledSeg.size(); i++)
				{
					if((*Itor).front().connSegID == handledSeg[i])
					{
						findSame = true;
						break;
					}
				}
				if(true == findSame)
				{
					Itor++;
					LineIndex++;
					continue;
				}

				vector<point3D_t> *pNextLine = NULL;
				foregroundSectionData *nextSegData = NULL;
				if(false == getSectionFromInputFgout(fgData, (*Itor).front().connSegID, &nextSegData) ||
                    NULL == nextSegData)
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				if(false == GetLineFromSegmentOfFgout(nextSegData, (*Itor).front().connLineID, &pNextLine) ||
                    NULL == pNextLine || pNextLine->empty())
				{
					Itor++;
					LineIndex++;
					continue;				
				}

				vector<point3D_t > currLineTmp, nextLineTmp;
				currLineTmp	= (*(ppCurrLines[LineIndex]));

				vector<point3D_t>::reverse_iterator tmpItor = (*pNextLine).rbegin();
				if(STARTPOINT == (*Itor).front().connType)
				{
					nextLineTmp = (*pNextLine);
				}
				else
				{
					tmpItor = (*pNextLine).rbegin();
					while (tmpItor != (*pNextLine).rend())
					{
						nextLineTmp.push_back(*tmpItor);
						tmpItor++;
					}
				}

				double curTheta = _secRotAngle[segID - 1];
				double nextTheta = _secRotAngle[(*Itor).front().connSegID - 1];
				double acuteAngle = abs(Pi - abs(curTheta - nextTheta));		
				if(STARTPOINT == (*Itor).front().connType)
				{
					acuteAngle = Pi - acuteAngle;
				}
				acuteAngle = (acuteAngle * 180) / Pi;
				if (!(acuteAngle > 45.0 && acuteAngle < 135.0))
				{
					Itor++;
					LineIndex++;
					continue;
				}

				double h1 = 0.0, h2 = 0.0;
				int refIndex = currLineTmp.size() / 4;
				point3D_t pointA = currLineTmp[refIndex];
				distanceOfPointToLine(pointA, currData.fgSectionData, LineIndex, STARTPOINT, h1);

				pointA = (*pNextLine)[refIndex];
				distanceOfPointToLine(pointA, (*nextSegData).fgSectionData, (*Itor).front().connLineID - 1, (*Itor).front().connType, h2);

				int totalNum = 130;
				int currNum = totalNum * h1 / (h1 + h2);
				int nextNum = totalNum * h2 / (h1 + h2);
				point3D_t currStart = currLineTmp[currNum - 1];
				point3D_t currEnd = currLineTmp.front();
				point3D_t nextStart = nextLineTmp[nextNum - 1];
				point3D_t nextEnd = nextLineTmp.front();
#if 0
				for(int i = 1; i < currNum + 1; i++)
				{
					double valueX = (double)i * (double)i * (nextStart.lon - nextEnd.lon) / (double)(currNum * currNum);
					double valueY = (double)i * (double)i * (nextStart.lat - nextEnd.lat) / (double)(currNum * currNum);
					currLineTmp[currNum - i].lon = currLineTmp[currNum - i].lon + valueX;
					currLineTmp[currNum - i].lat = currLineTmp[currNum - i].lat + valueY;		
				}
#endif

#if 1
				for(int j = 1; j < nextNum + 1; j++)
				{
					double valueX = (double)j * (double)j * (currStart.lon - nextEnd.lon) / (double)(nextNum * nextNum);
					double valueY = (double)j * (double)j * (currStart.lat - nextEnd.lat) / (double)(nextNum * nextNum);
					nextLineTmp[nextNum - j].lon = nextLineTmp[nextNum - j].lon + valueX;
					nextLineTmp[nextNum - j].lat = nextLineTmp[nextNum - j].lat + valueY;		
				}
#endif

				int minCurrInd = 0;
				int minNextInd = 0;
				if(false == findIndexPairOfMinDistancePoints(currLineTmp, nextLineTmp, minCurrInd, minNextInd))
				{
					Itor++;
					LineIndex++;
					continue;				
				}
				vector<point3D_t>::iterator minDistItor = currLineTmp.begin();
				advance(minDistItor, minCurrInd);
				currLineTmp.erase(currLineTmp.begin(), minDistItor);

				minDistItor = nextLineTmp.begin();
				advance(minDistItor, minNextInd);
				nextLineTmp.erase(nextLineTmp.begin(), minDistItor);

				(*ppCurrLines[LineIndex]).clear();
				(*pNextLine).clear();
				(*ppCurrLines[LineIndex]) = currLineTmp;

				if(STARTPOINT == (*Itor).front().connType)
				{
					(*pNextLine) = nextLineTmp;
				}
				else
				{
					tmpItor = nextLineTmp.rbegin();
					while (tmpItor != nextLineTmp.rend())
					{
						(*pNextLine).push_back(*tmpItor);
						tmpItor++;
					}
				}
				Itor++;
				LineIndex++;
			}
			break;
		}
	}while(0);

	freePointerArray(ppCurrLines, currlines);
	return true;
}

void CRoadVecGen3::getBgDatabaseData(OUT backgroundSectionData **bgSegData,
    IN bool revDirFlag/* = false*/)
{
    if (nullptr == bgSegData)
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


void CRoadVecGen3::getFgDatabaseData(OUT foregroundSectionData **fgSegData,
    IN  bool revDirFlag/* = false*/)
{
    if ( nullptr == fgSegData)
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


void CRoadVecGen3::getFgAllDatabaseData(OUT foregroundSectionData **fgSegData)
{
    if (nullptr == fgSegData)
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


void CRoadVecGen3::getFgOutDatabaseData(OUT foregroundSectionData **fgOutSegData)
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


void CRoadVecGen3::dotLineBlockIndex(IN  vector<point3D_t> &lineData,
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


void CRoadVecGen3::blockCombine(INOUT vector<int> &dotBlkIndexSt,
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


void CRoadVecGen3::polynomialFitting(IN    vector<point3D_t> &sourceLine,
    INOUT vector<point3D_t> &fittedLine, IN int degree)
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

        double mse = EMatrix(pDX, pDY, numOfSrcPnts, degree, normalization, coefficient);

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


void CRoadVecGen3::getLinePaintInfo(IN  vector<point3D_t> &sourceline,
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

	int numOfPoint = sourceline.size();
	if (1 == dotBlkIndexSt.size())
	{
		if (0 == dotBlkIndexSt.front() && ((numOfPoint - 1) == dotBlkIndexEd.front()) && (1e-7 > abs(sourceline.front().paintFlag - 0)))
		{
			return;
		}
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

void CRoadVecGen3::getFGLine(IN  vector<point3D_t> &bgline,
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

    if (!stInd.empty())
    {
	    for (int i = 0; i < stInd.front(); i++)
	    {
		    tmpline[i].paintFlag = 0.0;
	    }
    }

    if (!edInd.empty())
    {
	    for (int i = edInd.back(); i < numOfPnts; i++)
	    {
		    tmpline[i].paintFlag = 0.0;
	    }
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
    for (uint32 i = 0; i < fgline.size(); i++)
    {
		if(0.5 <= tmpline[i].paintFlag)
		{
			fgline[i].paintFlag = 1;
		}
		else
		{
			fgline[i].paintFlag = 0;
		}
    }
}

bool CRoadVecGen3::getNeighborLaneType(IN uint32 SegId, IN  list<reportSectionData> &sideSecData,
	OUT int                     &neigbourLaneType,
	OUT vector<point3D_t>       &leftline,
	OUT vector<point3D_t>       &rightline)
{
	list<vector<point3D_t>> sideLanelines;

	if(sideSecData.empty())
	{
		return false;
	}

	sideLanelines.clear();
	list<reportSectionData>::iterator secItor = sideSecData.begin();
	while (secItor != sideSecData.end())
	{
		if(SegId == secItor->sectionId || !secItor->rptSecData.empty())
		{
			sideLanelines.push_back(secItor->rptSecData.front().rptLaneData.front());
			sideLanelines.push_back(secItor->rptSecData.front().rptLaneData.back());
			break;
		}
		secItor++;
	}

	// check input parameters
	if (sideLanelines.empty() || (sideLanelines.front().size() != sideLanelines.back().size())
		|| leftline.empty() || rightline.empty() 
		|| (leftline.size() != rightline.size()))
	{
		return false;
	}

	int sideLinesNum = sideLanelines.front().size();
	int indexL = 0, indexR = 0;
	for(indexL = 0; indexL < sideLinesNum; indexL++)
	{
		if(1e-7 < abs(sideLanelines.front()[indexL].paintFlag - (-1)))
		{
			break;
		}
	}

	for(indexR = 0; indexR < sideLinesNum; indexR++)
	{
		if(1e-7 < abs(sideLanelines.back()[indexR].paintFlag - (-1)))
		{
			break;
		}
	}

	// number of sample line points
	int numOfPnts = leftline.size();

	// reset Y and paint for sample vector
	for (int i = 0; i < numOfPnts; i++)
	{
		leftline.at(i).lat          = 0.0;
		leftline.at(i).paintFlag    = 0.0;
		leftline.at(i).paintLength  = 0.0;
		leftline.at(i).count        = 0;
		rightline.at(i).lat         = 0.0;
		rightline.at(i).paintFlag   = 0.0;
		rightline.at(i).paintLength = 0.0;
		rightline.at(i).count = 0;
	}

	// rotate the input two lines first
	vector<point3D_t> leftRotated;
	vector<point3D_t> rightRotated;

	// current section rotation angle
	double theta = _secRotAngle[SegId - 1];
	lineRotation(sideLanelines.front(), -theta, leftRotated); // left line
	lineRotation(sideLanelines.back(), -theta, rightRotated); // right line

	// get paint information for re-sample lines
	getLinePaintInfo(leftRotated, leftline);
	getLinePaintInfo(rightRotated, rightline);

	// lane number estimation	
	int ltype = 0, rtype = 0;
	if(indexL >= sideLinesNum)
	{
		ltype = INVALID;
	}
	else
	{
		getLineType(leftline, _curSecId, ltype);
	}

	if(indexR >= sideLinesNum)
	{
		rtype = INVALID;
	}
	else
	{
		getLineType(rightline, _curSecId, rtype);
	}

	neigbourLaneType = (ltype << LEFT_LINE_TYPE_SHIFT) + rtype; 
	return true;
}

bool CRoadVecGen3::LaneDataPreprocessing(IN  list<vector<point3D_t>> &lanelines,
    IN  vector<point3D_t>             &leftsample,
    IN  vector<point3D_t>             &rightsample,
    IN  bool                          &bRevDir,
    OUT vector<int>                   &estLaneType,
    OUT list<list<vector<point3D_t>>> &twolines,
    OUT bool                          &bHasChangeLane)
{
    list<vector<point3D_t>> onelane;
    estLaneType.clear();
    twolines.clear();

    // check input parameters
    if (lanelines.empty() || leftsample.empty() || rightsample.empty() || leftsample.size() != rightsample.size())
    {
        return false;
    }

    // initialize leftsample, rightsample, matchedLane and twolines;
    int numOfPnts = leftsample.size(); // number of sample line points
    for(int i = 0; i < numOfPnts; i++)
    {
        leftsample.at(i).lat = 0.0;
        leftsample.at(i).paintFlag = 0.0;
        rightsample.at(i).lat = 0.0;
        rightsample.at(i).paintFlag = 0.0;
    }

    list<list<vector<point3D_t>>> allSeg;
    int sizeOfLaneLine = lanelines.front().size();
    bHasChangeLane = false;
    for(int i = 0; i < sizeOfLaneLine; i++)
    {
        if(1e-7 > abs(lanelines.front().at(i).paintFlag - 2))
        {
            bHasChangeLane = true;
            break;
        }
    }
    if(false == bHasChangeLane)
    {
        allSeg.push_back(lanelines);
    }
    else
    {
        // if only one lane in current section, ignore lane change
        if (1 == roadSegConfig_gp->getLaneNumInSeg(_curSecId, bRevDir))
        {
            for(int i = 0; i < sizeOfLaneLine; i++)
            {
                if(1e-7 > abs(lanelines.front().at(i).paintFlag - 2))
                {
                    lanelines.front().at(i).paintFlag = 0.0;
                }
                if(1e-7 > abs(lanelines.back().at(i).paintFlag - 2))
                {
                    lanelines.back().at(i).paintFlag = 0.0;
                }
            }

            bHasChangeLane = false;
            allSeg.push_back(lanelines);
        }
        else
        {
            SegChangLane(lanelines, allSeg);
        }
    }
	segment_type_e segType = NORMAL_E;
	roadSegConfig_gp->getSegmentType(_curSecId, segType);

    list<list<vector<point3D_t>>>::iterator allSegItor = allSeg.begin();
    while(allSegItor != allSeg.end() && !allSegItor->empty())
    {
        vector<point3D_t> leftlineTemp = leftsample;
        vector<point3D_t> rightlineTemp = rightsample;
        int estLaneTypeTemp = INVALID_INVALID;

        // rotate the input two lines first
        vector<point3D_t> leftRotated, leftRotatedValid;
        vector<point3D_t> rightRotated, rightRotatedValid;

        // current section rotation angle
        double theta = _secRotAngle[_curSecId - 1];
        lineRotation(allSegItor->front(), -theta, leftRotated); // left line
        lineRotation(allSegItor->back(), -theta, rightRotated); // right line

        // get paint information for re-sample lines
        getLinePaintInfo(leftRotated, leftlineTemp);
        getLinePaintInfo(rightRotated, rightlineTemp);

        //lane number estimation
        laneNumberEst(leftlineTemp, rightlineTemp, estLaneTypeTemp);
        //estLaneType.push_back(estLaneTypeTemp);

#if VISUALIZATION_ON
        list<vector<point3D_t>> rotated;
        rotated.push_back(leftRotated);
        rotated.push_back(rightRotated);
        sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_rotated.png",
            FG_MERGED_NUM, _curSecId, matchedLaneTemp, MERGED_TIMES[_curSecId - 1]);
        showImage(rotated, Scalar(0, 255, 0), IMAGE_NAME_STR2);
#endif

        if (INVALID_INVALID != estLaneTypeTemp)
        {

#if VISUALIZATION_ON
            sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_source.png",
                FG_MERGED_NUM, _curSecId, matchedLaneTemp, MERGED_TIMES[_curSecId - 1]);
            showImage(*allSegItor, Scalar(0, 255, 0), IMAGE_NAME_STR2);
#endif
			estLaneType.push_back(estLaneTypeTemp);
            // extract valid points for re-sampling or polynomial fitting
            for (uint32 i = 0; i < leftRotated.size(); i++)
            {
                if (0 <= leftRotated[i].paintFlag)
                {
                    leftRotatedValid.push_back(leftRotated[i]);
                }
            }
            for (uint32 i = 0; i < rightRotated.size(); i++)
            {
                if (0 <= rightRotated[i].paintFlag)
                {
                    rightRotatedValid.push_back(rightRotated[i]);
                }
            }

            // data fitting or interpolation re-sample
            vector<point3D_t> leftpola = leftlineTemp, leftpoly = leftlineTemp,
                rightpola = rightlineTemp, rightpoly = rightlineTemp;

            interpolationSample(leftRotatedValid,  leftpola);
            interpolationSample(rightRotatedValid, rightpola);

            polynomialFitting(leftRotatedValid,  leftpoly, DEFAULT_DEGREE);
            polynomialFitting(rightRotatedValid, rightpoly, DEFAULT_DEGREE);

            // according to mean , standard derivation and range of latitude to choose interpolation or
			// polynomial fitting for input data
			int bodyStartIdx = _secBodyStInd[_curSecId - 1];
			int bodyEndIdx = _secBodyEdInd[_curSecId - 1];

			int validNumOfPnts = bodyEndIdx - bodyStartIdx + 1;

			vector<double> dy0, dy1;
			double meanY0 = 0.0, stdY0 = 0.0, meanY1 = 0.0, stdY1 = 0.0, tmp = 0.0,	tmpMinPola = 30.0, tmpMinPoly = 30.0;
			for (int i = bodyStartIdx; i <= bodyEndIdx; i++)
			{
				tmpMinPola = 30.0;
				tmpMinPoly = 30.0;
				for(int j = 0; j < numOfPnts; j ++)
				{
					tmp = sqrt((leftpola[i].lat - rightpola[j].lat)*(leftpola[i].lat - rightpola[j].lat) + 
						(leftpola[i].lon - rightpola[j].lon)*(leftpola[i].lon - rightpola[j].lon));
					if (tmp < tmpMinPola)
					{
						tmpMinPola = tmp;
					}
				}
				dy0.push_back(tmpMinPola);

				// polynomial fitting
				for(int j = 0; j < numOfPnts; j ++)
				{
					tmp = sqrt((leftpoly[i].lat - rightpoly[j].lat)*(leftpoly[i].lat - rightpoly[j].lat) + 
						(leftpoly[i].lon - rightpoly[j].lon)*(leftpoly[i].lon - rightpoly[j].lon));
					if (tmp < tmpMinPoly)
					{
						tmpMinPoly = tmp;
					}
				}
				dy1.push_back(tmpMinPoly);
			}

			// mean and standard derivation
			for (int i = 0; i < validNumOfPnts; i ++)
			{
				meanY0 += dy0[i];
				meanY1 += dy1[i];
			}
			meanY0 /= validNumOfPnts;
			meanY1 /= validNumOfPnts;

			for (int i = 0; i < validNumOfPnts; i++)
			{
				stdY0 += (dy0[i] - meanY0) * (dy0[i] - meanY0);
				stdY1 += (dy1[i] - meanY1) * (dy1[i] - meanY1);
			}
			stdY0 = sqrt(stdY0 / validNumOfPnts);
			stdY1 = sqrt(stdY1 / validNumOfPnts);

			double dMaxPolyLeftLat = leftpoly[bodyStartIdx].lat;
			double dMinPolyLeftLat = leftpoly[bodyStartIdx].lat;
			double dMaxPolaLeftLat = leftpola[bodyStartIdx].lat;
			double dMinPolaLeftLat = leftpola[bodyStartIdx].lat;
            double dMaxPolyRightLat = rightpoly[bodyStartIdx].lat;
            double dMinPolyRightLat = rightpoly[bodyStartIdx].lat;
            double dMaxPolaRightLat = rightpola[bodyStartIdx].lat;
            double dMinPolaRightLat = rightpola[bodyStartIdx].lat;
			for (int i = bodyStartIdx+1; i <= bodyEndIdx; i++)
			{
				double tmp = leftpoly[i].lat;
				if(tmp < dMinPolyLeftLat)		dMinPolyLeftLat = tmp;
				if(tmp > dMaxPolyLeftLat)		dMaxPolyLeftLat = tmp;

				tmp = leftpola[i].lat;
				if(tmp < dMinPolaLeftLat)		dMinPolaLeftLat = tmp;
				if(tmp > dMaxPolaLeftLat)		dMaxPolaLeftLat = tmp;

                tmp = rightpoly[i].lat;
                if (tmp < dMinPolyRightLat) { dMinPolyRightLat = tmp; }
                if (tmp > dMaxPolyRightLat) { dMaxPolyRightLat = tmp; }

                tmp = rightpola[i].lat;
                if (tmp < dMinPolaRightLat) { dMinPolaRightLat = tmp; }
                if (tmp > dMaxPolaRightLat) { dMaxPolaRightLat = tmp; }
			}

			double rangePola = max(dMaxPolaLeftLat - dMinPolaLeftLat, dMaxPolaRightLat - dMinPolaRightLat);
			double rangePoly = max(dMaxPolyLeftLat - dMinPolyLeftLat, dMaxPolyRightLat - dMinPolyRightLat);

            bool cond00 = 3.0 < abs(meanY0) && abs(meanY0) < 6.0;
            bool cond01 = stdY0 < 1 && rangePola < 50.0;
            bool cond10 = 3.0 < abs(meanY1) && abs(meanY1) < 6.0;
            bool cond11 = stdY1 < 1 && rangePoly < 50.0;

            _curSecSmplType = USE_INTERPOLATION;
            if (cond00 && cond01 && cond10 && cond11)
            {
                if (stdY0 < stdY1)
                {
                    // interpolation
                    _curSecSmplType = USE_INTERPOLATION;
                }
                else
                {
                    // polynomial
                    _curSecSmplType = USE_POLYNOMIALFIT;
                }
            }
            else if (cond00 && cond01)
            {
                // interpolation
                _curSecSmplType = USE_INTERPOLATION;
            }
            else if (cond10 && cond11)
            {
                // polynomial
                _curSecSmplType = USE_POLYNOMIALFIT;
            }
            else
            {
#if VISUALIZATION_ON || SAVE_DATA_ON
                printf("section %d matchedLane %d merging %d is not valid\n",
                    _curSecId, matchedLaneTemp, MERGED_TIMES[_curSecId - 1]);
#endif

                //printf("Error: can not find method to re-sample: segId %d.\n", _curSecId);
                estLaneType.pop_back();

                allSegItor++;
                continue;
            }

            if (USE_INTERPOLATION == _curSecSmplType)
            {
                interpolationSample(leftRotatedValid,  leftlineTemp);
                interpolationSample(rightRotatedValid, rightlineTemp);
#if VISUALIZATION_ON
                list<vector<point3D_t>> intersampled_;
                intersampled_.push_back(leftpola);
                intersampled_.push_back(rightpola);
                sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_interpolate_sample_.png",
                    FG_MERGED_NUM, _curSecId, matchedLaneTemp, MERGED_TIMES[_curSecId - 1]);
                showImage(intersampled_, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
            }
            else
            {
                polynomialFitting(leftRotatedValid,  leftlineTemp, DEFAULT_DEGREE);
                polynomialFitting(rightRotatedValid, rightlineTemp, DEFAULT_DEGREE);
#if VISUALIZATION_ON
                list<vector<point3D_t>> polysampled_;
                polysampled_.push_back(leftpoly);
                polysampled_.push_back(rightpoly);
                sprintf_s(IMAGE_NAME_STR2, "fg_%d_sec_%d_lane_%d_merging_%d_polyval_sample_.png",
                    FG_MERGED_NUM, _curSecId, matchedLaneTemp, MERGED_TIMES[_curSecId - 1]);
                showImage(polysampled_, Scalar(0, 0, 255), IMAGE_NAME_STR2);
#endif
            }

            if(true == bHasChangeLane)
            {
                vector<double> leftRotatedLon, rightRotatedLon;
                int rotatedLineSize = leftRotated.size();

                for(int i = 0; i < rotatedLineSize; i++)
                {
                    if(0 < leftRotated.at(i).lon || 0 > leftRotated.at(i).lon)
                    {
                        leftRotatedLon.push_back(leftRotated.at(i).lon);
                    }
                    if(0 < rightRotated.at(i).lon || 0 > rightRotated.at(i).lon)
                    {
                        rightRotatedLon.push_back(rightRotated.at(i).lon);
                    }
                }
                double rotatedFrontLon = min(leftRotatedLon.front(), rightRotatedLon.front());
                double rotatedBackLon = max(leftRotatedLon.back(), rightRotatedLon.back());
                int index1 = int(floor((rotatedFrontLon - leftlineTemp.at(0).lon) / (leftlineTemp.at(1).lon - leftlineTemp.at(0).lon)));
                int index2 = int(floor((rotatedBackLon - leftlineTemp.at(0).lon) / (leftlineTemp.at(1).lon - leftlineTemp.at(0).lon)));

                index1 = (index1 >= 0) ? index1 : 0;
                index1 = (index1 <= numOfPnts) ? index1 : numOfPnts;
                index2 = (index2 >= 0) ? index2 : 0;
                index2 = (index2 <= numOfPnts) ? index2 : numOfPnts;

                if(index1 < index2)
                {
                    for(int i = 0; i < index1; i++)
                    {
                        leftlineTemp.at(i).paintFlag = -1;
                        rightlineTemp.at(i).paintFlag = -1;
                    }
                    for(int i = index2 + 1; i < numOfPnts; i++)
                    {
                        leftlineTemp.at(i).paintFlag = -1;
                        rightlineTemp.at(i).paintFlag = -1;
                    }
                }
                else
                {
                    for(int i = 0; i < index2; i++)
                    {
                        leftlineTemp.at(i).paintFlag = -1;
                        rightlineTemp.at(i).paintFlag = -1;
                    }
                    for(int i = index1 + 1; i < numOfPnts; i++)
                    {
                        leftlineTemp.at(i).paintFlag = -1;
                        rightlineTemp.at(i).paintFlag = -1;
                    }
                }
            }
#if 0
			if(T_ROAD_CROSS == segType)
			{
				double latAverage = 0.0;
				for(int i = 0; i < leftlineTemp.size(); i++)
				{
					latAverage +=  leftlineTemp[i].lat;
				}
				latAverage = latAverage / (double) leftlineTemp.size();

				for(int i = 0; i < leftlineTemp.size(); i++)
				{
					leftlineTemp[i].lat = latAverage;
				}

				latAverage = 0.0;
				for(int i = 0; i < rightlineTemp.size(); i++)
				{
					latAverage +=  rightlineTemp[i].lat;
				}
				latAverage = latAverage / (double) rightlineTemp.size();

				for(int i = 0; i < rightlineTemp.size(); i++)
				{
					rightlineTemp[i].lat = latAverage;
				}
			}
#endif
            onelane.push_back(leftlineTemp);
            onelane.push_back(rightlineTemp);
            twolines.push_back(onelane);
        }
        onelane.clear();
        allSegItor++;
    }
    if(estLaneType.empty() || twolines.empty())
    {
        return false;
    }
    else
    {
        return true;
    }
}

void CRoadVecGen3::getLineType(IN  vector<point3D_t> &line,
	IN uint32  segId, OUT int32  &lineType)
{
	// checkout input data
	if (line.empty())
	{
		lineType = INVALID;
		return;
	}

	// get dash line start / end index current line
	vector<int> stInd, edInd;
	int numOfBlks = 0, indSt = 0, indEd = 0;
	double mLine = 0.0, sLine = 0.0, tmp = 0.0;
	vector<double> lenLine;

	dotLineBlockIndex(line, stInd, edInd);

	// get line data with section body
	numOfBlks = stInd.size();
	for (int i = 0; i < numOfBlks; i++)
	{
		// max of body start index
		indSt = max(stInd[i], _secBodyStInd[segId - 1]);
		// min of body end index
		indEd = min(edInd[i], _secBodyEdInd[segId - 1]);

		if ((indSt < indEd) &&
			(indSt >= _secBodyStInd[segId - 1]) &&
			(indEd <= _secBodyEdInd[segId - 1]))
		{
			tmp = abs(indEd - indSt);
			lenLine.push_back(tmp);
			mLine += tmp;
		}
	}

	if (lenLine.empty())
	{
		lineType = INVALID;
		return;
	}

	// mean length and standard derivation
	mLine = mLine / lenLine.size();
	for (uint32 i = 0; i < lenLine.size(); i++)
	{
		sLine += (lenLine[i] - mLine) * (lenLine[i] - mLine);
	}
	sLine = sqrt(sLine / lenLine.size());

	if (mLine > 100)
	{
		lineType = 1;
	}
	else if (mLine > 0)
	{
		if (((mLine + sLine) > 100) && (sLine / mLine > 0.5))
		{
			lineType = 1;
		}
		else
		{
			lineType = 0;
		}
	}
	else
	{
		lineType = 0;
	}

	return;
}

void CRoadVecGen3::laneNumberEst(IN  vector<point3D_t> &leftline,
	IN  vector<point3D_t> &rightline,
	OUT int32             &estLaneType)
{
	int ltype = 0, rtype = 0;
	getLineType(leftline, _curSecId, ltype);
	getLineType(rightline, _curSecId, rtype);
	if(INVALID == ltype || INVALID == rtype)
	{
		estLaneType = INVALID_INVALID;
	}
	else
	{
		estLaneType = (ltype << LEFT_LINE_TYPE_SHIFT) + rtype;
	}	
	return;
}

/*
void CRoadVecGen3::laneNumberEst(IN  vector<point3D_t> &leftline,
    IN  vector<point3D_t> &rightline,
    OUT int32             &estLaneType)
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
        estLaneType = INVALID_LANE_NUM;
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


#if 1

    int ltype = 0, rtype = 0;
    if (mL > 100)
    {
        ltype = 1;
    }
    else if (mL > 0)
    {
        if (((mL + sL) > 100) && (sL / mL > 0.5))
        {
            ltype = 1;
        }
        else
        {
            ltype = 0;
        }
    }
    else
    {
        ltype = 0;
    }

    if (mR > 100)
    {
        rtype = 1;
    }
    else if (mR > 0)
    {
        if (((mR + sR) > 100) && (sR / mR > 0.5))
        {
            rtype = 1;
        }
        else
        {
            rtype = 0;
        }
    }
    else
    {
        rtype = 0;
    }

    int lanetype = SOLID_SOLID;
    if (ltype == 1 && rtype == 1)
    {
        lanetype = SOLID_SOLID;
    }

    if (ltype == 1 && rtype == 0)
    {
        lanetype = SOLID_DASH;
    }

    if (ltype == 0 && rtype == 1)
    {
        lanetype = DASH_SOLID;
    }

    if (ltype == 0 && rtype == 0)
    {
        lanetype = DASH_DASH;
    }

#else

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

#endif
	estLaneType = lanetype;
	
}
*/

/*
void CRoadVecGen3::checkCircleRoad()
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
*/
/*
void CRoadVecGen3::getMatchedLineInd(OUT vector<int> &matchedInd,
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

#if defined(_US_PALO_ALTO_VIDEO)
        if (segId == 1)
        {
            matchedInd.clear();
            matchedInd.push_back(-1);
            matchedInd.push_back(-1);
            matchedInd.push_back(0);
            matchedInd.push_back(-1);
            matchedInd.push_back(2);
            matchedInd.push_back(3);
        }
#endif
    }
}
*/


void CRoadVecGen3::SegChangLane(IN list<vector<point3D_t>> &laneRpt,
    OUT list<list<vector<point3D_t>>> &allSeg)
{
    if (laneRpt.empty())
    {
        return;
    }

    allSeg.clear();
    int numOfPnts = laneRpt.front().size(); // number of total points in a line
    bool paintIsTwo = false;                // judge that if paintFlag is equal to 2
    vector<int> start;                      // start points of changing lane areas
    vector<int> stop;                       // end point of changing lane areas
    struct point3D_t zeroPoint = { 0.0, 0.0, 0.0, -1.0, 0, 0.0 };

    // obtain index of endpoints of areas with paintFlag equals 2
    for(int i = 0; i < numOfPnts; i++)
    {
        if((1e-7 > abs(laneRpt.front().at(i).paintFlag - 2) || 1e-7 > abs(laneRpt.back().at(i).paintFlag - 2)) &&
            false == paintIsTwo)
        {
            paintIsTwo = true;
            start.push_back(i);
        }
        if((2 > laneRpt.front().at(i).paintFlag && 2 > laneRpt.back().at(i).paintFlag) &&
            true == paintIsTwo && !start.empty())
        {
            paintIsTwo = false;
            stop.push_back(i - 1);
        }
    }

    // divide the lane into several parts based on changing lane areas
    if(!start.empty() && !stop.empty())
    {
        if(start.size() > stop.size())
        {
            start.pop_back();
        }
        if(start.front() == 0)
        {
            start.erase(start.begin());
            stop.erase(stop.begin());
        }
        start.insert(start.begin(), -1);
        stop.insert(stop.begin(), -1);
        start.push_back(numOfPnts);
        stop.push_back(numOfPnts);
        list<vector<point3D_t>> oneSeg; // one element of allSeg

        // construct allSeg
        int size = start.size();
        for(int i = 0; i < size - 1; i++)
        {
            vector<point3D_t> tempLeft(numOfPnts, zeroPoint);
            vector<point3D_t> tempRight(numOfPnts, zeroPoint);
            for(int j = stop.at(i) + 1; j < start.at(i + 1); j++)
            {
                tempLeft.at(j).lon = laneRpt.front().at(j).lon;
                tempLeft.at(j).lat = laneRpt.front().at(j).lat;
                tempLeft.at(j).alt = laneRpt.front().at(j).alt;
                tempLeft.at(j).paintFlag = laneRpt.front().at(j).paintFlag;
                tempLeft.at(j).count = laneRpt.front().at(j).count;
                tempLeft.at(j).paintLength = laneRpt.front().at(j).paintLength;
                if (1e-7 >abs(tempLeft.at(j).paintFlag -2))
                {
                    tempLeft.at(j).lon = 0.0;
                    tempLeft.at(j).lat = 0.0;
                    tempLeft.at(j).alt = 0.0;
                    tempLeft.at(j).paintFlag = -1.0;
                    tempLeft.at(j).count = 0;
                    tempLeft.at(j).paintLength = 0.0;
                }

                tempRight.at(j).lon = laneRpt.back().at(j).lon;
                tempRight.at(j).lat = laneRpt.back().at(j).lat;
                tempRight.at(j).alt = laneRpt.back().at(j).alt;
                tempRight.at(j).paintFlag = laneRpt.back().at(j).paintFlag;
                tempRight.at(j).count = laneRpt.back().at(j).count;
                tempRight.at(j).paintLength = laneRpt.back().at(j).paintLength;
                if (1e-7 >abs(tempRight.at(j).paintFlag -2))
                {
                    tempRight.at(j).lon = 0.0;
                    tempRight.at(j).lat = 0.0;
                    tempRight.at(j).alt = 0.0;
                    tempRight.at(j).paintFlag = -1.0;
                    tempRight.at(j).count = 0;
                    tempRight.at(j).paintLength = 0.0;
                }
            }
            oneSeg.push_back(tempLeft);
            oneSeg.push_back(tempRight);
            allSeg.push_back(oneSeg);
            oneSeg.clear();
        }
    }
    else
    {
        for (int i = 0; i < numOfPnts; i++)
        {
            if (1e-7 >abs(laneRpt.front().at(i).paintFlag -2))
            {
                laneRpt.front().at(i).lon = 0.0;
                laneRpt.front().at(i).lat = 0.0;
                laneRpt.front().at(i).alt = 0.0;
                laneRpt.front().at(i).paintFlag = -1.0;
                laneRpt.front().at(i).count = 0;
                laneRpt.front().at(i).paintLength = 0.0;
            }
            if (1e-7 >abs(laneRpt.back().at(i).paintFlag -2))
            {
                laneRpt.back().at(i).lon = 0.0;
                laneRpt.back().at(i).lat = 0.0;
                laneRpt.back().at(i).alt = 0.0;
                laneRpt.back().at(i).paintFlag = -1.0;
                laneRpt.back().at(i).count = 0;
                laneRpt.back().at(i).paintLength = 0.0;
            }
        }
        allSeg.push_back(laneRpt);
    }
}

void CRoadVecGen3::setBgOldDataProportion(IN list<vector<point3D_t>> &laneDb, 
					OUT double &leftProportion, OUT double &rightProportion)
{
	if(laneDb.empty() || laneDb.front().empty() || laneDb.back().empty())
	{
		return;
	}

	if(0 > laneDb.front().at(0).count || 0 > laneDb.back().at(0).count)
	{
		return;
	}

	double maxProportion = 0.9, tmpProportion = 0.0;
	int totalCount = laneDb.front().at(0).count + 1;
	tmpProportion = (double)(laneDb.front().at(0).count) / (double)totalCount;
	if(0 > tmpProportion || 1 < tmpProportion)
	{
		return;
	}

	if(maxProportion < tmpProportion && 1 > tmpProportion)
	{
		tmpProportion = maxProportion;
	}
	leftProportion = tmpProportion;

	totalCount = laneDb.back().at(0).count + 1;
	tmpProportion = (double)(laneDb.back().at(0).count) / (double)totalCount;
	if(0 > tmpProportion || 1 < tmpProportion)
	{
		return;
	}

	if(maxProportion < tmpProportion && 1 > tmpProportion)
	{
		tmpProportion = maxProportion;
	}
	rightProportion = tmpProportion;

	return;
}

void CRoadVecGen3::MergForChangLane(IN list<vector<point3D_t>> &laneRpt,
    INOUT list<vector<point3D_t>> &laneDb)
{
    if (laneRpt.empty() || laneDb.empty())
    {
        return;
    }

    list<vector<point3D_t>> laneDbOld = laneDb; // original laneDb
    int numOfPnts = laneDb.front().size(); // number of total points in a line

	double leftProportion = 0.75, rightProportion = 0.75;
	segment_type_e segType = NORMAL_E;
	roadSegConfig_gp->getSegmentType(_curSecId, segType);
	//if(T_ROAD_CROSS == segType)
	//{
		setBgOldDataProportion(laneDb, leftProportion, rightProportion);
	//}	

    // obtain the index of the points with paintFlag != -1
    vector<int> indexOfNMone; // index of points whose paintFlag are not equal to -1
    for(int i = 0; i < numOfPnts; i++)
    {
        if(-1 < laneRpt.front().at(i).paintFlag)
        {
            indexOfNMone.push_back(i);
        }
    }

    if(!indexOfNMone.empty())
    {
        // merge this area of laneRpt with the corresponding area of laneDb
        double dLeft = 0, dRight = 0;
        int countLeft = 0, countRight = 0;
        for(int i = indexOfNMone.front(); i < indexOfNMone.back() + 1; i++)
        {
            float weight = (float)((0.0 < laneDb.front().at(i).paintLength && 0.0 < laneRpt.front().at(i).paintLength) ? 0.5 : 1.0);
            laneDb.front().at(i).lat = leftProportion * laneDb.front().at(i).lat + (1 - leftProportion) * laneRpt.front().at(i).lat;
            laneDb.front().at(i).paintFlag = float(laneDb.front().at(i).paintFlag + laneRpt.front().at(i).paintFlag);
            laneDb.front().at(i).paintLength = weight * laneDb.front().at(i).paintLength + 
                weight * laneRpt.front().at(i).paintLength;

            weight = (float)((0.0 < laneDb.back().at(i).paintLength && 0.0 < laneRpt.back().at(i).paintLength) ? 0.5 : 1.0);
            laneDb.back().at(i).lat = rightProportion * laneDb.back().at(i).lat + (1 - rightProportion) * laneRpt.back().at(i).lat;
            laneDb.back().at(i).paintFlag = float(laneDb.back().at(i).paintFlag + laneRpt.back().at(i).paintFlag);
            laneDb.back().at(i).paintLength = weight * laneDb.back().at(i).paintLength + 
                weight * laneRpt.back().at(i).paintLength;

            dLeft += laneDb.front().at(i).lat - laneDbOld.front().at(i).lat;
            dRight += laneDb.back().at(i).lat - laneDbOld.back().at(i).lat;
            countLeft++;
            countRight++;
        }
        double dLeftMean = dLeft / countLeft;
        double dRightMean = dRight / countRight;

        // translation of the areas which were not merged
        for(int i = 0; i < indexOfNMone.front(); i++)
        {
            laneDb.front().at(i).lat = laneDb.front().at(i).lat + dLeftMean;
            laneDb.front().at(i).paintFlag = (0 < laneDb.front().at(i).paintFlag) ? float(laneDb.front().at(i).paintFlag + 1) : laneDb.front().at(i).paintFlag;
            laneDb.back().at(i).lat = laneDb.back().at(i).lat + dRightMean;
            laneDb.back().at(i).paintFlag = (0 < laneDb.back().at(i).paintFlag) ? float(laneDb.back().at(i).paintFlag + 1) : laneDb.back().at(i).paintFlag;
        }
        for(int i = indexOfNMone.back() + 1; i < numOfPnts; i++)
        {
            laneDb.front().at(i).lat = laneDb.front().at(i).lat + dLeftMean;
            laneDb.front().at(i).paintFlag = (0 < laneDb.front().at(i).paintFlag) ? float(laneDb.front().at(i).paintFlag + 1) : laneDb.front().at(i).paintFlag;
            laneDb.back().at(i).lat = laneDb.back().at(i).lat + dRightMean;
            laneDb.back().at(i).paintFlag = (0 < laneDb.back().at(i).paintFlag) ? float(laneDb.back().at(i).paintFlag + 1) : laneDb.back().at(i).paintFlag;
        }
    }
}

const string NumTostring(int num)
{  
    ostringstream oss;  
    oss << num;
    return oss.str();      
}

#if 0
bool CRoadVecGen3::BGSectionDataSave(const string path)
{
    list<backgroundSectionData>::iterator iter_seg;
    string s = path;
    s.operator+=("FileIndex.txt");
	FILE *fp_fileindex = fopen(s.c_str(),"w+");
	if(NULL == fp_fileindex)
	{
        return("Open index file failed!\n");
	}
	
	string s1 = path;
	s1.operator+=("segment");
	
    //backgrounddata
    for(iter_seg = _bgDatabaseList.begin(); iter_seg != _bgDatabaseList.end(); iter_seg++)
    {
        //sectionId
        uint32 segid = iter_seg->sectionId;
		string s_tmp = s1;
        string num = NumTostring(segid);
        s_tmp.operator+=(num);
        s_tmp.operator+=(".txt");

		FILE *fp = fopen(s_tmp.c_str(),"w+");
		if(NULL == fp)
		{
		    fclose(fp_fileindex);
			return false;
		}
		
		//output the filename into indexfile
		s_tmp = "segment.txt\n";
        fpos_t pos = s_tmp.find(".txt",0); 
		//s_tmp.insert(29,num);
        s_tmp.insert(pos,num);
		fprintf(fp_fileindex,s_tmp.c_str());
		//segmentid
        fprintf(fp,"SegmentID:%d\n",segid);

        list<list<vector<point3D_t>>>::iterator iter_lane;
        uint32 lanenum = 0;
        for(iter_lane=iter_seg->bgSectionData.begin(); iter_lane != iter_seg->bgSectionData.end();iter_lane++)
        {
            fprintf(fp,"laneID:%d\n",lanenum);
            lanenum++;
            //empty 
            if(iter_lane->empty())
            {
                continue;
            }

            if(2 != iter_lane->size())// 2 line in one lane
            {
                printf("Save background line error!\n");
				fclose(fp);
				fclose(fp_fileindex);
                return false;
            }

            list<vector<point3D_t>>::iterator iter_line = iter_lane->begin();
            uint32 size = iter_line->size();
            for(uint32 index = 0;index < size ;index++)
            {
                //get point of line1 in the lane
                point3D_t p1 = iter_line->at(index);
                iter_line++;
                //get point of line2 in the lane
                point3D_t p2 = iter_line->at(index);
				iter_line--;

                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f,",p1.lat,p1.lon,p1.alt,p1.count,p1.paintFlag,p1.paintLength);
                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f\n",p2.lat,p2.lon,p2.alt,p2.count,p2.paintFlag,p2.paintLength);
            }
        }
		fclose(fp);
    }

	string s2 = path;
    s2.operator+=("segment_r");
    //reserve direction backgrounddata
    for(iter_seg = _bgRevDirList.begin(); iter_seg != _bgRevDirList.end(); iter_seg++)
    {
        //sectionId
        uint32 segid = iter_seg->sectionId;
		string s_tmp = s2;
        string num = NumTostring(segid);
        s_tmp.operator+=(num);
        s_tmp.operator+=(".txt");

		FILE *fp = fopen(s_tmp.c_str(),"w+");
		if(NULL == fp)
		{
		    fclose(fp_fileindex);
			return false;
		}
		
		//output the filename into indexfile
		s_tmp = "segment_r.txt\n";
        fpos_t pos = s_tmp.find(".txt",0); 
		s_tmp.insert(pos,num);
		//s_tmp.insert(9,num);
		fprintf(fp_fileindex,s_tmp.c_str());
		//segmentid
        fprintf(fp,"SegmentID:%d\n",segid);

        list<list<vector<point3D_t>>>::iterator iter_lane;
        uint32 lanenum = 0;
        for(iter_lane=iter_seg->bgSectionData.begin(); iter_lane != iter_seg->bgSectionData.end();iter_lane++)
        {
            fprintf(fp,"laneID:%d\n",lanenum);
            lanenum++;
            //empty 
            if(iter_lane->empty())
            {
                continue;
            }

            if(2 != iter_lane->size())// 2 line in one lane
            {
                printf("line error!\n");
				fclose(fp);
				fclose(fp_fileindex);
                return false;
            }

            list<vector<point3D_t>>::iterator iter_line = iter_lane->begin();
            uint32 size = iter_line->size();
            for(uint32 index = 0;index < size ;index++)
            {
                //get point of line1 in the lane
                point3D_t p1 = iter_line->at(index);
                iter_line++;
                //get point of line2 in the lane
                point3D_t p2 = iter_line->at(index);
                iter_line = iter_lane->begin();

                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f,",p1.lat,p1.lon,p1.alt,p1.count,p1.paintFlag,p1.paintLength);
                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f\n",p2.lat,p2.lon,p2.alt,p2.count,p2.paintFlag,p2.paintLength);
            }
        }
		fclose(fp);
    }

	fclose(fp_fileindex);
    return true;
}
#else
bool CRoadVecGen3::BGSectionDataSave(const string path)
{
    list<backgroundSectionData>::iterator iter_seg;
    string s = path;
    s.operator+=("bg_data.bin");
	//FILE *fp = fopen(s.c_str(),"w+");
	FILE *fp = fopen(s.c_str(),"wb+");
	if(NULL == fp)
	{
        return("Open index file failed!\n");
	}
		
    //backgrounddata
    for(iter_seg = _bgDatabaseList.begin(); iter_seg != _bgDatabaseList.end(); iter_seg++)
    {
        //sectionId
        uint32 segid = iter_seg->sectionId;	
        fprintf(fp,"SegmentID:%d\n",segid);

        list<list<vector<point3D_t>>>::iterator iter_lane;
        uint32 lanenum = 0;
        for(iter_lane=iter_seg->bgSectionData.begin(); iter_lane != iter_seg->bgSectionData.end();iter_lane++)
        {
            fprintf(fp,"laneID:%d\n",lanenum);
            lanenum++;
            //empty 
            if(iter_lane->empty())
            {
                continue;
            }

            if(2 != iter_lane->size())// 2 line in one lane
            {
                printf("Save background line error!\n");
				fclose(fp);
                return false;
            }

            list<vector<point3D_t>>::iterator iter_line = iter_lane->begin();
            uint32 size = iter_line->size();
            for(uint32 index = 0;index < size ;index++)
            {
                //get point of line1 in the lane
                point3D_t p1 = iter_line->at(index);
                iter_line++;
                //get point of line2 in the lane
                point3D_t p2 = iter_line->at(index);
				iter_line--;

                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f,",p1.lat,p1.lon,p1.alt,p1.count,p1.paintFlag,p1.paintLength);
                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f\n",p2.lat,p2.lon,p2.alt,p2.count,p2.paintFlag,p2.paintLength);
            }
        }
    }
	fclose(fp);

	s = path;
	s.operator+=("bgr_data.bin");
	//fp = fopen(s.c_str(),"w+");
	fp = fopen(s.c_str(),"wb+");
	if(NULL == fp)
	{
		return("Open index file failed!\n");
	}

	//background data reserve
    for(iter_seg = _bgRevDirList.begin(); iter_seg != _bgRevDirList.end(); iter_seg++)
    {
        //sectionId
        uint32 segid = iter_seg->sectionId;	
        fprintf(fp,"SegmentID:%d\n",segid);

        list<list<vector<point3D_t>>>::iterator iter_lane;
        uint32 lanenum = 0;
        for(iter_lane=iter_seg->bgSectionData.begin(); iter_lane != iter_seg->bgSectionData.end();iter_lane++)
        {
            fprintf(fp,"laneID:%d\n",lanenum);
            lanenum++;
            //empty 
            if(iter_lane->empty())
            {
                continue;
            }

            if(2 != iter_lane->size())// 2 line in one lane
            {
                printf("Save background line error!\n");
				fclose(fp);
                return false;
            }

            list<vector<point3D_t>>::iterator iter_line = iter_lane->begin();
            uint32 size = iter_line->size();
            for(uint32 index = 0;index < size ;index++)
            {
                //get point of line1 in the lane
                point3D_t p1 = iter_line->at(index);
                iter_line++;
                //get point of line2 in the lane
                point3D_t p2 = iter_line->at(index);
				iter_line--;

                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f,",p1.lat,p1.lon,p1.alt,p1.count,p1.paintFlag,p1.paintLength);
                fprintf(fp,"%.14f,%.14f,%.14f,%d,%f,%f\n",p2.lat,p2.lon,p2.alt,p2.count,p2.paintFlag,p2.paintLength);
            }
        }
    }	
	fclose(fp);

    return true;
}
#endif

bool CRoadVecGen3::BGSectionDataLoad(const string path)
{
    string s_in;
	
    s_in = path;
    s_in.operator+=("bg_data.bin");
    //FILE *fp = fopen(s_in.c_str(),"r");
    FILE *fp = fopen(s_in.c_str(),"rb+");
    if(NULL == fp)
    {
        printf("Can not open index file!\n");
        return false;
    }

    //background data		
    _bgDatabaseList.clear();    
    BGSectionDataAnalysis(fp , _bgDatabaseList);
	fclose(fp);

    s_in = path;
    s_in.operator+=("bgr_data.bin");
    FILE *fpr = fopen(s_in.c_str(),"rb+");
    //FILE *fpr = fopen(s_in.c_str(),"r");
    if(NULL == fpr)
    {
        printf("Can not open index file!\n");
        return false;
    }	

    //background data reserve	
    _bgRevDirList.clear();    
    BGSectionDataAnalysis(fpr , _bgRevDirList);
	fclose(fpr);
	
    return true;
}

bool CRoadVecGen3::BGSectionDataAnalysis(FILE *fp , list<backgroundSectionData> &databaseList)
{
    uint32 index=0;
    char ch[1000];

	if(NULL == fp)
	{
	    return false;
	}
	if(!databaseList.empty())
	{
		databaseList.clear();
	}
	
    fpos_t filepos;	   
    string s;	   
    size_t pos = 0;	  
    while(!feof(fp))
    {
 	   backgroundSectionData data_tmp;    
 	   
 	   /**** search the not NULL segment ****/
 	   fgetpos(fp , &filepos);//record position
 	   fscanf(fp,"%s\n",ch);//read whole row
 	   //find the "segment id:"
 	   s.assign(ch);
 	   if(std::string::npos != s.find("SegmentID:",pos))
 	   {		   
 		   fsetpos(fp , &filepos);
 		   fscanf(fp,"SegmentID:%d\n",&data_tmp.sectionId);
 		   databaseList.push_back(data_tmp);
 		   continue;
 	   }
 	   
 	   /**** analysis the segment content ****/ 	   
 	   databaseList.pop_back();
 	   fsetpos(fp , &filepos);//recover segment  file posision
 	   vector<point3D_t> line1;
 	   vector<point3D_t> line2; 
	   uint32 first_lane=0;
 	   while(!feof(fp))
 	   {
 		   fgetpos(fp , &filepos);//record position 
 		   fscanf(fp,"%s\n",ch);//read whole row
 		   s.assign(ch);
 		   if(std::string::npos != s.find("laneID:",pos))
 		   {
 		       if(0 == first_lane) //first lane
 		       {
 		           first_lane = 1;
                   continue;
			   }
			   
 			   list<vector<point3D_t>> lane; 
 			   if((!line1.empty()) && (!line1.empty()))
 			   {
 				   lane.push_back(line1); 
 				   lane.push_back(line2);
 				   line1.clear();
 				   line2.clear();
 			   }
			   data_tmp.bgSectionData.push_back(lane);
 		   }
 		   else if(std::string::npos != s.find("SegmentID:",pos))
 		   {
 		       // handle last lane
 			   list<vector<point3D_t>> lane;
 			   if((!line1.empty()) && (!line2.empty()))
 			   {
 				   lane.push_back(line1); 
 				   lane.push_back(line2);
 				   line1.clear();
 				   line2.clear();
 			   }
 			   data_tmp.bgSectionData.push_back(lane);

			   //push the data into list
 			   databaseList.push_back(data_tmp);
 
 			   fsetpos(fp , &filepos);		   
 			   break;
 		   }
 		   else
 		   {
 			   fsetpos(fp , &filepos);
 			   //get point of line1 && line2
 			   point3D_t p1;
 			   point3D_t p2;
 			   fscanf(fp,"%lf,%lf,%lf,%d,%f,%f,%lf,%lf,%lf,%d,%f,%f\n",
 					  &p1.lat,&p1.lon,&p1.alt,&p1.count,&p1.paintFlag,&p1.paintLength
 					 ,&p2.lat,&p2.lon,&p2.alt,&p2.count,&p2.paintFlag,&p2.paintLength);
 		
 			   line1.push_back(p1);
 			   line2.push_back(p2);
 
 			   if(feof(fp))
 			   {
 				   list<vector<point3D_t>> lane;
 				   lane.push_back(line1); //push line info
 				   lane.push_back(line2);
 				   data_tmp.bgSectionData.push_back(lane);
 				   databaseList.push_back(data_tmp);
 			   }
 		   }
 	    }
    }

	return true;
	
}

void CRoadVecGen3::preprocessRptData(INOUT list<list<vector<point3D_t>>>& rptData,
    IN list<vector<point3D_t>> &gpsData)
{
    int numOfLanes = rptData.size();
    int numOfTracks = gpsData.size();
    double distThreshold = 0.0005;

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
                int lCnt = 0, rCnt = 0, lrCnt = 0;
                double lDist = 0.0, rDist = 0.0, lrDist = 0.0;
                for (int i = 0; i < numOfVecPnts; ++i)
                {
                    bool bPaintL = false, bPaintR = false;
                    // painted point only
                    if (distThreshold >= abs(1.0 - laneIt->front().at(i).paintFlag))
                    {
                        double lat = laneIt->front().at(i).lat - trackIt->at(i).lat;
                        double lon = laneIt->front().at(i).lon - trackIt->at(i).lon;
                        lDist += sqrt(lat * lat + lon * lon);
                        lCnt++;

                        bPaintL = true;
                    }

                    if (distThreshold >= abs(1.0 - laneIt->back().at(i).paintFlag))
                    {
                        double lat = laneIt->back().at(i).lat - trackIt->at(i).lat;
                        double lon = laneIt->back().at(i).lon - trackIt->at(i).lon;
                        rDist += sqrt(lat * lat + lon * lon);
                        rCnt++;

                        bPaintR = true;
                    }

                    if (bPaintL && bPaintR)
                    {
                        double lat = laneIt->back().at(i).lat - laneIt->front().at(i).lat;
                        double lon = laneIt->back().at(i).lon - laneIt->front().at(i).lon;
                        lrDist += sqrt(lat * lat + lon * lon);
                        lrCnt++;
                    }
                }

                // add data for undetected points
                double mLDist = (0 < lCnt) ? (lDist / lCnt) : 0.0;
                double mRDist = (0 < rCnt) ? (rDist / rCnt) : 0.0;
                double mDist = (0 < lrCnt) ? (lrDist / lrCnt) : 0.0;

                if (distThreshold >= abs(mLDist))
                {
                    mLDist = mRDist;
                }
                if (distThreshold >= abs(mRDist))
                {
                    mRDist = mLDist;
                }
                if (distThreshold >= abs(mDist))
                {
                    mDist = mLDist + mRDist;
                }

                if (distThreshold >= abs(mDist))
                {
                    mDist = 3.5;
                    mLDist = 0.5 * mDist;
                    mRDist = 0.5 * mDist;
                }

                // for each undetected point
                for (int i = 0; i < numOfVecPnts - 1; i++)
                {
                    vector<double> vec, vecOrthL, vecOrthR;
                    vec.push_back(trackIt->at(i + 1).lon - trackIt->at(i).lon);
                    vec.push_back(trackIt->at(i + 1).lat - trackIt->at(i).lat);

                    getOrthonormalVec(vec, vecOrthL, vecOrthR);

                    if (distThreshold >= abs(-1.0 - laneIt->front().at(i).paintFlag))
                    {
                        laneIt->front().at(i).lon = trackIt->at(i).lon + vecOrthL[0] * mLDist;
                        laneIt->front().at(i).lat = trackIt->at(i).lat + vecOrthL[1] * mLDist;
                        laneIt->front().at(i).paintFlag = 0;
                    }

                    if (distThreshold >= abs(-1.0 - laneIt->back().at(i).paintFlag))
                    {
                        laneIt->back().at(i).lon = trackIt->at(i).lon + vecOrthR[0] * mRDist;
                        laneIt->back().at(i).lat = trackIt->at(i).lat + vecOrthR[1] * mRDist;
                        laneIt->back().at(i).paintFlag = 0;
                    }
                }
            }

            ++laneIt;
            ++trackIt;
        }
    }
}


void CRoadVecGen3::getOrthonormalVec(IN vector<double> &vec,
    OUT vector<double> &vecOrthL,
    OUT vector<double> &vecOrthR)
{
    double x = 0.0, y = 0.0;

    if (2 == vec.size())
    {
        if ((1e-7 < abs(vec[0])) || (1e-7 < abs(vec[1])))
        {
            x = sqrt(vec[1] * vec[1] / (vec[0] * vec[0] + vec[1] * vec[1]));
            x = (0.0 < vec[1]) ? -x : x;

            y = (1e-7 < abs(vec[1])) ? (-1.0 * vec[0] * x / vec[1]) : ((0 < vec[0]) ? 1.0 : -1.0);
        }
    }

    vecOrthL.clear(); vecOrthR.clear();
    vecOrthL.push_back(x); vecOrthL.push_back(y);
    vecOrthR.push_back(-x); vecOrthR.push_back((-y));
}


void CRoadVecGen3::unifySolidPaint(IN backgroundSectionData &bgSegData,
    IN foregroundSectionData &fgSegData,
    IN int                    laneNum,
    IN bool                   bRevDir)
{
    // check database data
    if (bgSegData.bgSectionData.empty() || fgSegData.fgSectionData.empty() ||
        bgSegData.sectionId != fgSegData.sectionId)
    {
        return;
    }

    int segId = bgSegData.sectionId;

    // only one lane
    if (1 == laneNum)
    {
        unifyLinePaint(fgSegData.fgSectionData.front(), segId, 0, LEFT_ROAD_LINE, bRevDir, true);
        unifyLinePaint(fgSegData.fgSectionData.back(), segId, 0, RIGHT_ROAD_LINE, bRevDir, true);
    }
    else if (laneNum == bgSegData.bgSectionData.size())
    {
        if(!bgSegData.bgSectionData.front().empty())
        {
            unifyLinePaint(fgSegData.fgSectionData.front(), segId, 0, LEFT_ROAD_LINE, bRevDir);
        }

        if (!bgSegData.bgSectionData.back().empty())
        {
            unifyLinePaint(fgSegData.fgSectionData.back(), segId, laneNum - 1, RIGHT_ROAD_LINE, bRevDir);
        }
    }
}


void CRoadVecGen3::unifyLinePaint(IN vector<point3D_t> &line,
    IN int segId,
    IN int laneIndex,
    IN int leftOrRight,
    IN bool bRevDir,
    IN bool bOneLane/* = false*/)
{
    // check line validation
    if (!line.empty())
    {
        // check whether there is more than threshold painted points
        int numOfPnts  = line.size();

        // if there is only one lane, ignore ratio check for this case
        if (!bOneLane)
        {
            float maxPaint = 1.0;
            for (int i = 0; i < numOfPnts; i++)
            {
                if (line.at(i).paintFlag > maxPaint)
                {
                    maxPaint = line.at(i).paintFlag;
                }
            }

            int numOfPaintedPnts = 0;
            for (int i = 0; i < numOfPnts; i++)
            {
                if (0.5 <= (line.at(i).paintFlag / maxPaint))
                {
                    numOfPaintedPnts++;
                }
            }

            double ratio = (1.0 * numOfPaintedPnts) / (1.0 * numOfPnts);
            if (UNIFORM_SOLID_WEIGHT >= ratio)
            {
                return;
            }
        }

        // painted all points according to line type
        int typeL = 0, typeR = 0;
        if (true == roadSegConfig_gp->getLineTypeOfCurrentLane(segId, laneIndex, bRevDir, typeL, typeR))
        {
            if ((SOLID == typeL) && (LEFT_ROAD_LINE == leftOrRight || LEFT_RIGHT_ROAD_LINE == leftOrRight))
            {
                for (int i = 0; i < numOfPnts; i++)
                {
                    line.at(i).paintFlag = 1.0;
                    line.at(i).paintLength = (float)numOfPnts;
                }
            }

            if ((SOLID == typeR) && (RIGHT_ROAD_LINE == leftOrRight || LEFT_RIGHT_ROAD_LINE == leftOrRight))
            {
                for (int i = 0; i < numOfPnts; i++)
                {
                    line.at(i).paintFlag = 1.0;
                    line.at(i).paintLength = (float)numOfPnts;
                }
            }
        }
    }
}


} // end of namespace ns_database


