/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  RoadVecGen2.h
* @brief This is class definition header file for RoadVecGen2, which gets new
*        data from vehicle and merges them with database data to update road
*        lane information. This is an updated version of RoadVecGen.
*
* Change Log:
*      Date                Who             What
*      2015/08/29       Ming Chen         Create
*      2015/11/3        Xin Shao          Modify
*******************************************************************************
*/

#pragma once

#include "apiDataStruct.h"
#include "SecRptData2.h"
#include "RoadSeg.h"
#include <string>

namespace ns_database
{
	#define Pi (3.1415926)
	#define POINT_TYPE_END (0)
	#define POINT_TYPE_START (1)

	typedef struct _rptLaneLink
	{
		int	  segId;
		bool  ValidFlag;	  
		bool  bHasChangeLane;
		bool  revDirect;
		int  turnTo;
		point3D_t fPointL;
		point3D_t fPointR;
		point3D_t bPointL;
		point3D_t bPointR;
		int   LlineId;
		int   RlineId;
		int   laneIndex;
	} rptLaneLink;

class CRoadVecGen3
{
public:
    CRoadVecGen3(void);
    //CRoadVecGen3(string configFilePath);
    ~CRoadVecGen3(void);

    CRoadVecGen3(const CRoadVecGen3&) {}
    CRoadVecGen3& operator = (const CRoadVecGen3& ) {}


    /*
    * @FUNC
    *     Update road sections data with new reported data from vehicle.
    *
    * @PARAMS
    *     rptData - road lines of the lane vehicle reported.
    *     gpsData - car GPS track data used to handle missing part.
    *     fgData  - updated data of road. there may be multiple sections, iterate
    *               one by one from first list. The second list stands for lane
    *               number and vector stands for lines of each lane.
    *               There may by empty items in the output.
    *     modifiedSectionId - used to save modified section's Id.
    *
    */
    bool roadSectionsGen(IN  list<list<vector<point3D_t>>> &rptData,
        IN  list<vector<point3D_t>> &gpsData,
        OUT list<list<vector<point3D_t>>> &fgData,
        OUT list<uint32> &modifiedSectionId);

    /*
    * @FUNC
    *     Update road sections data without new reported data from vehicle.
    *     Just generate foreground database from background data.
    *
    * @PARAMS
    *     fgData  - updated data of road. there may be multiple sections, iterate
    *               one by one from first list. The second list stands for lane
    *               number and vector stands for lines of each lane.
    *               There may by empty items in the output.
    *
    */
    bool roadSectionsGen(OUT list<list<vector<point3D_t>>> &fgData);

	
    /*
    * @FUNC
    *     init the internal parameters according to configure 
    */	
	void InitRoadVecGen();
	
    /*
    * @FUNC
    *     Set road section configuration file path in order to get configuration
    *     data
    *
    * @PARAMS
    *     filename - full path of configuration file.
    *
    */
    //void setSectionConfigPath(IN string filename, OUT list<segAttributes_t> &segConfigList);

    /*
    * @FUNC
    *     Given a point, get the section ID of this point.
    *
    * @PARAMS
    *     p  - input point.
    *
    */
    uint32 getSectionId(IN point3D_t p);

    /*
    * @FUNC
    *     Get road lines moved distance after merging.
    *
    * @PARAMS
    *     dist - shift distance of each line when doing data merging. The first
    *            value corresponds to the leftmost line, and the last value is
    *            the rightmost line, ...
    *
    */
    void getShitDist(OUT vector<double> &dist);

    /*
    * @FUNC
    *     Reset database. Release all background and foreground database data.
    *
    * @PARAMS
    *     No input/output parameters.
    *
    */
    void resetDatabase();

    /*
    * @FUNC
    *     Load predefined section data for special sections.
    *
    * @PARAMS
    *     segId    - which section to used pre-load data.
    *     filename - pre-load data, format is TXT.
    *
    */
    bool loadDefaultSegData(IN uint32 segId, IN string filename);

    /*
    * @FUNC
    *     Output back ground road vector list
    *
    * @PARAMS
    *     bgVecOut - output road vector list
    *     bRevDir  - save forward or backward direction database
    *
    */
    void getBgRoadVec(OUT std::list<backgroundSectionData> &bgVecOut,
        IN bool bRevDir = false);

    /*
    * @FUNC
    *     Set back ground DB using input road vector list
    *
    * @PARAMS
    *     bgVecIn - input vector to set DB
    *     bRevDir - set forward or backward direction DB
    *
    */
    void setBgRoadVec(IN std::list<backgroundSectionData> &bgVecIn,
        IN bool bRevDir = false);

    bool BGSectionDataSave(const string path);//save background data
    bool BGSectionDataLoad(const string path);//updata background data

protected:
    CSecRptData2                   _secRptDataObj;
    //std::string                   _configPath;
	int32                         _startSegId;

    bool                          _bHasRevDirData; // whether contains reverse data
    bool                          _bHasChanged;    // whether _bHasRevDirData be set
    //bool                          _bCircleRoad;
    uint32                        _curSecId;       // current section ID
    RESAMPLE_METHOD               _curSecSmplType; // current section re-sample method
    //vector<int>                   _secLaneNum;     // number of lanes for section
    vector<int>                   _secBodyStInd;   // section body data start index
    vector<int>                   _secBodyEdInd;   // section body data end index
    vector<double>                _secRotAngle;    // rotation angle for sections
    list<vector<point3D_t>>       _secLeftData;    // section left line sample data
    list<vector<point3D_t>>       _secRightData;   // section right line sample data
    //list<vector<int>>             _secLaneType;    // section lane type
    //list<vector<int>>             _secLaneConn;    // adjacent section connection relationship

    //list<segAttributes_t>         _segConfigList;  // section configurations
    list<backgroundSectionData>   _bgDatabaseList; // background database
    list<backgroundSectionData>   _bgRevDirList;   // reverse direction background database
    list<foregroundSectionData>   _fgDatabaseList; // foreground database
    list<foregroundSectionData>   _fgRevDirList;   // reverse direction foreground database
    list<foregroundSectionData>   _fgAllDirList;   // foreground all direction database
    list<foregroundSectionData>   _fgOutputList;   // foreground output data

    HANDLE                        _hMutexMerging;  // mutex handle for reset

    /*
    * @FUNC
    *     Read section configuration file from input full path. The configuration
    *     is stored in _segConfigList.
    *
    * @PARAMS
    *
    */
    //void readSecConfig(OUT list<segAttributes_t> &segCfgList);

    /*
    * @FUNC
    *     Initialize background and foreground database according to section
    *     configuration.
    *
    * @PARAMS
    */
    void initDatabase();

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
    void interpolationSample(IN vector<point3D_t> &sourceLine,
        INOUT vector<point3D_t> &sampledLine);

    /*
    * @FUNC
    *     Calculate each section line rotation angle and X data range
    *     according to section full configuration. The angle is the line
    *     from start points to end points.
    *
    * @PARAMS
    *     No input parameters. All used variables are class members.
    *     _secDataRange is used to store full section X data.
    *     _secBodyStInd is used to store each section body start index.
    *     _secBodyEdInd is used to store each section body end index.
    *     The size of _secDataRange, _secBodyStInd and _secBodyEdInd should
    *     be the same as _segConfigList.
    *
    */

    void calcRotAngleAndRange();

    /*
    * @FUNC
    *     Rotate input line according to rotation angle.
    *
    * @PARAMS
    *     sourceLine  - source line data of X, Y.
    *     theta       - rotation angle.
    *     rotatedLine - returned rotated line.
    *
    */
    void lineRotation(IN  vector<point3D_t> &sourceLine,
        IN  double             theta,
        OUT vector<point3D_t> &rotatedLine);

    /*
    * @func
    *     Background database update, update each lane separately.
    *
    * @params
    *     reportData     - reported data from vehicle, including lines of
    *                      the lane vehicle running.
    *
    */
    bool mergeSectionLane(IN reportSectionData &reportData);

    /*
    * @func
    *     Foreground database update. Merge common line of two adjacent lanes
    *     for all section data in background database.
    *
    * @params
    *     bRevDir - stitch forward or backward database
    *
    */
    bool stitchSectionLanes(IN bool bRevDir = false);

    /*
    * @func
    *     Foreground database update. Merge common line of two adjacent lanes
    *     for all section data in background database.
    *
    * @params
    *     bNeedMerge - whether the shared line should be merged
    *
    */
    bool stitchSharedLines(IN bool bNeedMerge = false);

    /*
    * @FUNC
    *     Get corresponding section configuration data and background database
    *     data according to current section ID.
    *
    * @PARAMS
    *     bgSegData     - output extracted background data of input section
    *
    */
    void getBgDatabaseData(OUT backgroundSectionData **bgSegData,
        IN  bool revDirFlag = false);

    /*
    * @FUNC
    *     Get corresponding section configuration data and foreground database
    *     data according to current section ID.
    *
    * @PARAMS
    *     fgSegData     - output extracted foreground data of input section
    *
    */
    void getFgDatabaseData(OUT foregroundSectionData  **fgSegData,
        IN  bool revDirFlag = false);

    /*
    * @FUNC
    *     Get corresponding section configuration data and foreground database
    *     data according to current section ID.
    *
    * @PARAMS
    *     fgSegData     - output extracted foreground data of input section
    *
    */
    void getFgAllDatabaseData(OUT foregroundSectionData  **fgSegData);

    /*
    * @FUNC
    *     Get corresponding section configuration data and output foreground
    *     database data according to current section ID.
    *
    * @PARAMS
    *     fgOutSegData  - output extracted foreground data of input section
    *
    */
    void getFgOutDatabaseData(OUT foregroundSectionData **fgOutSegData);


    /*
    * @FUNC
    *     Get dash line start and end index from line data.
    *
    * @PARAMS
    *     lineData      - input line data to check dash line paint index.
    *     dotBlkIndexSt - output of all dash line start index.
    *     dotBlkIndexEd - output of all dash line end index.
    *
    */
    void dotLineBlockIndex(IN  vector<point3D_t> &lineData,
        OUT vector<int>       &dotBlkIndexSt,
        OUT vector<int>       &dotBlkIndexEd);

    /*
    * @FUNC
    *     Combine dash line block index if the length is within threshold.
    *
    * @PARAMS
    *     dotBlkIndexSt - all dash line start index. output merged start index.
    *     dotBlkIndexEd - all dash line end index. output merged end index.
    */
    void blockCombine(INOUT vector<int> &dotBlkIndexSt,
        INOUT vector<int> &dotBlkIndexEd);

    /*
    * @FUNC
    *     Polynomial curve line fitting for input line.
    *
    * @PARAMS
    *     soureLine  - input source line, X, Y is used to do polynomial fitting.
    *     fittedLine - output fitted line, X is used to calculate Y values.
    *
    */
    void polynomialFitting(IN    vector<point3D_t> &sourceLine,
        INOUT vector<point3D_t> &fittedLine, IN int degree);

    /*
    * @FUNC
    *     Get paint information for input line.
    *
    * @PARAMS
    *     sourceline - input line data.
    *     destline   - destination line of output data with paint information
    *                  added. input X is used to do sample
    *
    */
    void getLinePaintInfo(IN  vector<point3D_t> &sourceline,
        OUT vector<point3D_t> &destline);

    /*
    * @FUNC
    *     Get foreground database line data according to input and rotation
    *     angle.
    *
    * @PARAMS
    *     bgline   - background database line data.
    *     theta    - section rotation angle
    *     fgline   - normalized paint information foreground base line
    *
    */
    void getFGLine(IN  vector<point3D_t> &bgline,
        double                 theta,
        OUT vector<point3D_t> &fgline);

    /*
    * @FUNC
    *     Preprocessing for input lane data(two lines).
    *
    * @PARAMS
    *     lanelines      - input lane data, which contains two lines.
    *     leftsample     - original sample of _secLeftData.
    *     rightsample    - original sample of _secRightData.
    *     estLaneType    - a set of estimated lane line types of current section.
    *     twolines       - output left line data and right line data.
    *     bHasChangeLane - if has changing lane areas.
    *
    */
    bool LaneDataPreprocessing(IN  list<vector<point3D_t>> &lanelines,
        IN  vector<point3D_t>             &leftsample,
        IN  vector<point3D_t>             &rightsample,
        IN  bool                          &bRevDir,
        OUT vector<int>                   &estLaneType,
        OUT list<list<vector<point3D_t>>> &twolines,
        OUT bool                          &bHasChangeLane);

    /*
    * @FUNC
    *     Estimate lane number of two input lines. Mean and standard
    *     derivation of dash line length(start/end) are used to decide
    *     line type. The correlation between left and right line is also
    *     used to judge lane type.
    *
    * @PARANS
    *     leftline     - left line of input lane data, X is re-sampled
    *     rightline    - right line of input lane data, X is re-sampled
    *     estLaneType  - estimated lane line type.
    *
    */
    void laneNumberEst(IN  vector<point3D_t> &leftline,
        IN  vector<point3D_t> &rightline,
        OUT int               &estLaneType);

    /*
    * @FUNC
    *     Remove foreground database section overlap and output lines.
    *
    * @PARAMS
    *     fgData - output line data of foreground database without overlap.
    *
    */
    void removeOverlap(OUT list<list<vector<point3D_t>>> &fgData);

    /*
    * @FUNC
    *     Remove foreground database section overlap and output lines.
    *
    * @PARAMS
    *
    */
    void removeOverlap();

    /*
    * @FUNC
    *    Process adjacent section connection data.
    *
    * @PARAMS
    *    fgData - output line data of foreground database after processing
    *             connection part.
    * bCommLinesMerged - whether forward and backward direction shared lines
    *                    is merged or not.
    *
    */
    void jointProcessing(OUT list<list<vector<point3D_t>>> &fgData);

    /*
    * @FUNC
    *    Check whether current road is circle based on adjacent section
    *    connection relationship.
    *
    * @PARAMS
    *    save the value in member _bCircleRoad.
    *
    */
    //void checkCircleRoad(void);

    /*
    * @FUNC
    *    Matched current section lines with previous section lines.
    *
    * @PARAMS
    *    matchedInd  - matched section line index.
    *    segId       - previous section Id.
    *    bHasRevData - whether contains reverse direction data.
    *    bCommLinesMerged - whether share lines be merged.
    *
    */
    //void getMatchedLineInd(OUT vector<int> &matchedInd,
    //    IN uint32 segId,
    //    IN bool   bHasRevData,
    //    IN bool   bCommLinesMerged);

	int CRoadVecGen3::getSectionFromOutputList(IN int SegID, OUT foregroundSectionData **pSegData);

	int CRoadVecGen3::getContinousValidIndex(INOUT vector<int> &validLaneInd);

	bool CRoadVecGen3::GetLineFromOutputList(IN int SegID, IN int LineID, OUT vector<point3D_t> **pLine);

	void CRoadVecGen3::freePointerArray(vector<point3D_t> **ppCurrLines, int currlines);

	void CRoadVecGen3::getFgData(OUT list<list<vector<point3D_t>>> &fgData);

	void CRoadVecGen3::adjacentEndOrStartPoint(IN foregroundSectionData *currData, IN int flag, OUT list<list<vector<ns_roadsegment::lineConn>>> &lines);

	void CRoadVecGen3::adjacentMultiplePoint(IN int flag, IN list<list<vector<ns_roadsegment::lineConn>>> &lines);

	void CRoadVecGen3::adjacentOneSeg(IN int flag, IN list<vector<ns_roadsegment::lineConn>> &lines);

	int CRoadVecGen3::devideMatchLinesIntoSegGroups(IN list<vector<ns_roadsegment::lineConn>> MatchedLines, 
													OUT list<vector<ns_roadsegment::lineConn>> MatchedSegment[4], 
													OUT list<list<vector<ns_roadsegment::lineConn>>> &lines,
													OUT int &segNum);

	int CRoadVecGen3::devideMatchLinesIntoSegGroups(IN list<vector<ns_roadsegment::lineConn>> MatchedLines, OUT list<list<vector<ns_roadsegment::lineConn>>> &lines);
	
	int CRoadVecGen3::getNumOfValidLanesInCurrBgSeg(IN backgroundSectionData &bgSegData);

    /*
    * @FUNC
    *     Divide input lane data with changing lane area into several parts.
    *
    * @PARAMS
    *     laneRpt   - input lane data, which contains changing lane areas.
    *     allSeg - several parts of laneRpt divided by changing lane areas.
    *
    */
	void CRoadVecGen3::SegChangLane(IN list<vector<point3D_t>> &laneRpt,
		OUT list<list<vector<point3D_t>>> &allSeg);

    /*
    * @FUNC
    *     Merge processing for a part before or after a changing lane area.
    *
    * @PARAMS
    *     laneRpt   - a part before or after a changing lane area.
    *     laneDb - lane data base.
    *
    */
    void MergForChangLane(IN list<vector<point3D_t>> &laneRpt,
        INOUT list<vector<point3D_t>> &laneDb);

	void CRoadVecGen3::getLineType(IN  vector<point3D_t> &line,
		IN uint32  segId, OUT int32  &lineType);

	bool CRoadVecGen3::getNeighborLaneType(IN uint32 SegId, IN  list<reportSectionData> &sideSecData,
		OUT int                     &neigbourLaneType,
		OUT vector<point3D_t>       &leftline,
		OUT vector<point3D_t>       &rightline);

	bool CRoadVecGen3::mergeSectionLane(IN reportSectionData &reportData, IN list<reportSectionData> &sideSecData,
		IN vector<rptLaneLink> &laneLink);

	bool CRoadVecGen3::getSideLane(IN  list<list<vector<point3D_t>>> &rptData,
		OUT list<list<vector<point3D_t>>> &mainRptData, 
		OUT list<list<vector<point3D_t>>> &sideRptData);

	bool CRoadVecGen3::ContinuousLaneLinkTrack(IN vector<rptLaneLink> &laneLink,
		INOUT rptLaneLink &currSegLane, IN bool revDirect, IN vector<int> &matchLaneIdx);

	void CRoadVecGen3::judgeDiversion(IN vector<rptLaneLink> &laneLink,
		INOUT rptLaneLink &currSegLane, IN list<vector<point3D_t>> &twoLines, IN bool revDirect);

	bool CRoadVecGen3::getPrevSegLaneOfTrack(IN vector<rptLaneLink> &laneLink,
		IN rptLaneLink &currSegLane, OUT rptLaneLink &prevSegLane);
    bool BGSectionDataAnalysis(FILE *fp , list<backgroundSectionData> &databaseList);

    /*
    * @FUNC
    *     Pre-processing reported new data. For undetected part, use car GPS
    *     track to calculate left and right points. The width is calculated
    *     by using detected relative points.
    *
    * @PARAMS
    *     rptData - reported left and right lines data.
    *     gpsData - car GPS track data.
    *
    */
    void preprocessRptData(INOUT list<list<vector<point3D_t>>> &rptData,
            IN list<vector<point3D_t>> &gpsData);

    /*
    * @FUNC
    *     Get orthonormal vector of a 2D input vector.
    *
    * @PARAM
    *     vec      - input 2D vector
    *     vecOrthL - left orthonormal vector
    *     vecOrthR - right orthonormal vector
    *
    */
    void getOrthonormalVec(IN vector<double> &vec,
        OUT vector<double> &vecOrthL,
        OUT vector<double> &vecOrthR);

    /*
    * @FUNC
    *     Get main lane and side lane GPS track.
    *
    * @PARAMS
    *     gpsData - car GPS track data.
    *     mainGpsData - main lane GPS track.
    *     sideGpsData - side lane GPS track.
    *
    */
    bool getSideLaneGps(IN list<vector<point3D_t>> &gpsData,
        OUT list<vector<point3D_t>> &mainGpsData,
        OUT list<vector<point3D_t>> &sideGpsData);

	void CRoadVecGen3::crossTHandle(IN foregroundSectionData *currData);

	void CRoadVecGen3::CrossTVirtualCon(IN foregroundSectionData *currData);

	void CRoadVecGen3::CrossTVirtualCon(IN int segID, IN foregroundSectionData &segData, 
			INOUT list<foregroundSectionData> &fgData, INOUT vector<int> &handledSeg);

	bool CRoadVecGen3::findIndexPairOfMinDistancePoints(IN vector<point3D_t> &currLine, 
		IN vector<point3D_t> &nextLine, OUT int &minCurrInd, OUT int &minNextInd);

	bool CRoadVecGen3::fitTwoSerialLinesToOne(IN int currSegId, IN vector<point3D_t> &currLine, 
		IN int nextSegId, IN vector<point3D_t> &nextLine, IN int degree, OUT vector<point3D_t> &wholeLine);

	bool CRoadVecGen3::GetLineFromInputSegment(IN foregroundSectionData *SegData, IN int LineID, OUT vector<point3D_t> **pLine);

	bool CRoadVecGen3::cutFittedCurveToTwoLinesByBody(IN vector<point3D_t> &wholeLine, IN int currSegId, 
		IN int currConType, OUT vector<point3D_t> &currLine, OUT vector<point3D_t> &nextLine);

	void CRoadVecGen3::getModifiedSectionId(IN  list<reportSectionData> &secData,
		IN list<list<vector<point3D_t>>> &fgData, OUT list<uint32> &modifiedSectionId);

	bool CRoadVecGen3::findSameSectionId(IN  vector<uint32> &allId, IN uint32 segId);

	void CRoadVecGen3::adjustForwardAndReverseGap(IN bool bNeedMerge, foregroundSectionData *fgSegAllData);

	void CRoadVecGen3::setBgOldDataProportion(IN list<vector<point3D_t>> &laneDb, 
					OUT double &leftProportion, OUT double &rightProportion);

	void CRoadVecGen3::preprocessVirtualConnection(IN foregroundSectionData *currData, INOUT vector<point3D_t> **ppCurrLines);

	bool CRoadVecGen3::GetLineFromInputFgout(IN list<foregroundSectionData> &fgData, 
						IN int segId, IN int LineID, OUT vector<point3D_t> **pLine);

	bool CRoadVecGen3::getSectionFromInputFgout(IN list<foregroundSectionData> &fgData, IN int SegID, 
	OUT foregroundSectionData **pSegData);

	bool CRoadVecGen3::GetLineFromSegmentOfFgout(IN foregroundSectionData *pSegData, 
						IN int LineID, OUT vector<point3D_t> **pLine);

	bool CRoadVecGen3::distanceOfPointToLine(IN point3D_t pointA, IN list<vector<point3D_t>> &segData, 
											IN int lineIndex, IN int connType, OUT double &distanc);

	bool CRoadVecGen3::crossTCurveFitting(IN int segID, INOUT foregroundSectionData &currData, 
	INOUT list<foregroundSectionData> &fgData, INOUT vector<int> &handledSeg);

    /*
    * @FUNC
    *     find the closest point index of input line to fixed point, and erase
    *     points over the index according to flag.
    *
    * @PARAMS
    *     line   - input line data.
    *     fixPnt - fixed end point of matched line.
    *     flag   - line end point match flag.
    *
    */
    void findAndErasePointsOverMinDist(IN vector<point3D_t> &line,
        IN point3D_t &fixPoint,
        IN int flag);

    /*
    * @FUNC
    *     Get uniform paint for solid lines. The condition is that for one lane
    *     road, if the configuration is solid then paint all points as paint.
    *     For more than one lane, only check the boundaries, if configuration is
    *     solid line and more then a threshold value of painted points, then
    *     make all points painted.
    *
    * @PARAMS
    *     bgSegData - background section data.
    *     fgSegData - foreground section data.
    *     laneNum   - configuration number of lanes in the given direction.
    *     bRevDir   - whether the road is forward or backward lanes.
    *
    */
    void unifySolidPaint(IN backgroundSectionData &bgSegData,
        IN foregroundSectionData &fgSegData,
        IN int                    laneNum,
        IN bool                   bRevDir);

    /*
    * @FUNC
    *     Unify line paint according to section configuration.
    *
    * @PARAM
    *     line        - need to unify paint line data.
    *     segId       - current section Id.
    *     laneIndex   - current lane index of the direction.
    *     leftOrRight - check with left or right line type.
    *     bRevDir     - whether the road is forward or backward lanes.
    *     bOneLane    - whether there is only one lane.
    *
    */
    void unifyLinePaint(IN vector<point3D_t> &line,
        IN int segId,
        IN int laneIndex,
        IN int leftOrRight,
        IN bool bRevDir,
        IN bool bOneLane = false);

	void CRoadVecGen3::prevAndNextSegmentInquireLoop(IN int segId, INOUT vector<uint32> &allId);
};

}
