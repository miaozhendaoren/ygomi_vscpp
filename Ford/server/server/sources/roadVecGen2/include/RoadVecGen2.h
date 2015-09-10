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
*******************************************************************************
*/

#pragma once

#include "apiDataStruct.h"
#include "ExtractSection.h"
#include <string>

namespace ns_database
{
    class CExtractSection;

    class CRoadVecGen2
    {
    public:
        CRoadVecGen2(void);
        CRoadVecGen2(string configFilePath);
        ~CRoadVecGen2(void);

        CRoadVecGen2(const CRoadVecGen2&) {}
        CRoadVecGen2& operator = (const CRoadVecGen2& ) {}

        
        /*
        * @FUNC
        *     Update road sections data with new reported data from vehicle.
        *
        * @PARAMS
        *     rptData - road lines of the lane vehicle reported.
        *     fgData  - updated data of road. there may be multiple sections, iterate
        *               one by one from first list. The second list stands for lane
        *               number and vector stands for lines of each lane.
        *               There may by empty items in the output.
        *
        */
        bool roadSectionsGen(IN  list<list<vector<point3D_t>>> &rptData,
                             OUT list<list<vector<point3D_t>>> &fgData);

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
        *     Set road section configuration file path in order to get configuration
        *     data
        *
        * @PARAMS
        *     filename - full path of configuration file.
        *
        */
        void setSectionConfigPath(IN string filename, OUT list<segAttributes_t> &segConfigList);

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
        *
        */
        void getBgRoadVec(OUT std::list<backgroundSectionData> &bgVecOut);

        /*
        * @FUNC
        *     Set back ground DB using input road vector list
        *
        * @PARAMS
        *     bgVecIn - input vector to set DB
        *
        */
        void setBgRoadVec(IN std::list<backgroundSectionData> &bgVecIn);

    protected:
        CExtractSection               _extractSecObj;

        std::string                   _configPath;

        sectionCon                    _stSecConfig;
        uint32                        _curSecId;       // current section ID
        vector<int>                   _secLaneNum;     // number of lanes for section
        vector<int>                   _secBodyStInd;   // section body data start index
        vector<int>                   _secBodyEdInd;   // section body data end index
        vector<double>                _secRotAngle;    // rotation angle for sections
        list<vector<point3D_t>>       _secLeftData;   // section left line sample data
        list<vector<point3D_t>>       _secRightData;  // section right line sample data

        list<segAttributes_t>         _segConfigList;  // section configurations
        list<backgroundSectionData>   _bgDatabaseList; // background database
        list<foregroundSectionData>   _fgDatabaseList; // foreground database
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
        void readSecConfig(OUT list<segAttributes_t> &segCfgList);

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
        void interpolationSample(IN    vector<point3D_t> &sourceLine,
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
        *     bgDatabaseData - background database data of current section,
        *                      stored lane by lane.
        *
        */
        bool mergeSectionLane(IN    reportSectionData     &reportData,
                              INOUT backgroundSectionData *bgDatabaseData);

        /*
        * @func
        *     Foreground database update. Merge common line of two adjacent lanes
        *     for all section data in background database.
        *
        * @params
        *     there is no input parameters as all are class members.
        *
        */
        bool stitchSectionLanes();

        /*
        * @FUNC
        *     Get corresponding section configuration data and background database
        *     data according to current section ID.
        *
        * @PARAMS
        *     bgSegData     - output extracted background data of input section
        *
        */
        void getDatabaseData(OUT backgroundSectionData        **bgSegData);

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
                               INOUT vector<point3D_t> &fittedLine);

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
        *     lanelines   - input lane data, which contains two lines.
        *     matchedLane - matched lane number of current section.
        *     leftline    - output left line data, X is re-sampled input.
        *     rightline   - output right line data, X is re-sampled input.
        *
        */
        bool LaneDataPreprocessing(IN  list<vector<point3D_t>> &lanelines,
                                   OUT int                     &matchedLane,
                                   OUT vector<point3D_t>       &leftline,
                                   OUT vector<point3D_t>       &rightline);

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
        *     matchedLane  - matched lane number.
        *
        */
        void laneNumberEst(IN  vector<point3D_t> &leftline,
                           IN  vector<point3D_t> &rightline,
                           OUT int               &matchedLane);

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
        * @PARAM
        *    fgData - output line data of foreground database after processing
        *             connection part.
        *
        */
        void jointProcessing(OUT list<list<vector<point3D_t>>> &fgData);

    };

}
