/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  RoadVecGen.h
* @brief This is class definition header file for RoadVecGen, which gets new
*        data from vehicle and merges them with database data to update road
*        lane information.
*
* Change Log:
*      Date                Who             What
*      2015/08/19       Ming Chen         Create
*******************************************************************************
*/

#include "apiDataStruct.h"
#include "ExtractSection.h"

namespace ns_database
{
    class CExtractSection;

    class CRoadVecGen
    {
    public:
        CRoadVecGen();
        CRoadVecGen(char *configFilePath);
        ~CRoadVecGen();

        CRoadVecGen(const CRoadVecGen&) {}
        CRoadVecGen& operator = (const CRoadVecGen& ) {}

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
        void roadSectionsGen(IN  list<list<vector<point3D_t>>>  rptData,
                             OUT list<list<vector<point3D_t>>> &fgData);

        /*
        * @FUNC
        *     Set road section configuration file path in order to get configuration
        *     data
        *
        * @PARAMS
        *     filename - full path of configuration file.
        *
        */
        void setSectionConfigPath(char *filename);

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


    protected:
        CExtractSection               _extractSecObj;
        sectionCon                    _stSecConfig;
        char                          _configPath[MAX_PATH];
        list<segAttributes_t>         _segConfigList; // section configurations
        list<backgroundSectionData>   _bgDatabaseList; // background database
        list<foregroundSectionData>   _fgDatabaseList; // foreground database

        /*
        * @FUNC
        *     Read section configuration file from input full path. The configuration
        *     is stored in _segConfigList.
        *
        * @PARAMS
        *
        */
        void readSecConfig();

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

        void calcRotationAngle(IN  segAttributes_t  sectionConfig,
                               OUT double          &theta,
                               OUT vector<double>  &xLimitation);

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
        bool mergeSectionLane(IN    segAttributes_t        sectionConfig,
                              IN    reportSectionData      reportData,
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
                             OUT segAttributes_t               &configSegData,
                             OUT backgroundSectionData        **bgSegData);

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
        void dotLineBlockIndex(IN  vector<point3D_t>  lineData,
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
        *     Get line estimation value, which is the mean and standard variation
        *     of line point block length. First get each block start/end index,
        *     then combine some small blocks to estimate each block length.
        *
        * @PARAMS
        *     lineData - input line to calculate estimation value.
        *
        */
        double getLineEstValue(IN vector<point3D_t> lineData);

        /*
        * @FUNC
        *     Polynomial curve line fitting for input line.
        *
        * @PARAMS
        *     soureLine  - input source line, X, Y is used to do polynomial fitting.
        *     fittedLine - output fitted line, X is used to calculate Y values.
        *
        */
        void polynomialFitting(IN    vector<point3D_t>  sourceLine,
                               INOUT vector<point3D_t> &fittedLine);

        /*
        * @FUNC
        *     Get paint information for input line.
        *
        * @PARAMS
        *     sourceline - input line data.
        *     destline   - destination line of output data with paint information
        *                  added. input x is used to do sample
        *
        */
        void getLinePaintInfo(IN  vector<point3D_t>  sourceline,
                              OUT vector<point3D_t> &destline);

    };

}
