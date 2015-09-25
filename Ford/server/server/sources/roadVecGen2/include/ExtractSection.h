/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  ExtractSection.h
* @brief This is class definition header file for extractSection, which extract 
*        sections from report new data according to section configure  
*
* Change Log:
*      Date                Who             What
*      2015/08/20       Zhong Ning         Create
*******************************************************************************
*/

#ifndef __EXTRACT_SECTION_H__
#define __EXTRACT_SECTION_H__

#include "apiDataStruct.h"
#include "configure.h"

namespace ns_database
{
#define  CONJOINTSECID 22
#define  STARTSECID    1
#define  MINPOINTNUM   500
#define  SECTIONDEBUG  11

#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
    #define  ROADCIRCLE    1
#elif (RD_LOCATION == RD_GERMAN_LEHRE)
    #define  ROADCIRCLE    0
#elif (RD_LOCATION == RD_US_DETROIT)
    #define  ROADCIRCLE    1
#endif

    enum ISTATUS
    {
        I_SUCCESS = 0,
        I_WARING  = 1,
        I_FAIL    = -1
    };

    class CExtractSection
    {
    public:
        CExtractSection() {}
        ~CExtractSection() {}

        CExtractSection(const CExtractSection&){}
        CExtractSection& operator = (const CExtractSection&){}

        /*
        *@FUNC
        *    Extract sections from report data according to section configure
        *
        *@PARAMS
        *    ltSectionDataScale - each section scale 
        *    stsectionConfig    - section attribute configure
        *    ltRptData          - new report data
        *    ltRptSectionData   - after extract section from new report data
        *
        */

        ISTATUS extractSections(IN list<segAttributes_t>          &ltSectionDataScale,
                                IN  sectionCon                    &stSectionConfig,
                                IN list<list<vector<point3D_t>>>  &ltRptData,
                                OUT list<reportSectionData>       &ltRptSectionData);

        /*
        *@FUNC
        *    Calculate  and return road distance
        *
        *@PARAMS
        *    LaneData - report lane data 
        * 
        */

        double calcLaneLength(IN list<vector<point3D_t>> &ltLaneData);

        /*
        *@FUNC
        *    Locate the overlap area that head of data and tail of data,
        *
        *@PARAMS
        *    ltLaneData              - report lane data 
        *    stSectionCon            - section attribute configure
        *    ltEffectiveLaneData     - data that not contain coordinate value of point is 0
        which means that point has not been detected
        *    ltMainEffectiveLaneData - data between uiStartLoc and uiEndLoc of ltEffectiveLaneData
        *    uiStartLoc              - data that exclude head overlap area the start location
        *    uiEndLoc                - data that exclude tail overlap area the end location
        *    fLength                 - report lane length
        *
        */
        ISTATUS getReportHeadandTailOverlapLocation(IN list<vector<point3D_t>>  &ltLaneData,
                                                    IN sectionCon               &stSectionCon,
                                                    OUT list<vector<point3D_t>> &ltEffectiveLaneData,
                                                    OUT list<vector<point3D_t>> &ltMainEffectiveLaneData,
                                                    OUT uint32                  &uiStartLoc,
                                                    OUT uint32                  &uiEndLoc,
                                                    OUT double                  &dbLength);


        /*
        *@FUNC
        *    Search the lane change area,that area 
        *
        *@PARAMS
        *    LaneData     - report lane data 
        *    stSectionCon - section attribute configure
        *    uiStartLoc   - data that exclude head overlap area the start location
        *    uiEndLoc     - data that exclude tail overlap area the end location
        *
        */

        ISTATUS splitChangeLaneData(IN list<vector<point3D_t>>        &newReportData,
                                    OUT list<list<vector<point3D_t>>> &ltSubReportData);

        /*
        *@FUNC
        *    Locate the overlap area that head of data and tail of data
        *
        *@PARAMS
        *    LaneData     - report lane data 
        *    stSectionCon - section attribute configure
        *    uiStartLoc   - data that exclude head overlap area the start location
        *    uiEndLoc     - data that exclude tail overlap area the end location
        *
        */

        ISTATUS getHeadandTailOverlapLocation(IN  list<vector<point3D_t>> &LaneData,
                                              IN sectionCon               &stSectionCon,
                                              OUT uint32                  &uiStartLoc,
                                              OUT uint32                  &uiEndLoc);

        /*
        *@FUNC
        *    Get the new report data
        *
        *@PARAMS
        *    vfileName     - new data file name
        *    ltReportData  - new report data
        *
        */

        ISTATUS getReportData(IN vector<string>                 &vfileName,
                              OUT list<list<vector<point3D_t>>> &ltReportData);

        /*
        *@FUNC
        *    Locate points locate on which two section connected area 
        *
        *@PARAMS
        *    point3D             - data point
        *    vSectionCentrePoint - section boundary center
        *    segClosestLoc       - section distance point closest
        *    segSecondClosetLoc  - section distance point second closest
        *
        */

        ISTATUS locateCandidateSectionBoundary(IN point3D_t                &point3D,
                                               IN vector<segAttributes_t> &vSectionCentrePoint,
                                               OUT segAttributes_t        &segClosestLoc,
                                               OUT segAttributes_t        &segSecondClosetLoc);

        /*
        *@FUNC
        *    Locate the  accurate section 
        *
        *@PARAMS
        *    point3DRefOne     - one the two reference point
        *    point3DRefTwo     - one the two reference point
        *    segCandidateOne   - data that exclude head overlap area the start location
        *    segCandidateTwo   - data that exclude tail overlap area the end location
        *    segClosestSection - accurate section that means start section of report data
        *
        */

        ISTATUS filterCandidateSectionBoundary(IN point3D_t         &point3DRefOne,
                                               IN point3D_t         &point3DRefTwo,
                                               IN segAttributes_t   &segCandidateOne,
                                               IN segAttributes_t   &segCandidateTwo,
                                               OUT segAttributes_t &segClosestSection);

        /*
        *@FUNC
        *    get the  contained sections that report new data 
        *
        *@PARAMS
        *    ltSectionDataScale  - report lane data 
        *    ltRawData           - section attribute configure
        *    ltEffectiveLaneData - data that not contain coordinate value of point is 0
        *                           which means that point has not been detected
        *    uiStartLoc          - data that exclude head overlap area the start location
        *    uiEndLoc            - data that exclude tail overlap area the end location
        *    ltMatchSections     - matched sections data
        *
        */

        ISTATUS getMatchSections(IN list<segAttributes_t>    &ltSectionDataScale,
                                 IN sectionCon               &stSectionConfig,
                                 IN list<vector<point3D_t>>  &ltRawData,
                                 IN list<vector<point3D_t>>  &ltEffectiveLaneData,
                                 IN uint32                    &uiStartLoc,
                                 IN uint32                    &uiEndLoc,
                                 OUT list<reportSectionData> &ltMatchSections);

        ISTATUS matchSections(IN list<segAttributes_t>    &ltSectionDataScale,
                              IN uint32                    uiStartSecionID,
                              IN uint32                    uiEndSecionID,
                              IN vector<segAttributes_t>  &ltSectionCentrePoint,
                              IN vector<point3D_t>        &vRawLeftLineData,
                              IN vector<point3D_t>        &vRawRightLineData,
                              OUT list<reportSectionData> &ltMatchSections);
    };

};

#endif
