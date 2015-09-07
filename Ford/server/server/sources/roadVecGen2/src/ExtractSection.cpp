/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  ExtractSection.cpp
* @brief This is class implementation file for extractSection, which extract 
*        sections from report new data according to section configure  
*
* Change Log:
*      Date                Who             What
*      2015/08/20       Zhong Ning         Create
*******************************************************************************
*/

#include "apiDataStruct.h"
#include "ExtractSection.h"

namespace ns_database
{
    /***************************************************************************************************
    IN list<segAttributes_t> &sectionConfig        each section configure :section ID section boundary points
    IN list<list<vector<point3D_t>>> &reportData   more than once report data
    OUT reportSectionData &rptSectionData          extract multi sections from report data 
    ***************************************************************************************************/
    ISTATUS CExtractSection::extractSections(IN list<segAttributes_t>         &ltSectionDataScale,
                                             IN  sectionCon                   &stsectionConfig,
                                             IN list<list<vector<point3D_t>>> &reportData,
                                             OUT list<reportSectionData>      &ltRptSectionData)
    {
        list<vector<point3D_t>>   ltNewReportData,
            ltNoChangeLaneData;
        list<list<vector<point3D_t>>>  ltSubReportData;           // store multiple no change lane data that split from once report data 

        list<vector<point3D_t>>  ltEffectiveLaneData,
            ltMainEffectiveLaneData;

        uint32    uiStartLoc = 0,
            uiEndLoc = 0;
        double    dbLength = 0;

        ISTATUS   iStatus = I_SUCCESS;

        list<list<vector<point3D_t>>>::iterator iter = reportData.begin();

        if (reportData.empty())
            return I_FAIL;
        else
        {
            for (; iter!=reportData.end();++iter)
            {
                ltNewReportData = *iter;

                iStatus = splitChangeLaneData(ltNewReportData,ltSubReportData);

                if (iStatus == I_SUCCESS)
                {
                    for (list<list<vector<point3D_t>>>::iterator iter = ltSubReportData.begin();iter != ltSubReportData.end(); ++iter)
                    {
                        ltNoChangeLaneData = *iter;
                        vector<point3D_t> vLeftLine = ltNoChangeLaneData.front();
                        if (vLeftLine.size() > MINPOINTNUM)  // avoid handle few points which means short length about report data
                        {
                            iStatus = getReportHeadandTailOverlapLocation(ltNoChangeLaneData,
                                                                           stsectionConfig,
                                                                           ltEffectiveLaneData,
                                                                           ltMainEffectiveLaneData,
                                                                           uiStartLoc,
                                                                           uiEndLoc,
                                                                           dbLength);
                            if (iStatus == I_SUCCESS)
                            {
                                iStatus = getMatchSections(ltSectionDataScale,
                                                           stsectionConfig,
                                                           ltNoChangeLaneData,
                                                           ltEffectiveLaneData,
                                                           uiStartLoc,
                                                           uiEndLoc,
                                                           ltRptSectionData);
                                ltEffectiveLaneData.clear();
                                ltMainEffectiveLaneData.clear();
                            }
                            else
                            {
                                ltEffectiveLaneData.clear();
                                ltMainEffectiveLaneData.clear();
                                continue;
                            }
                        }
                    }

                }
                else
                    return iStatus;
                ltSubReportData.clear();
            }
            return I_SUCCESS;
        }
    }
    /******************************************************************************

    *******************************************************************************/
        ISTATUS CExtractSection::getMatchSections(
            IN list<segAttributes_t>    &ltSectionDataScale,
            IN sectionCon               &stSectionConfig,
            IN list<vector<point3D_t>>  &ltRawData,
            IN list<vector<point3D_t>>  &ltEffectiveLaneData,
            IN uint32                    &uiStartLoc,
            IN uint32                    &uiEndLoc,
            OUT list<reportSectionData> &ltMatchSections)
    {
        vector<segAttributes_t> ltSectionCentrePoint;

        vector<point3D_t>     vEffectiveLaneCentreData,
            vEffectiveLeftLine,
            vEffectiveRightLine,
            vSectionLeftLine,
            vSectionRightLine;

        segAttributes_t       segSectionCentre;
        point3D_t point3D;

        segAttributes_t      segClosestLoc_start,
            segSecondClosestLoc_start,
            segClosestLoc_end,
            segSecondClosestLoc_end,
            segClosestSection_start,
            segClosestSection_end;

        memset(&segClosestLoc_start,0,sizeof(segClosestLoc_start));
        memset(&segSecondClosestLoc_start,0,sizeof(segSecondClosestLoc_start));
        memset(&segClosestLoc_end,0,sizeof(segClosestLoc_end));
        memset(&segSecondClosestLoc_end,0,sizeof(segSecondClosestLoc_end));
        memset(&segClosestSection_start,0,sizeof(segClosestSection_start));
        memset(&segClosestSection_end,0,sizeof(segClosestSection_end));

        double               dlMinLength_Front = 0,
                             dlMinLength_Back = 0;

        int         uiSectionFrontLoc = 0,
                    uiSectionBackLoc   = 0;

        ISTATUS iStatus = I_SUCCESS;
        reportSectionData    rptSectionData;
        list<vector<point3D_t>> ltOneLaneData;
        list<list<vector<point3D_t>>> ltMultiLaneData;
        vector<point3D_t> vRawLeftLineData = ltRawData.front();
        vector<point3D_t> vRawRightLineData = ltRawData.back();
        //1.get section boundary centre and  centre point between left line and right line of lane
        for (list<segAttributes_t>::iterator iter = ltSectionDataScale.begin(); iter!=ltSectionDataScale.end();++iter)
        {
            segSectionCentre.ports[0].lon = (*iter).ports[0].lon;
            segSectionCentre.ports[0].lat = (*iter).ports[0].lat;

            segSectionCentre.ports[1].lon = (*iter).ports[1].lon;
            segSectionCentre.ports[1].lat = (*iter).ports[1].lat;

            segSectionCentre.ports[2].lon = (*iter).ports[2].lon;
            segSectionCentre.ports[2].lat = (*iter).ports[2].lat;

            segSectionCentre.ports[3].lon = (*iter).ports[3].lon;
            segSectionCentre.ports[3].lat = (*iter).ports[3].lat;

            segSectionCentre.segId_used = (*iter).segId_used;
            segSectionCentre.segId = (*iter).segId;
            ltSectionCentrePoint.push_back(segSectionCentre);
        }


        vEffectiveLeftLine  = ltEffectiveLaneData.front();
        vEffectiveRightLine = ltEffectiveLaneData.back();

        if (vEffectiveLeftLine.size()!=vEffectiveRightLine.size())
        {
            return I_FAIL;
        } 
        else
        {
            for (int i = 0;i<vEffectiveLeftLine.size();i++)
            {
                point3D.lon = (vEffectiveLeftLine[i].lon+vEffectiveRightLine[i].lon)/2;
                point3D.lat = (vEffectiveLeftLine[i].lat+vEffectiveRightLine[i].lat)/2; 

                vEffectiveLaneCentreData.push_back(point3D);
            }
        }
        //2.locate matched start and end section of report data

        iStatus = locateCandidateSectionBoundary(vEffectiveLaneCentreData[uiStartLoc],
                                                ltSectionCentrePoint,
                                                segClosestLoc_start,
                                                segSecondClosestLoc_start);

        if (iStatus!=I_SUCCESS)
        {
            return iStatus;
        }
        iStatus = locateCandidateSectionBoundary(vEffectiveLaneCentreData[uiEndLoc],
                                                 ltSectionCentrePoint,
                                                 segClosestLoc_end,
                                                 segSecondClosestLoc_end);

        if (iStatus!=I_SUCCESS)
        {
            return iStatus;
        }

        if ((segClosestLoc_start.segId == segClosestLoc_end.segId)|| (segClosestLoc_start.segId == segSecondClosestLoc_end.segId)||
            (segSecondClosestLoc_start.segId == segClosestLoc_end.segId)||(segSecondClosestLoc_start.segId == segSecondClosestLoc_end.segId))
        {
            return I_FAIL;
        } 
        else
        {
            iStatus = filterCandidateSectionBoundary(vEffectiveLaneCentreData[uiStartLoc],
                                                     vEffectiveLaneCentreData[uiStartLoc+10], 
                                                     segClosestLoc_start,
                                                     segSecondClosestLoc_start,
                                                     segClosestSection_start);

            iStatus = filterCandidateSectionBoundary(vEffectiveLaneCentreData[uiEndLoc],
                                                     vEffectiveLaneCentreData[uiEndLoc-10],
                                                     segClosestLoc_end,
                                                     segSecondClosestLoc_end,
                                                     segClosestSection_end);


            //3 .extract matched section from report data
            if (ROADCIRCLE == 0)
            {
                
                uint32 uiStartId = segClosestSection_start.segId<segClosestSection_end.segId?segClosestSection_start.segId:segClosestSection_end.segId;
                uint32 uiEndId   = segClosestSection_start.segId>segClosestSection_end.segId?segClosestSection_start.segId:segClosestSection_end.segId;
                iStatus = matchSections(ltSectionDataScale,uiStartId,uiEndId,ltSectionCentrePoint,vRawLeftLineData,vRawRightLineData,ltMatchSections);
                if (iStatus!= I_SUCCESS)
                    return iStatus;
            }
            else
            {
                if (segClosestSection_start.segId <= segSecondClosestLoc_end.segId-1)
                {
                iStatus = matchSections(ltSectionDataScale,
                                       segClosestSection_start.segId,
                                       segClosestSection_end.segId,
                                       ltSectionCentrePoint,
                                       vRawLeftLineData,
                                       vRawRightLineData,
                                       ltMatchSections);
                    if (iStatus!= I_SUCCESS)
                        return iStatus;
                } 
                else
                {
                iStatus = matchSections(ltSectionDataScale,
                                        segClosestSection_start.segId,
                                        CONJOINTSECID,
                                        ltSectionCentrePoint,
                                        vRawLeftLineData,
                                        vRawRightLineData,
                                        ltMatchSections);
                    if (iStatus!= I_SUCCESS)
                        return iStatus;
                iStatus = matchSections(ltSectionDataScale,
                                        STARTSECID,
                                        segClosestSection_end.segId,
                                        ltSectionCentrePoint,
                                        vRawLeftLineData,
                                        vRawRightLineData,
                                        ltMatchSections);
                    if (iStatus!= I_SUCCESS)
                        return iStatus;

                }

            }
           
        }
        return iStatus;
    }


    /****filter the correct section from two candinate sections *****/
    ISTATUS CExtractSection::filterCandidateSectionBoundary(
        IN point3D_t         &point3DRefOne,
        IN point3D_t         &point3DRefTwo,
        IN segAttributes_t   &segCandidateOne,
        IN segAttributes_t   &segCandidateTwo,
        OUT segAttributes_t &segClosestSection)
    {
        double dlDisA = ((point3DRefOne.lon - segCandidateOne.ports[0].lon)* (point3DRefOne.lon - segCandidateOne.ports[0].lon)+
            (point3DRefOne.lat - segCandidateOne.ports[0].lat)* (point3DRefOne.lat - segCandidateOne.ports[0].lat));

        double dlDisB = ((point3DRefOne.lon - segCandidateTwo.ports[0].lon)* (point3DRefOne.lon - segCandidateTwo.ports[0].lon)+
            (point3DRefOne.lat - segCandidateTwo.ports[0].lat)* (point3DRefOne.lat - segCandidateTwo.ports[0].lat));

        double dlDisC = ((point3DRefTwo.lon - segCandidateOne.ports[0].lon)* (point3DRefTwo.lon - segCandidateOne.ports[0].lon)+
            (point3DRefTwo.lat - segCandidateOne.ports[0].lat)* (point3DRefTwo.lat - segCandidateOne.ports[0].lat));

        double dlDisD = ((point3DRefTwo.lon - segCandidateTwo.ports[0].lon)* (point3DRefTwo.lon - segCandidateTwo.ports[0].lon)+
            (point3DRefTwo.lat - segCandidateTwo.ports[0].lat)* (point3DRefTwo.lat - segCandidateTwo.ports[0].lat));

        if ((dlDisA < dlDisC)&&(dlDisB > dlDisD))
        {
            segClosestSection = segCandidateTwo;
        }
        else
        {
            segClosestSection = segCandidateOne;
        }

        return I_SUCCESS;
    }


    /*********locate the closest and second closest section boundary that distance from given point***********/
    ISTATUS CExtractSection::locateCandidateSectionBoundary(
        IN point3D_t               &point3D,
        IN vector<segAttributes_t> &vSectionCentrePoint,
        OUT segAttributes_t        &segFirstCandidate,
        OUT segAttributes_t        &segSecondCandidate)
    {
        double        dlLength  = 0,
                      dlLength_two = 0,
                      dlLength_three = 0,
                      dlShorestLength = 0,
                      dlShorerLength  = 0;
        ISTATUS       iStatus = I_SUCCESS;
        int           uiClosestLoc=0,
                      uiSecondClosetLoc = 1;
        // step 1 get the closest boundary
        if (vSectionCentrePoint.empty())
            return I_FAIL;
        dlShorestLength = sqrt((point3D.lon - vSectionCentrePoint[0].ports[0].lon)*(point3D.lon - vSectionCentrePoint[0].ports[0].lon)+
            (point3D.lat - vSectionCentrePoint[0].ports[0].lat)*(point3D.lat - vSectionCentrePoint[0].ports[0].lat));
        dlShorerLength = dlShorestLength;
        for (int i = 1; i<vSectionCentrePoint.size(); i++)
        {
            dlLength = sqrt((point3D.lon - vSectionCentrePoint[i].ports[0].lon)*(point3D.lon - vSectionCentrePoint[i].ports[0].lon)+
                (point3D.lat - vSectionCentrePoint[i].ports[0].lat)*(point3D.lat - vSectionCentrePoint[i].ports[0].lat));

            if (dlLength < dlShorestLength)
            {
                dlShorerLength = dlShorestLength;
                dlShorestLength = dlLength;
                uiClosestLoc = i;
            }
        }
        // step 2 get the second closest boundary     
        if (uiClosestLoc>0 && uiClosestLoc< vSectionCentrePoint.size()-1)
        {

            dlLength_two = sqrt((point3D.lon - vSectionCentrePoint[uiClosestLoc -1].ports[0].lon)*(point3D.lon - vSectionCentrePoint[uiClosestLoc -1].ports[0].lon)+
                (point3D.lat - vSectionCentrePoint[uiClosestLoc -1].ports[0].lat)*(point3D.lat - vSectionCentrePoint[uiClosestLoc -1].ports[0].lat));

            dlLength_three = sqrt((point3D.lon - vSectionCentrePoint[uiClosestLoc + 1].ports[0].lon)*(point3D.lon - vSectionCentrePoint[uiClosestLoc +1].ports[0].lon)+
                (point3D.lat - vSectionCentrePoint[uiClosestLoc +1].ports[0].lat)*(point3D.lat - vSectionCentrePoint[uiClosestLoc +1].ports[0].lat));

            if (dlLength_two <= dlLength_three)
                uiSecondClosetLoc = uiClosestLoc - 1;
            else
                uiSecondClosetLoc = uiClosestLoc + 1;
        } else if (uiClosestLoc == 0)
            uiSecondClosetLoc = uiClosestLoc + 1;
        else 
            uiSecondClosetLoc = uiClosestLoc - 1;


        segFirstCandidate = vSectionCentrePoint[uiClosestLoc];
        segSecondCandidate = vSectionCentrePoint[uiSecondClosetLoc];
        return I_SUCCESS;
    }
    /*****encounter change lane need to split new report data into multi no change lane data
    OUT list<list<vector<point3D_t>>>   store multi segment that split from  list<vector<point3D_t>>  ****/
    ISTATUS CExtractSection::splitChangeLaneData(
        IN list<vector<point3D_t>>        &newReportData,
        OUT list<list<vector<point3D_t>>> &ltSubReportData)
    {
        vector<point3D_t> vData_L = newReportData.front();
        vector<point3D_t> vData_R = newReportData.back();
        list<vector<point3D_t>> ltSubSegmentData;
        vector<point3D_t> vSubData_L,
            vSubData_R;

        if (newReportData.empty())
            return I_FAIL;

        for (int i = 1; i < vData_L.size(); i ++)  //first point not been considered
        {
            if (vData_L[i].paintFlag != 2)
            {
                vSubData_L.push_back(vData_L[i]);
                vSubData_R.push_back(vData_R[i]);
            }else if (vData_L[i].paintFlag == 2 && vData_L[i-1].paintFlag != 2)
            {
                ltSubSegmentData.push_back(vSubData_L);
                ltSubSegmentData.push_back(vSubData_R);
                ltSubReportData.push_back(ltSubSegmentData);
                vSubData_L.clear();
                vSubData_R.clear();
                ltSubSegmentData.clear();
            }
        }

        if (!vSubData_L.empty()&&!vSubData_R.empty())
        {
            ltSubSegmentData.push_back(vSubData_L);
            ltSubSegmentData.push_back(vSubData_R);
            ltSubReportData.push_back(ltSubSegmentData);
            vSubData_L.clear();
            vSubData_R.clear();
            ltSubSegmentData.clear();
        }
        return I_SUCCESS;
    }

    /*get the head and tail overlap start end location*/
    ISTATUS CExtractSection::getReportHeadandTailOverlapLocation(
        IN list<vector<point3D_t>>  &ltLaneData,
        IN sectionCon               &stSectionCon,
        OUT list<vector<point3D_t>> &ltEffectiveLaneData,
        OUT list<vector<point3D_t>> &ltMainEffectiveLaneData,
        OUT uint32                  &uiStartLoc,
        OUT uint32                  &uiEndLoc,
        OUT double                  &dbLength)
    {
        //step 1.extract effective point which flag not -1
        vector<point3D_t> vRawLineData_L,
            vRawLineData_R,
            vEffectiveLineData_L,
            vEffectiveLineData_R,
            vMainEffectiveLineData_L,
            vMainEffectiveLineData_R;
        list<vector<point3D_t>>::iterator iter = ltLaneData.begin();
        int k =0;
        ISTATUS iStatus = I_SUCCESS;
        if (ltLaneData.empty())
            return I_FAIL;
        while (iter != ltLaneData.end())
        {

            vRawLineData_L = *iter;     ++ iter;

            vRawLineData_R = *iter;     ++ iter;

            if (vRawLineData_R.size() != vRawLineData_L.size())
            {
                return I_FAIL;
            }


            for (int i = 0;i < vRawLineData_L.size(); i++)
            {
                if (vRawLineData_L[i].lat != 0.0 && vRawLineData_L[i].lon != 0.0&&
                    vRawLineData_R[i].lat != 0.0 && vRawLineData_R[i].lon != 0.0)
                {
                    vEffectiveLineData_L.push_back(vRawLineData_L[i]);
                    vEffectiveLineData_R.push_back(vRawLineData_R[i]);
                }      
            }
            ltEffectiveLaneData.push_back(vEffectiveLineData_L);
            ltEffectiveLaneData.push_back(vEffectiveLineData_R);

            vRawLineData_L.clear();
            vRawLineData_R.clear();

            //step 2.calculate lane length and get effective lane data head and tail overlap 
            dbLength=calcLaneLength(ltEffectiveLaneData);

            if (dbLength <= stSectionCon.dbMinLength + 2*stSectionCon.dbOverlap)  // length of report data must more than section minimum Length plus double overlap length
            { 
                ltEffectiveLaneData.clear();
                continue;
            }
            else
            {
                iStatus = getHeadandTailOverlapLocation(ltEffectiveLaneData,stSectionCon,uiStartLoc,uiEndLoc);

                for (int i = uiStartLoc; i <= uiEndLoc; i++)
                {
                    vMainEffectiveLineData_L.push_back(vEffectiveLineData_L[i]);
                    vMainEffectiveLineData_R.push_back(vEffectiveLineData_R[i]);
                }
                ltMainEffectiveLaneData.push_back(vMainEffectiveLineData_L);
                ltMainEffectiveLaneData.push_back(vMainEffectiveLineData_R);
                vEffectiveLineData_L.clear();
                vEffectiveLineData_R.clear();
                return I_SUCCESS;
            }
        }

        return I_FAIL;
    }

    /*calculate lane length ltLaneData should u not contain change lane data*/
    double CExtractSection::calcLaneLength(IN list<vector<point3D_t>> &ltLaneData)
    {
        double  dbLength = 0;
        vector<point3D_t> vLine = ltLaneData.front(); // get one line of lane to calculate lane length
        for (int i = 1; i < vLine.size(); i++)
        {
            dbLength = dbLength + sqrt((vLine[i].lon - vLine[i-1].lon)*(vLine[i].lon - vLine[i-1].lon)+
                (vLine[i].lat - vLine[i-1].lat)*(vLine[i].lat - vLine[i-1].lat));
        }
        return dbLength;
    }


    /**********loacte the position of head and end  overlap of report new data*************/
    ISTATUS CExtractSection::getHeadandTailOverlapLocation(
        IN  list<vector<point3D_t>> &LaneData,
        IN  sectionCon              &stSectionCon,
        OUT uint32                  &uiStartLoc,
        OUT uint32                  &uiEndLoc)
    {
        //step 1.just operate Left Line to calculate location
        vector<point3D_t> vLeftLine;
        double dbLength = 0;
        vLeftLine = LaneData.front();
        for (int i = 1;i<vLeftLine.size();i++)
        {
            dbLength = dbLength + sqrt((vLeftLine[i].lon - vLeftLine[i-1].lon)*(vLeftLine[i].lon - vLeftLine[i-1].lon)+
                (vLeftLine[i].lat - vLeftLine[i-1].lat)*(vLeftLine[i].lat - vLeftLine[i-1].lat));
            if (dbLength > stSectionCon.dbOverlap)
            {
                uiStartLoc = i;
                break;
            }
        }
        dbLength = 0;
        for (int i = vLeftLine.size()-1;i>2;i--)
        {
            dbLength = dbLength + sqrt((vLeftLine[i].lon - vLeftLine[i-1].lon)*(vLeftLine[i].lon - vLeftLine[i-1].lon)+
                (vLeftLine[i].lat - vLeftLine[i-1].lat)*(vLeftLine[i].lat - vLeftLine[i-1].lat));
            if (dbLength > stSectionCon.dbOverlap)
            {
                uiEndLoc = i;
                break;
            }
        }
        return I_SUCCESS;
    }

    ISTATUS CExtractSection::matchSections(
        IN  list<segAttributes_t>   &ltSectionDataScale,
        IN  uint32                   uiStartSecionID,
        IN  uint32                   uiEndSecionID,
        IN  vector<segAttributes_t> &ltSectionCentrePoint,
        IN  vector<point3D_t>       &vRawLeftLineData,
        IN  vector<point3D_t>       &vRawRightLineData,
        OUT list<reportSectionData> &ltMatchSections)
    {
        double dlMinLength_Front(0), dlMinLength_Back(0), dlTempLength(0);
        int uiSectionFrontLoc(0),uiSectionBackLoc(0);

        vector<point3D_t> vSectionLeftLine, vSectionRightLine;
        list<vector<point3D_t>> ltSectionData,ltOneLaneData;
        list<list<vector<point3D_t>>> ltLaneSectionData, ltMultiSectionData, ltMultiLaneData;
        list<list<list<vector<point3D_t>>>> ltSecionsData;

        reportSectionData rptSecionsData;
        

        for (uint32 i = uiStartSecionID;i<uiEndSecionID;i++) 
        {
            for (int j = 0; j< ltSectionCentrePoint.size(); j++)
            {
                if ( i == ltSectionCentrePoint[j].segId)  // match section to get section area 
                {
                    dlMinLength_Front = sqrt((ltSectionCentrePoint[j].ports[2].lon - vRawLeftLineData[0].lon)*(ltSectionCentrePoint[j].ports[2].lon - vRawLeftLineData[0].lon)+
                        (ltSectionCentrePoint[j].ports[2].lat - vRawLeftLineData[0].lat)*(ltSectionCentrePoint[j].ports[2].lat - vRawLeftLineData[0].lat));
                    uiSectionFrontLoc = 0;
                    dlMinLength_Back = sqrt((ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[0].lon)*(ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[0].lon)+
                        (ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[0].lat)*(ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[0].lat));
                    uiSectionBackLoc = 0;

                    for (int k = 1;k < vRawLeftLineData.size(); k++) // traverse left line to search the closest point
                    {
                        dlTempLength = sqrt((ltSectionCentrePoint[j].ports[2].lon - vRawLeftLineData[k].lon)*(ltSectionCentrePoint[j].ports[2].lon - vRawLeftLineData[k].lon)+
                            (ltSectionCentrePoint[j].ports[2].lat - vRawLeftLineData[k].lat)*(ltSectionCentrePoint[j].ports[2].lat - vRawLeftLineData[k].lat));

                        if (dlTempLength < dlMinLength_Front )
                        {
                            dlMinLength_Front = dlTempLength;
                            uiSectionFrontLoc = k;
                        }
                        dlTempLength = sqrt((ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[k].lon)*(ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[k].lon)+
                            (ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[k].lat)*(ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[k].lat));

                        if (dlTempLength < dlMinLength_Back)
                        {
                            dlMinLength_Back = dlTempLength;
                            uiSectionBackLoc = k;
                        }

                    }

                    // step 1.get one section data
                    for (unsigned int m = uiSectionFrontLoc;m<=uiSectionBackLoc;m++)
                    {
                        vSectionLeftLine.push_back(vRawLeftLineData[m]);
                        vSectionRightLine.push_back(vRawRightLineData[m]);
                    }

                    ltSectionData.push_back(vSectionLeftLine);
                    ltSectionData.push_back(vSectionRightLine);

                    ltLaneSectionData.push_back(ltSectionData);

                    vSectionLeftLine.clear();
                    vSectionRightLine.clear();
                    ltSectionData.clear();


                    // step 2.store section into whole data
                    list<reportSectionData>::iterator iter = ltMatchSections.begin();
                    for(;iter!=ltMatchSections.end(); ++iter)
                    {
                        if (i==(*iter).sectionId)
                        {
                            (*iter).rptSecData.push_back(ltLaneSectionData);
                            ltLaneSectionData.clear();
                            break;
                        }                 
                    }
                    if (!ltLaneSectionData.empty())
                    {
                        rptSecionsData.sectionId = i;
                        rptSecionsData.rptSecData.push_back(ltLaneSectionData);
                        ltMatchSections.push_back(rptSecionsData);
                        ltLaneSectionData.clear();
                        rptSecionsData.rptSecData.clear();
                    }
                    break;
                }
            }
        }
        return I_SUCCESS;
    }

}