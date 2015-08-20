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

/*******************************************************************************
return  0   success
       -1   fail
        1   warning
********************************************************************************/

namespace ns_database
{
    /***************************************************************************************************
    IN list<segAttributes_t> &sectionConfig        each section configure :section ID section boundary points
    IN list<list<vector<point3D_t>>> &reportData   more than once report data
    OUT reportSectionData &rptSectionData          extract multi sections from report data 
    ***************************************************************************************************/
    int CExtractSection::extractSections(IN list<segAttributes_t> &ltSectionDataScale,
        IN  sectionCon stsectionConfig,
        IN list<list<vector<point3D_t>>> &reportData,
        OUT list<reportSectionData> &ltRptSectionData)
    {
        list<vector<point3D_t>>   ltNewReportData,
            ltNoChangeLaneData;
        list<list<vector<point3D_t>>>  ltSubReportData;           // store multiple no change lane data that split from once report data 

        list<vector<point3D_t>>  ltEffectiveLaneData,
            ltMainEffectiveLaneData;

        uint32    uiStartLoc = 0,
            uiEndLoc = 0;
        double    dbLength = 0;

        list<list<vector<point3D_t>>>::iterator iter = reportData.begin();

        if (reportData.empty())
            return -1;
        else
        {
            for (; iter!=reportData.end();++iter)
            {
                ltNewReportData = *iter;

                int iStatus = splitChangeLaneData(ltNewReportData,ltSubReportData);

                if (iStatus == 0)
                {
                    for (list<list<vector<point3D_t>>>::iterator iter = ltSubReportData.begin();iter != ltSubReportData.end(); ++iter)
                    {
                        ltNoChangeLaneData = *iter;
                        vector<point3D_t> vLeftLine = ltNoChangeLaneData.front();
                        if (vLeftLine.size() > 500)  // avoid handle few points which means short length about report data
                        {
                            int iStatus = getReportHeadandTailOverlapLocation(ltNoChangeLaneData,stsectionConfig,ltEffectiveLaneData,ltMainEffectiveLaneData,uiStartLoc,uiEndLoc,dbLength);
                            if (iStatus == 0)
                            {
                                int iStatus = getMatchSections(ltSectionDataScale,ltNoChangeLaneData,ltEffectiveLaneData,uiStartLoc,uiEndLoc,ltRptSectionData);
                                return iStatus;
                            }
                            else
                                return iStatus;
                        }
                        else
                            return -1;
                    }

                }
                else
                    return iStatus;
            }
            return 0;
        }
    }
    /******************************************************************************

    *******************************************************************************/
    int CExtractSection::getMatchSections(IN list<segAttributes_t> &ltSectionDataScale,
        IN list<vector<point3D_t>> &ltRawData,
        IN list<vector<point3D_t>> &ltEffectiveLaneData,
        IN uint32 uiStartLoc,
        IN uint32 uiEndLoc,
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

        unsigned int         uiSectionFrontLoc = 0,
            uiSectionBackLoc   = 0;


        reportSectionData    rptSectionData;
        list<vector<point3D_t>> ltOneLaneData;
        list<list<vector<point3D_t>>> ltMultiLaneData;
        vector<point3D_t> vRawLeftLineData = ltRawData.front();
        vector<point3D_t> vRawRightLineData = ltRawData.back();
        //1.get section boundary centre and  centre point between left line and right line of lane
        for (list<segAttributes_t>::iterator iter = ltSectionDataScale.begin(); iter!=ltSectionDataScale.end();++iter)
        {
            segSectionCentre.ports[0].lon = ((*iter).ports[0].lon + (*iter).ports[1].lon)/2;  
            segSectionCentre.ports[0].lat = ((*iter).ports[0].lat + (*iter).ports[1].lat)/2;

            segSectionCentre.ports[1].lon = ((*iter).ports[2].lon + (*iter).ports[3].lon)/2;  
            segSectionCentre.ports[1].lat = ((*iter).ports[2].lat + (*iter).ports[3].lat)/2;

            segSectionCentre.ports[2].lon = ((*iter).ports[4].lon + (*iter).ports[5].lon)/2;  
            segSectionCentre.ports[2].lat = ((*iter).ports[4].lat + (*iter).ports[5].lat)/2;

            segSectionCentre.ports[3].lon = ((*iter).ports[6].lon + (*iter).ports[7].lon)/2;  
            segSectionCentre.ports[3].lat = ((*iter).ports[6].lat + (*iter).ports[7].lat)/2;

            segSectionCentre.segId_used = (*iter).segId_used;
            segSectionCentre.segId = (*iter).segId;
            ltSectionCentrePoint.push_back(segSectionCentre);
        }


        vEffectiveLeftLine  = ltEffectiveLaneData.front();
        vEffectiveRightLine = ltEffectiveLaneData.back();

        if (vEffectiveLeftLine.size()!=vEffectiveRightLine.size())
        {
            return -1;
        } 
        else
        {
            for (unsigned int i = 0;i<vEffectiveLeftLine.size();i++)
            {
                point3D.lon = (vEffectiveLeftLine[i].lon+vEffectiveRightLine[i].lon)/2;
                point3D.lat = (vEffectiveLeftLine[i].lat+vEffectiveRightLine[i].lat)/2; 

                vEffectiveLaneCentreData.push_back(point3D);
            }
        }
        //2.locate matched start and end section of report data

        int iStatus = locateCandidateSectionBoundary(vEffectiveLaneCentreData[uiStartLoc],ltSectionCentrePoint,segClosestLoc_start,segSecondClosestLoc_start);

        if (iStatus!=0)
        {
            return iStatus;
        }
        int iStauts = locateCandidateSectionBoundary(vEffectiveLaneCentreData[uiEndLoc],ltSectionCentrePoint,segClosestLoc_end,segSecondClosestLoc_end);

        if (iStatus!=0)
        {
            return iStatus;
        }

        if ((segClosestLoc_start.segId == segClosestLoc_end.segId)|| (segClosestLoc_start.segId == segSecondClosestLoc_end.segId)||
            (segSecondClosestLoc_start.segId == segClosestLoc_end.segId)||(segSecondClosestLoc_start.segId == segSecondClosestLoc_end.segId))
        {
            return -1;
        } 
        else
        {
            int iStatus = filterCandidateSectionBoundary(vEffectiveLaneCentreData[uiStartLoc],vEffectiveLaneCentreData[uiStartLoc+10], segClosestLoc_start,segSecondClosestLoc_start,segClosestSection_start);
            iStatus = filterCandidateSectionBoundary(vEffectiveLaneCentreData[uiEndLoc],vEffectiveLaneCentreData[uiEndLoc-10], segClosestLoc_end,segSecondClosestLoc_end,segClosestSection_end);


            //3 .extract matched section from report data
            if (segClosestSection_start.segId <= segSecondClosestLoc_end.segId-1)
            {
                for (uint32 i = segSecondClosestLoc_start.segId;i<segSecondClosestLoc_end.segId-1;i++)
                {
                    for (unsigned int j = 0; j< ltSectionCentrePoint.size(); j++)
                    {
                        if ( i == ltSectionCentrePoint[j].segId)
                        {
                            dlMinLength_Front = sqrt((ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[0].lon)*(ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[0].lon)+
                                (ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[0].lat)*(ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[0].lat));
                            uiSectionFrontLoc = 0;
                            dlMinLength_Back = sqrt((ltSectionCentrePoint[j].ports[4].lon - vRawLeftLineData[0].lon)*(ltSectionCentrePoint[j].ports[4].lon - vRawLeftLineData[0].lon)+
                                (ltSectionCentrePoint[j].ports[4].lat - vRawLeftLineData[0].lat)*(ltSectionCentrePoint[j].ports[4].lat - vRawLeftLineData[0].lat));
                            uiSectionBackLoc = 0;

                            for (unsigned int k = 1;k < vRawLeftLineData.size(); k++)
                            {
                                if (dlMinLength_Front < sqrt((ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[k].lon)*(ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[k].lon)+
                                    (ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[k].lat)*(ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[k].lat)))
                                {
                                    dlMinLength_Front = sqrt((ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[k].lon)*(ltSectionCentrePoint[j].ports[3].lon - vRawLeftLineData[k].lon)+
                                        (ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[k].lat)*(ltSectionCentrePoint[j].ports[3].lat - vRawLeftLineData[k].lat));

                                    uiSectionFrontLoc = k;
                                }

                                if (dlMinLength_Back < sqrt((ltSectionCentrePoint[j].ports[4].lon - vRawLeftLineData[k].lon)*(ltSectionCentrePoint[j].ports[4].lon - vRawLeftLineData[k].lon)+
                                    (ltSectionCentrePoint[j].ports[4].lat - vRawLeftLineData[k].lat)*(ltSectionCentrePoint[j].ports[4].lat - vRawLeftLineData[k].lat)))
                                {
                                    dlMinLength_Front = sqrt((ltSectionCentrePoint[j].ports[4].lon - vRawLeftLineData[k].lon)*(ltSectionCentrePoint[j].ports[4].lon - vRawLeftLineData[k].lon)+
                                        (ltSectionCentrePoint[j].ports[4].lat - vRawLeftLineData[k].lat)*(ltSectionCentrePoint[j].ports[4].lat - vRawLeftLineData[k].lat));

                                    uiSectionBackLoc = k;
                                }

                            }


                        }

                    }
                    //4.store data into section 
                    for (unsigned int m = uiSectionFrontLoc;m<=uiSectionBackLoc;m++)
                    {
                        vSectionLeftLine.push_back(vRawLeftLineData[m]);
                        vSectionRightLine.push_back(vRawRightLineData[m]);
                    }

                    ltOneLaneData.push_back(vSectionLeftLine);
                    ltOneLaneData.push_back(vSectionRightLine);

                    ltMultiLaneData.push_back(ltOneLaneData);
                    rptSectionData.sectionId = i;
                    rptSectionData.rptSecData.push_back(ltMultiLaneData); 

                    ltMatchSections.push_back(rptSectionData);

                    //5.clear temp variable
                    vSectionLeftLine.clear();
                    vSectionRightLine.clear();
                    ltOneLaneData.clear();
                    ltMultiLaneData.clear();
                    rptSectionData.rptSecData.clear();
                }
                return 0;
            } 
            else
            {
                return 1;
            }
        }

    }


    /****filter the correct section from two candinate sections *****/
    int CExtractSection::filterCandidateSectionBoundary(IN point3D_t point3DRefOne,
        IN point3D_t point3DRefTwo,
        IN segAttributes_t segCandidateOne,
        IN segAttributes_t segCandidateTwo,
        OUT segAttributes_t segClosestSection)
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
            return 1;
        }
        else
        {
            segClosestSection = segCandidateOne;
            return 1; 
        }
    }


    /*********locate the closest and second closest section boundary that distance from given point***********/
    int CExtractSection::locateCandidateSectionBoundary(IN point3D_t point3D,
        IN vector<segAttributes_t> &vSectionCentrePoint,
        OUT segAttributes_t segFirstCandidate,
        OUT segAttributes_t segSecondCandidate)
    {
        double  dlLength  = MinLength,
            dlMinLength = MaxLength;

        unsigned int   uiClosestLoc=0,
            uiSecondClosetLoc = 0;
        if (vSectionCentrePoint.empty())
            return -1;
        for (unsigned int i = 0; i<vSectionCentrePoint.size(); i++)
        {
            dlLength = sqrt((point3D.lon - vSectionCentrePoint[i].ports[0].lon)*(point3D.lon - vSectionCentrePoint[i].ports[0].lon)+
                (point3D.lat - vSectionCentrePoint[i].ports[0].lat)*(point3D.lat - vSectionCentrePoint[i].ports[0].lat));

            if (dlLength < dlMinLength)
            {
                uiSecondClosetLoc = uiClosestLoc;
                dlMinLength = dlLength;
                uiClosestLoc = i;
            }
        }
        segFirstCandidate = vSectionCentrePoint[uiClosestLoc];
        segSecondCandidate = vSectionCentrePoint[uiSecondClosetLoc];
        return 0;
    }
    /*****encounter change lane need to split new report data into multi no change lane data
    OUT list<list<vector<point3D_t>>>   store multi segment that split from  list<vector<point3D_t>>  ****/
    int CExtractSection::splitChangeLaneData(IN list<vector<point3D_t>> &newReportData,
        OUT list<list<vector<point3D_t>>> &ltSubReportData)
    {
        vector<point3D_t> vData_L = newReportData.front();
        vector<point3D_t> vData_R = newReportData.back();
        list<vector<point3D_t>> ltSubSegmentData;
        vector<point3D_t> vSubData_L,
            vSubData_R;

        if (newReportData.empty())
            return -1;

        for (unsigned int i = 1; i < vData_L.size(); i ++)  //first point not been considered
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
        if (vSubData_L.empty()||vData_R.empty())
            return -1;

        if (ltSubReportData.empty()) // change lane situation have not encounter
        {
            ltSubSegmentData.push_back(vSubData_L);
            ltSubSegmentData.push_back(vSubData_R);
            ltSubReportData.push_back(ltSubSegmentData);
            vSubData_L.clear();
            vSubData_R.clear();
            ltSubSegmentData.clear();
        }
        return 0;
    }

    /*get the head and tail overlap start end location*/
    int CExtractSection::getReportHeadandTailOverlapLocation(IN list<vector<point3D_t>>  &ltLaneData,
        IN sectionCon stSectionCon,
        OUT list<vector<point3D_t>> &ltEffectiveLaneData,
        OUT list<vector<point3D_t>> &ltMainEffectiveLaneData,
        OUT uint32 uiStartLoc,
        OUT uint32 uiEndLoc,
        OUT double dbLength)
    {
        //1.extract effective point which flag not -1
        vector<point3D_t> vRawLineData_L,
            vRawLineData_R,
            vEffectiveLineData_L,
            vEffectiveLineData_R,
            vMainEffectiveLineData_L,
            vMainEffectiveLineData_R;
        list<vector<point3D_t>>::iterator iter = ltLaneData.begin();
        unsigned int k =0;
        if (ltLaneData.empty())
            return -1;
        while (iter != ltLaneData.end())
        {

            vRawLineData_L = *iter;     ++ iter;

            vRawLineData_R = *iter;     ++ iter;

            if (vRawLineData_R.size() != vRawLineData_L.size())
            {
                return 1;
            }


            for (unsigned int i = 0;i < vRawLineData_L.size(); i++)
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
            vEffectiveLineData_L.clear();
            vEffectiveLineData_R.clear();
        }

        //2.calculate lane length and get effective lane data head and tail overlap 
        dbLength=calcLaneLength(ltEffectiveLaneData);

        if (dbLength <= stSectionCon.dbMinLength + 2*stSectionCon.dbOverlap)  // length of report data must more than section minimum Length plus double overlap length
            return 1;
        else
        {
            int iStatus = getHeadandTailOverlapLocation(ltEffectiveLaneData,stSectionCon,uiStartLoc,uiEndLoc);

            for (unsigned int i = uiStartLoc; i <= uiEndLoc; i++)
            {
                vMainEffectiveLineData_L.push_back(vEffectiveLineData_L[i]);
                vMainEffectiveLineData_R.push_back(vEffectiveLineData_R[i]);
            }
            ltMainEffectiveLaneData.push_back(vMainEffectiveLineData_L);
            ltMainEffectiveLaneData.push_back(vMainEffectiveLineData_R);

            return iStatus;
        }
    }

    /*calculate lane length ltLaneData should u not contain change lane data*/
    double CExtractSection::calcLaneLength(IN list<vector<point3D_t>> &ltLaneData)
    {
        double  dbLength = 0;
        vector<point3D_t> vLine = ltLaneData.front(); // get one line of lane to calculate lane length
        for (unsigned int i = 1; i < vLine.size(); i++)
        {
            dbLength = dbLength + sqrt((vLine[i].lon - vLine[i-1].lon)*(vLine[i].lon - vLine[i-1].lon)+
                (vLine[i].lat - vLine[i-1].lat)*(vLine[i].lat - vLine[i-1].lat));
        }
        return dbLength;
    }


    /**********loacte the position of head and end  overlap of report new data*************/
    int CExtractSection::getHeadandTailOverlapLocation(IN  list<vector<point3D_t>> &LaneData,
        IN sectionCon stSectionCon,
        OUT uint32 &uiStartLoc, 
        OUT uint32 &uiEndLoc)
    {
        //1.just operate Left Line to calculate location
        vector<point3D_t> vLeftLine;
        double dbLength = 0;
        vLeftLine = LaneData.front();
        for (unsigned int i = 1;i<vLeftLine.size();i++)
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
        for (unsigned int i = vLeftLine.size()-1;i>2;i--)
        {
            dbLength = dbLength + sqrt((vLeftLine[i].lon - vLeftLine[i-1].lon)*(vLeftLine[i].lon - vLeftLine[i-1].lon)+
                (vLeftLine[i].lat - vLeftLine[i-1].lat)*(vLeftLine[i].lat - vLeftLine[i-1].lat));
            if (dbLength > stSectionCon.dbOverlap)
            {
                uiEndLoc = i;
                break;
            }
        }
        return 0;
    }

}