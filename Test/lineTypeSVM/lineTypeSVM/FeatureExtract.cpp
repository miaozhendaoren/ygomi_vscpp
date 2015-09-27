/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  FeatureExtract.h
* @brief Implementation file of extracting road line painting information to use
*        SVM or another ML algorithm to do lane number estimation.
*
* Change Log:
*      Date                Who             What
*      2015/09/25       Chen Ming        Create
*******************************************************************************
*/


#include "FeatureExtract.h"
#include "VisualizationApis.h"

namespace ns_database
{
// number of feature vector elements
const int FEATURE_VEC_ITEMS = 100;

static int section_num = 0;

const int SAVE_DATA = 0;

CFeatureExtract::CFeatureExtract(void)
{

}


CFeatureExtract::CFeatureExtract(string configFilePath) : CRoadVecGen2(configFilePath)
{

}


CFeatureExtract::~CFeatureExtract(void)
{

}


bool CFeatureExtract::extractFeature(IN list<list<vector<point3D_t>>> &rptData,
    INOUT list<vector<float>> &extData)
{
    if (rptData.empty() || _segConfigList.empty())
    {
        return false;
    }

    uint32 sampleInterval = 20;
    list<reportSectionData> segData;
    _secRptDataObj.segMultiRptData(rptData, sampleInterval, _segConfigList, segData);

#if SAVE_DATA
    // save section data to image
    saveSegData(segData);
#endif

    int numOfRptData = segData.size();
    list<list<list<vector<point3D_t>>>>::iterator grpIt;
    list<list<vector<point3D_t>>>::iterator laneIt;
    vector<float> featureVec;

    list<reportSectionData>::iterator rptIt = segData.begin();
    while (rptIt != segData.end())
    {
        if (!rptIt->rptSecData.empty())
        {
            grpIt = rptIt->rptSecData.begin();
            while (grpIt != rptIt->rptSecData.end())
            {
                if (!grpIt->empty())
                {
                    laneIt = grpIt->begin();
                    while (laneIt != grpIt->end())
                    {
                        if (!laneIt->empty())
                        {
                            // left line
                            featureVec.clear();
                            extractLineFeature(rptIt->sectionId, laneIt->front(), featureVec);
                            if (!featureVec.empty())
                            {
                                extData.push_back(featureVec);
                            }

                            // right line
                            featureVec.clear();
                            extractLineFeature(rptIt->sectionId, laneIt->back(), featureVec);
                            if (!featureVec.empty())
                            {
                                extData.push_back(featureVec);
                            }
                        }

                        laneIt++;
                    }
                }

                grpIt++;
            }
        }

        rptIt++;
    }

    return true;
}


void CFeatureExtract::setSegCfgPath(IN string &cfgPath)
{
    list<segAttributes_t> segConfigList;
    setSectionConfigPath(cfgPath, segConfigList);
}


void CFeatureExtract::extractLineFeature(IN uint32 &segId,
    IN vector<point3D_t> &lineData,
    INOUT vector<float>    &extData)
{
    // initialize feature vector
    extData.clear();

    if (lineData.empty())
    {
        return;
    }

    // get left/right re-sample line data according to current section Id
    uint32 curItem = 0;
    vector<point3D_t> *leftSample = nullptr;
    list<vector<point3D_t>>::iterator splLItor = _secLeftData.begin();
    while (splLItor != _secLeftData.end())
    {
        if (curItem  == segId - 1)
        {
            leftSample  = &(*splLItor);
            break;
        }
        splLItor++; curItem++;
    }
    int numOfSplPnts = leftSample->size();

    // rotate the input two lines first
    vector<point3D_t> leftRotated, leftRotatedValid;
    vector<point3D_t> rightRotated, rightRotatedValid;

    // current section rotation angle
    double theta = _secRotAngle[segId - 1];
    lineRotation(lineData, -theta, leftRotated);

    // get paint information for re-sample lines
    getLinePaintInfo(leftRotated, *leftSample);

    list<vector<point3D_t>> lane;
    lane.push_back(*leftSample);

#if SAVE_DATA
    char filename[_MAX_PATH];
    sprintf_s(filename, _MAX_PATH - 1, "section_data_%d_%d.txt",
        segId, section_num++);
    saveListVec(lane, filename);
#endif

    // down sample to specified vector elements number
    int sp = static_cast<int>(floor((double)numOfSplPnts / FEATURE_VEC_ITEMS));
    int mp = numOfSplPnts % FEATURE_VEC_ITEMS;
    int pp = static_cast<int>(floor((double)(mp - 1) / 2));
    for (int i = pp; i < FEATURE_VEC_ITEMS * sp + pp; i += sp)
    {
        extData.push_back(leftSample->at(i).paintFlag);
    }
}


void CFeatureExtract::saveSegData(IN list<reportSectionData> &segData)
{
    if (segData.empty())
    {
        return;
    }

    int grpCnt = 0, laneCnt = 0;
    char filename[_MAX_PATH] = { '\0' };
    list<list<list<vector<point3D_t>>>>::iterator grpIt;
    list<list<vector<point3D_t>>>::iterator laneIt;

    list<reportSectionData>::iterator rptIt = segData.begin();
    while (rptIt != segData.end())
    {
        grpCnt = 0;
        laneCnt = 0;

        if (!rptIt->rptSecData.empty())
        {
            grpIt = rptIt->rptSecData.begin();
            while (grpIt != rptIt->rptSecData.end())
            {
                if (!grpIt->empty())
                {
                    laneIt = grpIt->begin();
                    while (laneIt != grpIt->end())
                    {
                        if (!laneIt->empty())
                        {
                            sprintf_s(filename, "section_%d_group_%d_lane_%d.png",
                                      rptIt->sectionId, grpCnt, laneCnt);
                            showImage(*laneIt, Scalar(0, 255, 0), filename);

                            laneCnt++;
                        }

                        laneIt++;
                    }

                    grpCnt++;
                }

                grpIt++;
            }
        }

        rptIt++;
    }
}


}


