/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  FeatureExtract.h
* @brief Header file of extracting road line painting information to use SVM or
*        another ML algorithm to do lane number estimation.
*
* Change Log:
*      Date                Who             What
*      2015/09/25       Chen Ming        Create
*******************************************************************************
*/

#ifndef LINETYPESVM_FEATUREEXTRACT_H_
#define LINETYPESVM_FEATUREEXTRACT_H_

#include "apiDataStruct.h"
#include "SecRptData.h"
#include "RoadVecGen2.h"
#include <string>

namespace ns_database
{

class CFeatureExtract: public CRoadVecGen2
{
public:
    CFeatureExtract(void);
    CFeatureExtract(string configFilePath);
    ~CFeatureExtract(void);

    CFeatureExtract(const CFeatureExtract&) {}
    CFeatureExtract& operator = (const CFeatureExtract& ) {}

    /*
    * @FUNC:
    *    Extract input line feature data and output the feature vector list.
    *
    * @PARAMS:
    *    rptData - reported new data lines.
    *    extData - extracted feature vector list.
    *
    */
    bool extractFeature(IN    list<list<vector<point3D_t>>> &rptData,
                        INOUT list<vector<float>>           &extData);

    /*
    * @FUNC:
    *    Set section configuration file path, which is used to calculate section
    *    rotation angle ,x limits and re-sample x full list.
    *
    * @PARAM:
    *    cfgPath - full path of section configuration file.
    */
    void setSegCfgPath(IN string &cfgPath);


protected:
    /*
    * @FUNC:
    *    Extract line feature according to following steps.
    *    Rotate input line according to section rotation angle.
    *    Re-sample input data based on section X full range list.
    *    Re-sample full section data to specified feature vector.
    *
    * @PARAMS:
    *    segId    - section Id of the input line.
    *    lineData - input line data to be extracted.
    *    extData  - extracted feature vector of single line.
    *
    */
    void extractLineFeature(IN uint32            &segId,
                            IN vector<point3D_t> &lineData,
                            INOUT vector<float>  &extData);

    /*
    *
    */
    void saveSegData(IN list<reportSectionData> &segData);
};

}


#endif

