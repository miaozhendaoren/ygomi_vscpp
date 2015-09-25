/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  SecRptData.h
* @brief This is class definition header file for SecRptData, which extract 
*        sections from report new data according to section configure
*
* Change Log:
*      Date                Who                    What
*      2015/09/11    Qian Xu,Shili Wang          Create
*******************************************************************************
*/

#pragma once

#include "apiDataStruct.h"

namespace ns_database
{
	class CSecRptData
	{
	public:
		CSecRptData(void);
		virtual ~CSecRptData(void);

		/*
        * @FUNC
		*     extract sections from report new data according to section configure file
		*
		* @PARAMS
        *     rptData - road lines of the lane vehicle reported.
        *     _segConfigList  - section attribute configure,including the sectionID 
		*                       and 4 section points Info.
        *     secData - extracted sections from new report data;for every section it has 
		*               sectinID and road lines of the lane vehicle reported
		*
        */
		int CSecRptData::segMultiRptData(IN  list<list<vector<point3D_t>>> rptData,
			                             IN  uint32 sampleInterval,
			                             IN  list<segAttributes_t> _segConfigList,
                                         OUT list<reportSectionData> &secData);
	protected:

		/*
        * @FUNC
		*     get the section config the class will use inner.
        *
        * @PARAMS
        *     _segConfigList - section attribute configure.
        *     secConfigList  - section attribute configure inner use.
		*                      transfer the body point and overlap point save order
		*
        */
		BOOL getSecConfigList(IN  list<segAttributes_t> _segConfigList,
			                  OUT list<segAttributes_t> &secConfigList);

		/*
        * @FUNC
		*     find the extracted section Info of one rptData.
        *
        * @PARAMS
        *     singleRptData - left road line of the lane vehicle reported.
        *     secConfigList  - section attribute configure.
		*     singleMatchedDataInfo - all extracted section Info of one rptData.
		*                             every section has its segID,start index and end index in rptData.
		*
        */
		int getSingleRptDataInfo(IN  vector<point3D_t> singleRptData,
			                     IN  uint32 sampleInterval,
			                     IN  list<segAttributes_t> secConfigList,
			                     OUT vector<sampleSectionBody_t> &singleMatchedDataInfo);

		/*
        * @FUNC
        *     resample reported data from vehicle.
        *
        * @PARAMS
        *     singleRptData - left road line of the lane vehicle reported.
        *     samplePoint  - resampled points of the current report data.
		*                    every point has segID(not getted yet this func),
		*                    start index in the current rptData and its point Info in left road line.
        *
        */
		int resampleData(IN  vector<point3D_t> singleRptData,
			             IN  uint32 sampleInterval,
			             OUT list<samplePoint_t> &samplePoint);

		/*
        * @FUNC
        *    find section ID for every sample point.
        *
        * @PARAMS
		*     secConfigList - section attribute configure.
        *     singleRptData - left road line of the lane,one vehicle reported.
        *     samplePoint  - resampled points of the current report data.
		*                    every point gets its segID.
        *
        */
		void findSamplePointSecID(IN  list<segAttributes_t> secConfigList,
			                     INOUT list<samplePoint_t> &samplePoint,
			                     OUT bool &closedLoopFlag);

		/*
        * @FUNC
        *     get every coarse section body Info of one rptData.
        *
        * @PARAMS
        *     samplePoint - resampled points of the current report data.
        *     closedLoopFlag  - is the sections in secConfigList a closed loop or not.
        *     configSecNum - total section number in secConfigList.
		*     sampleSectionBody - all coarse section body Info of rptData.
		*                         every coarse section body has its segID,
		*                         start index and end index in rptData.
		*
        */
		int getSecBodyInfo(IN  list<samplePoint_t> samplePoint,
			               IN  bool closedLoopFlag,
			               IN  int  configSecNum,
			               OUT vector<sampleSectionBody_t> &sampleSectionBody);

		/*
        * @FUNC
        *     get every coarse section overlap of rptData.
        *
        * @PARAMS
        *     sampleSectionBody - all coarse section body Info of rptData.
        *     singleRptData  - left road line of the lane,one vehicle reported.
		*     sampleSectionOverlap - all coarse section overlap of rptData.
		*                            every coarse section overlap has its segID,overlap data,
		*                            start index and end index in rptData.
		*
        */
		int getSecOverlap(IN  vector<sampleSectionBody_t> sampleSectionBody,
			              IN  vector<point3D_t> singleRptData,
			              OUT list<sampleSectionOverlap_t> &sampleSectionOverlap);

		/*
        * @FUNC
        *     rot the section in secConfigList and its corresponding coarse section overlap in rptData to horizontal.
		*     find the exact section overlap in rptData
        *
        * @PARAMS
        *     sampleSectionOverlap - all coarse section overlap of rptData.
        *     secConfigList  - section attribute configure.
		*     singleMatchedDataInfo - all extracted section Info of one rptData.
		*                             every section has its segID,start index and end index in rptData.
		*
        */
		int rotAndCompare(IN  list<sampleSectionOverlap_t> sampleSectionOverlap,
			              IN  list<segAttributes_t> secConfigList,
			              OUT vector<sampleSectionBody_t> &singleMatchedDataInfo);	

		/*
        * @FUNC
		*     extract section data of all rptData.
        *
        * @PARAMS
        *     SecDataInfo - the section data Info which has segID,start index and end index in the rptData 
		*     rptData - road lines of the lane vehicle reported.
		*     isValidData - is the rptData has a section data out.
        *     secConfigList  - section attribute configure.
		*     secData - extracted sections from new report data;for every section it has 
		*               sectinID and road lines of the lane vehicle reported.               
		*
        */
		int extractSecData(IN list<vector<sampleSectionBody_t>> SecDataInfo,
			               IN list<list<vector<point3D_t>>> rptData,
			               IN vector<int> isValidData,
			               IN list<segAttributes_t> secConfigList,
			               OUT list<reportSectionData> &secData);


	};/* class CSecRptData */

}/* namespace ns_database */

