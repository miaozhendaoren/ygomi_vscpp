/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  SecRptData2.h
* @brief This is class definition header file for SecRptData2, which extract 
*        sections from report data according to section configure
*
* Change Log:
*      Date                Who                    What
*      2015/09/11    Qian Xu,Shili Wang          Create
*******************************************************************************
*/

#pragma once

#include "apiDataStruct.h"
#include "RoadSeg.h"

namespace ns_database
{
	class CSecRptData2
	{
	public:
		CSecRptData2();
		virtual ~CSecRptData2();

		/*
        * @FUNC
		*     get section and section body points configuration
		*
		* @PARAMS
        *     secCfgInfo - section information.
        *     secPointInfo  - section body points information.
        *     bodyPts3D - all body points coordinates
		*
        */
		bool CSecRptData2::initCfg(IN vector<secCfgInfo_t> secCfgInfo,
			                       IN vector<secPointInfo_t> secPointInfo,
			                       IN vector<point3D_t> bodyPts3D);
		/*
        * @FUNC
		*     extract sections from report new data according to section configure file
		*
		* @PARAMS
        *     rptData - road lines of the lane vehicle reported.
        *     sampleInterval  - sample interval by point number.
        *     secData - extracted sections from new report data;for every section it has 
		*               sectinID and road lines of the lanes vehicle reported
		*
        */
		int CSecRptData2::segMultiRptData(IN  list<list<vector<point3D_t>>> rptData,
			                              IN  uint32 sampleInterval,
			                              OUT list<reportSectionData> &secData);

		double CSecRptData2::getLength( IN point3D_t point1, IN point3D_t point2 );

	protected:

		enum RESAMPLE_METHOD
		{
			USE_POINTNUM = 0,
			USE_METER = 1,
		};

		vector<secCfgInfo_t>           _secCfgInfo;      //section configure,ordered from segId 1
		vector<secPointInfo_t>         _secPointInfo;    //section body points configure
		vector<point3D_t>              _bodyPts3D;       //all body points coordinates,ordered from pointId 0; 
		vector<vector<uint32>>         _parSeg;          //for one section,get its other correlative sections
		double                         _sampleInterval;  //sample interval by point number,initialized by outside
		double                         _minSegLen;       //the shortest length of all sections
		RESAMPLE_METHOD                _smpltype;        //used sample method
		/*
        * @FUNC
		*     save all sections and points to text file if define SAVE_CFG_ON 1
		*
        */
		bool CSecRptData2::saveCfg();

		/*
        * @FUNC
		*     save the current input rptData to text file if define SAVE_CURR_RPT_DATA 1
		*
        */
		void CSecRptData2::saveData(IN  list<list<vector<point3D_t>>> rptData,
			                        IN  char* filename);

		/*
        * @FUNC
		*     load the rptData from text file if define READ_DATA_FROM_FILE 1
		*
        */
		void CSecRptData2::loadData(IN  char* filename,
			                        OUT list<list<vector<point3D_t>>> &rptData);

		/*
        * @FUNC
		*     find the extracted section Info of one rptData.
        *
        * @PARAMS
        *     singleRptData - left line data of the lane vehicle reported.
		*     singleMatchedDataInfo - all extracted section Info of one rptData.
		*                             every section has its segID,start index and end index in rptData.
		*
        */
		int getSingleRptDataInfo(IN  vector<point3D_t> singleRptData,
			                     OUT vector<sampleSectionBody_t> &singleMatchedDataInfo);

		/*
        * @FUNC
        *     resample reported data from vehicle.
        *
        * @PARAMS
        *     singleRptData - left line data of the lane vehicle reported.
        *     samplePoint  - resampled points of the current report data.
		*                    every point has segID(init to 0 in this func),
		*                    start index in the current rptData and its point coordinate.
        *
        */
		int resampleData(IN  vector<point3D_t> singleRptData,
			             OUT list<samplePoint_t> &samplePoint);

		/*
        * @FUNC
        *     find section ID for every sample point.
        *
        * @PARAMS
        *     samplePoint  - resampled points of the current report data.
		*                    every point gets its segID.
        *
        */
		void findSamplePtsSecID( INOUT list<samplePoint_t> &samplePoint );

		/*
        * @FUNC
        *     get every coarse section body Info of one rptData.
        *
        * @PARAMS
        *     samplePoint - resampled points of the current report data.
		*     sampleSectionBody - all coarse section body Info of rptData.
		*                         every coarse section body has its segID,
		*                         start index and end index in rptData.
		*
        */
		int getSecBodyInfo(IN  list<samplePoint_t> samplePoint,
			               IN  vector<point3D_t> singleRptData,
			               OUT vector<sampleSectionBody_t> &sampleSectionBody);

		/*
        * @FUNC
        *     get every coarse section overlap of rptData.
        *
        * @PARAMS
        *     sampleSectionBody - all coarse section body Info of rptData.
        *     singleRptData - left line data of the lane vehicle reported.
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
		*     sampleSectionBody - all coarse section body Info of rptData.
		*     singleRptData - left line data of the lane vehicle reported.
		*     singleMatchedDataInfo - all extracted section Info of one rptData.
		*                             every section has its segID,start index and end index in rptData.
		*
        */
		int rotAndCompare(IN  list<sampleSectionOverlap_t> sampleSectionOverlap,
			              IN  vector<sampleSectionBody_t> sampleSectionBody,
						  IN  vector<point3D_t> singleRptData,
			              OUT vector<sampleSectionBody_t> &singleMatchedDataInfo);	

		int mergeJumpSeg(vector<sampleSectionBody_t> &singleMatchedDataInfo, vector<sampleSectionBody_t> sampleSectionBody, vector<point3D_t> singleRptData);

	    double compareSegLen(IN vector<point3D_t> singleRptData, 
		                     IN sampleSectionBody_t oneSectionBody);

		bool mergeShortSeg(IN  vector<point3D_t> singleRptData,
			               INOUT  vector<sampleSectionBody_t> &sampleSectionBody,
					 	   INOUT int &mergeNO );

        void mergeSameSeg(INOUT int &mergeNum,
		                  INOUT vector<sampleSectionBody_t> &singleMatchedDataInfo );

        double getOverlapScale( IN point3D_t dataPt,
		                        IN uint32 segId );

        int mergeJumpSeg2( vector<sampleSectionBody_t> &sampleSectionBody, vector<point3D_t> singleRptData);


		/*
        * @FUNC
		*     extract section data of all rptData.
        *
        * @PARAMS
        *     SecDataInfo - the section data Info which has segID,start index and end index in the rptData 
		*     rptData - road lines of the lane vehicle reported.
		*     isValidData -  is the rptData lane has found a section.
        *     secConfigList  - section attribute configure.
		*     secData - extracted sections from new report data;for every section it has 
		*               sectinID and road lines of the lane vehicle reported.               
		*
        */
		int extractSecData(IN list<vector<sampleSectionBody_t>> SecDataInfo,
			               IN list<list<vector<point3D_t>>> rptData,
			               IN vector<int> isValidData,
			               OUT list<reportSectionData> &secData);

		//
		void CSecRptData2::debugPrintf(const string errorPrint);

		double CSecRptData2::getAngle(IN point3D_t samplePoint,
			                          IN point3D_t sectionPoint,
			                          IN point3D_t connPoint);

		int CSecRptData2::getSampleSectionID(IN samplePoint_t oneSamplePoint,
			                                 OUT uint32 &sampleSectionID);


		BOOL CSecRptData2::getSectionBoundry(IN list<samplePoint_t> samplePoint,
			                                 INOUT uint32 &seekStartRow,
			                                 OUT sampleSectionBody_t &aSampleBodySection,
			                                 OUT BOOL &selectOverFlag);

		int CSecRptData2::rotPoint(IN point3D_t sourcePoint,
			                       IN double theta,
			                       OUT point3D_t &rotedPoint);

	};/* class CSecRptData2 */

}/* namespace ns_database */



