/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  SecRptData2.cpp
* @brief This is class implementation file for SecRptData2, which extract 
*        sections from report data according to section configure  
*
* Change Log:
*      Date                Who                   What
*      2015/09/11     Qian Xu,Shili Wang        Create
*******************************************************************************
*/

#include <algorithm>
#include <iterator>
#include "SecRptData2.h"
#include "VisualizationApis.h"
#include "configure.h"
#include "RoadSeg.h"
#include "AppInitCommon.h"

using namespace std;

#define DEBUG_PRINT_ON  0
#define SAVE_CFG_ON  0
#define SAVE_CURR_RPT_DATA 0
#define READ_DATA_FROM_FILE 0

namespace ns_database
{
	#define MAX_SEC_LENGH               1000
	#define T_ROAD_OVERLAP_SCALE        5
	#define MAX_POINT_TO_SEC_DIST       500
	#define MAX_MEAN_SEC_DIST_Y         500
    #define MAX_JUMP_SEG_NO                   100
    #define MAX_INTER_ANGLE_IN_DEGREE         60
    #define THRESH_LENRATIO_MUL_ANGLEDIFF     0.2
	#define THRESH_MATCH_SEG_TO_CFG_SEG_LENGH_RATIO  0.5
	#define LAST_SEG_TO_CFG_SEG_LENGH_RATIO         1.0

	CSecRptData2::CSecRptData2(void)
	{

	}

	bool CSecRptData2::initCfg(IN vector<secCfgInfo_t> secCfgInfo,
		                       IN vector<secPointInfo_t> secPointInfo,
							   IN vector<point3D_t> bodyPts3D)
	{
		_secCfgInfo = secCfgInfo;
		if ( _secCfgInfo.empty())
		{
			debugPrintf("Initialize section configure failed!");
			return false;
		}
		_secPointInfo = secPointInfo;
		if ( _secPointInfo.empty() )
		{
			debugPrintf("Initialize section points configure failed!");
			return false;
		}
		_bodyPts3D = bodyPts3D;
		if ( _bodyPts3D.empty() || _bodyPts3D.size()!= _secPointInfo.size())
		{
			debugPrintf("Initialize all body points coordinate configure failed!");
			return false;
		}

#if SAVE_CFG_ON
		saveCfg();
#endif

		//compute _parseg for all sections
		uint32 segId;
		uint32 segType;
		vector<uint32> flyCrossSecId;
		vector<uint32> parSeg;

		for (int i = 0;i<_secCfgInfo.size();i++)
		{

			segId = _secCfgInfo[i].secId;
			segType = _secCfgInfo[i].secType;

			if (!parSeg.empty())
			{
				parSeg.clear();
			}
			if (!flyCrossSecId.empty())
			{
				flyCrossSecId.clear();
			}
#if 0
			switch(segType)
			{
			case NORMAL_E:  //normal road
			case T_ROAD_CROSS:
				{
					parSeg.push_back(segId);        //for normal section set parSeg set to itself
					_parSeg.push_back(parSeg);
					parSeg.swap(vector<uint32>());
					break;
				}

			default:
				{
					vector<uint32> vecPrevId = _secCfgInfo[i].prevSegId;
					vector<uint32> vecNextId = _secCfgInfo[i].nextSegId;
					vector<uint32> parSeg1,parSeg2;
					if (!vecNextId.empty())
					{

						uint32 nextId = vecNextId[0];
						parSeg1 = _secCfgInfo[nextId-1].prevSegId;
					}

					if (!vecPrevId.empty())
					{

						uint32 prevId = vecPrevId[0];
						parSeg2 = _secCfgInfo[prevId-1].nextSegId;
					}       

					sort(begin(parSeg1), end(parSeg1));
					sort(begin(parSeg2), end(parSeg2));
					set_union(begin(parSeg1), end(parSeg1),begin(parSeg2), end(parSeg2),back_inserter(parSeg));

					roadSegConfig_gp->getflyCrossId( segId,flyCrossSecId);

					if ( !flyCrossSecId.empty() )
					{
						vector<uint32> newParSeg;
						sort(begin(parSeg), end(parSeg));
						sort(begin(flyCrossSecId), end(flyCrossSecId));
						set_union(begin(parSeg), end(parSeg),begin(flyCrossSecId), end(flyCrossSecId),back_inserter(newParSeg));
						_parSeg.push_back(newParSeg);
					}
					else
					{
						_parSeg.push_back(parSeg);
					}

					parSeg.swap(vector<uint32>());
					flyCrossSecId.swap(vector<uint32>());
					break;
				} //end default
			}  //end switch segType
#endif
#if 1
			uint32 prePtId = _secCfgInfo[i].prePointId;
			vector<uint32> preConnSecs = _secPointInfo[prePtId].connSecs;
			uint32 nextPtId = _secCfgInfo[i].nextPointId;
			vector<uint32> nextConnSecs = _secPointInfo[nextPtId].connSecs;

			vector<uint32> connSecs;
			sort(begin(preConnSecs), end(preConnSecs));
			sort(begin(nextConnSecs), end(nextConnSecs));
			set_union(begin(preConnSecs), end(preConnSecs),begin(nextConnSecs), end(nextConnSecs), back_inserter(connSecs));

			vector<uint32> preSecs = _secCfgInfo[i].prevSegId;
			vector<uint32> nextSecs = _secCfgInfo[i].nextSegId;

			int j;
			for ( j = 0; j < preSecs.size(); j++)
			{
				uint32 currSeg = preSecs[j];
				vector<uint32>::iterator itrConnSecs = find(connSecs.begin(), connSecs.end(), currSeg);
				if ( itrConnSecs != connSecs.end() )  
				{
					// currSeg is in connSecId
					itrConnSecs = connSecs.erase(itrConnSecs);
				}
			}

			for ( j = 0; j < nextSecs.size(); j++)
			{
				uint32 currSeg = nextSecs[j];
				vector<uint32>::iterator itrConnSecs = find(connSecs.begin(), connSecs.end(), currSeg);
				if ( itrConnSecs != connSecs.end() )  
				{
					// currSeg is in connSecId
					itrConnSecs = connSecs.erase(itrConnSecs);
				}
			}
			//_parSeg.push_back( connSecs );

			roadSegConfig_gp->getflyCrossId( segId,flyCrossSecId);

			if ( !flyCrossSecId.empty() )
			{
				vector<uint32> parSeg;
				sort(begin(parSeg), end(parSeg));
				sort(begin(flyCrossSecId), end(flyCrossSecId));
				set_union(begin(connSecs), end(connSecs),begin(flyCrossSecId), end(flyCrossSecId),back_inserter(parSeg));
				_parSeg.push_back(parSeg);
			}
			else
			{
				_parSeg.push_back( connSecs );
			}
#endif
		} //end for

		//get the minimum length of all sections
		uint32 prePointId,nextPointId;
		point3D_t prePoint,nextPoint;
		double segLengh;
		_minSegLen = MAX_SEC_LENGH;
		for ( int i = 0;i<_secCfgInfo.size();i++ )
		{
			prePointId = _secCfgInfo[i].prePointId;
			prePoint = _bodyPts3D[prePointId];
			nextPointId = _secCfgInfo[i].nextPointId;
			nextPoint = _bodyPts3D[nextPointId];
			segLengh = getLength( prePoint,nextPoint );

			if ( segLengh < _minSegLen )
			{
				_minSegLen = segLengh;
			}
		}

		//set sample method
		//_smpltype = USE_POINTNUM;
		_smpltype = USE_METER;

		return true;
	}  //end initCfg

	bool CSecRptData2::saveCfg()
	{
		//save all sections: segId,bodyLeftLon,bodyLeftLat,bodyRightLon,bodyRightLon
		char* secsPath = "secsCfg.txt";
		// open file for write
		FILE *fpSecs = nullptr;
		errno_t err = fopen_s(&fpSecs, secsPath, "w+");
		if (0 != err)
		{
			printf("failed to open file %s\n", secsPath);
			return false;
		}

		for (int j = 0;j<_secCfgInfo.size();j++)
		{
			uint32 segId = _secCfgInfo[j].secId;
			uint32 bodyLeftId = _secCfgInfo[j].prePointId;
			uint32 bodyRightId = _secCfgInfo[j].nextPointId;

			double bodyLeftLon = _bodyPts3D[bodyLeftId].lon;
			double bodyLeftLat = _bodyPts3D[bodyLeftId].lat;
			double bodyRightLon = _bodyPts3D[bodyRightId].lon;
			double bodyRightLat = _bodyPts3D[bodyRightId].lat;

			double exLeftLon = _secCfgInfo[j].prePoint_ext.lon;
			double exLeftLat = _secCfgInfo[j].prePoint_ext.lat;
			double exRightLon = _secCfgInfo[j].nextPoint_ext.lon;
			double exRightLat = _secCfgInfo[j].nextPoint_ext.lat;

			fprintf_s(fpSecs, "%d,% lf,% lf,% lf,% lf,% lf,% lf,% lf,% lf\n",
				      segId, bodyLeftLon, bodyLeftLat, bodyRightLon, bodyRightLat,
					  exLeftLon, exLeftLat, exRightLon, exRightLat);
		}
		fclose(fpSecs);

		// save all points : id,lon,lat
		char* ptsPath = "ptsCfg.txt";
		FILE *fpPts = nullptr;
		err = fopen_s(&fpPts, ptsPath, "w+");
		if (0 != err)
		{
			printf("failed to open file %s\n", ptsPath);
			return false;
		}

		for (int i = 0;i<_bodyPts3D.size();i++)
		{
			uint32 pointId = i;
			double pointLon = _bodyPts3D[i].lon;
			double pointLat = _bodyPts3D[i].lat;
			
			fprintf_s(fpPts, "%d,% lf,% lf\n", pointId, pointLon, pointLat);
		}
		fclose(fpPts);
	}

	void CSecRptData2::saveData(IN  list<list<vector<point3D_t>>> rptData,
		                        IN  char* filename)
	{
		list<list<vector<point3D_t>>>::iterator rptItor = rptData.begin();
		list<vector<point3D_t>>::iterator leftLineItor,rightLineItor;

		if (!rptItor->empty())
		{
			leftLineItor = rptItor->begin();
			rightLineItor = (++(rptItor->begin()));
		}
		// write left line
		FILE *fp = nullptr;
		errno_t err = fopen_s(&fp, filename, "w+");
		if (0 != err)
		{
			printf("failed to open file %s\n", filename);
			return;
		}

		if (!leftLineItor->empty())
		{
			vector<point3D_t>::iterator pntL = leftLineItor->begin();
			vector<point3D_t>::iterator pntR = rightLineItor->begin();
			for (; pntL != leftLineItor->end(); pntL++)
			{
				fprintf_s(fp, "%lf, %lf, %f, ", pntL->lon, pntL->lat, pntL->paintFlag);
				fprintf_s(fp, "%lf, %lf, %f\n", pntR->lon, pntR->lat, pntR->paintFlag);
				pntR++;
			}
		}
		fclose(fp);
	}

	void CSecRptData2::loadData(IN  char* filename,
		                        OUT list<list<vector<point3D_t>>> &rptData)
	{ 
		if (!rptData.empty())
		{
			rptData.swap(list<list<vector<point3D_t>>>());
		}
		vector<point3D_t> leftLine,rightLine;
		// read left line
		FILE *fp = nullptr;
		errno_t err = fopen_s(&fp, filename, "r");
		if (0 != err)
		{
			printf("failed to open file %s\n", filename);
			return;
		}

		while (!feof(fp))
		{
			point3D_t leftpnt = { 0 }, rightpnt = { 0 };
			fscanf_s(fp, "%lf, %lf, %f, %lf, %lf, %f\n",
				     &leftpnt.lon, &leftpnt.lat, &leftpnt.paintFlag,
				     &rightpnt.lon, &rightpnt.lat, &rightpnt.paintFlag);
			leftLine.push_back(leftpnt);
			rightLine.push_back(rightpnt);
		}

		list<vector<point3D_t>> lane;
		lane.push_back(leftLine);
		lane.push_back(rightLine);

		rptData.push_back(lane);

		fclose(fp);
	}

	CSecRptData2::~CSecRptData2(void)
	{
		_secCfgInfo.swap(vector<secCfgInfo_t>());
		_secPointInfo.swap(vector<secPointInfo_t>());
		_bodyPts3D.swap(vector<point3D_t>());
		_parSeg.swap(vector<vector<uint32>>());
		_sampleInterval = 0;
	}

	void CSecRptData2::debugPrintf(const string errorPrint)
	{
#if DEBUG_PRINT_ON
		printf(errorPrint.c_str());
#endif
	}

	/******************************
	*get extracted section data out 
	******************************/

	int CSecRptData2::segMultiRptData(IN  list<list<vector<point3D_t>>> rptData,
		                              IN  uint32 sampleInterval,
		                              OUT list<reportSectionData> &secData)
	{		
		if(!secData.empty())
		{
			secData.clear();
		}

		_sampleInterval = sampleInterval;

#if SAVE_CURR_RPT_DATA 
		char* savePath = "curr_rpt_data.txt";
		saveData( rptData,savePath );
#endif
#if READ_DATA_FROM_FILE
		char* readPath = "curr_rpt_data.txt";
		loadData( readPath,rptData );
#endif
		list<vector<sampleSectionBody_t>> SecDataInfo;
		vector<sampleSectionBody_t> singleMatchedDataInfo;
		vector<int> isValidData;
		int count = 0;

		list<list<vector<point3D_t>>> ::iterator rptDataItor = rptData.begin();

		/* Deal with rptData one by one*/
		while(rptDataItor != rptData.end())
		{
			list<vector<point3D_t>>::iterator lanesItor = (*rptDataItor).begin();//longitude,latitude of left and right lane
			if((*lanesItor).empty())
			{
				debugPrintf("segMultiRptData:the input report data is empty\n");
				return false;
			}

			getSingleRptDataInfo(*lanesItor,singleMatchedDataInfo);
			if(!singleMatchedDataInfo.empty())
			{
				SecDataInfo.push_back(singleMatchedDataInfo);
				isValidData.push_back(count);
			}
			count++;
			rptDataItor++;
		}

		extractSecData(SecDataInfo,rptData,isValidData,secData);

		return true;
	}

	
	//Get section's segID,startRow,endRow of each rptData
	int CSecRptData2::getSingleRptDataInfo(IN  vector<point3D_t> singleRptData,
		                                   OUT vector<sampleSectionBody_t> &singleMatchedDataInfo)
	{
		if(!singleMatchedDataInfo.empty())
		{
			singleMatchedDataInfo.clear();
		}

		if ( singleRptData.empty() )
		{
			debugPrintf("getSingleRptDataInfo:The singleRptData is empty!\n");
			return false;
		}
		list<samplePoint_t> samplePoint;
		vector<sampleSectionBody_t> sampleSectionBody;
		list<sampleSectionOverlap_t> sampleSectionOverlap;
		vector<sampleSectionBody_t> aMatchNZData;

		/*step1:re_sample reported data from vehicle.*/
		resampleData(singleRptData,samplePoint);

		if(samplePoint.empty())
		{
			debugPrintf("resampleData:The sample point is empty!\n");
			return false;
		}
		/*step2:find section ID for every sample point.*/
		findSamplePtsSecID( samplePoint );

		/*step3:get every coarse section body Info of one rptData.*/
		getSecBodyInfo( samplePoint,singleRptData,sampleSectionBody );

		if(sampleSectionBody.empty())
		{
			debugPrintf("getSecBodyInfo:the sampleSectionBody is empty!\n");
			return false;
		}

		/*step4:get every coarse section overlap of rptData.*/
		getSecOverlap(sampleSectionBody,singleRptData,sampleSectionOverlap);

		if(sampleSectionOverlap.empty())
		{
			debugPrintf("getSecOverlapInfo:the sampleOverlapSection is empty!\n");
			return false;
		}
		/*step5:rot the section in secConfigList and its corresponding coarse section overlap 
		        in rptData to horizontal.find the exact section overlap in rptData*/
		rotAndCompare(sampleSectionOverlap,sampleSectionBody,singleRptData,singleMatchedDataInfo);
		if(singleMatchedDataInfo.empty())
		{
			debugPrintf("rotAndCompare : the singleMatchedDataInfo is empty!\n");
			return false;
		}

		//check if the wanted output data index beyond the rptData size
		vector<sampleSectionBody_t>::iterator itMatchedData = singleMatchedDataInfo.begin();
		while ( itMatchedData != singleMatchedDataInfo.end() )
		{
			uint32 startLoc = (*itMatchedData).startLoc;
			uint32 endLoc = (*itMatchedData).endLoc;
			if ( startLoc >= singleRptData.size() || endLoc >= singleRptData.size())
			{
				debugPrintf("getSingleRptDataInfo : out data index beyond the rptData size! Delete this section!\n");
				itMatchedData = singleMatchedDataInfo.erase( itMatchedData );
			}
			else
			{
		        itMatchedData++;
			}
		}

		return true;
	}

	int CSecRptData2::extractSecData(IN list<vector<sampleSectionBody_t>> SecDataInfo,
		                             IN list<list<vector<point3D_t>>> rptData,
		                             IN vector<int> isValidData,
		                             OUT list<reportSectionData> &secData)
	{
		list<vector<point3D_t>> ::iterator itr;
		vector<point3D_t> ::iterator leftItor,rightItor;

		int startRow = 0;
		int endRow = 0;

		// for section containing valid data, iterate each group
		for(int dataIndex = 0;dataIndex < isValidData.size();dataIndex++)
		{
			list<vector<sampleSectionBody_t>>::iterator videoNumItor = SecDataInfo.begin();
			list<list<vector<point3D_t>>> ::iterator rptDataItor = rptData.begin();

			advance(videoNumItor,dataIndex);
			advance(rptDataItor,isValidData[dataIndex]);

			for(int index = 0;index< (*videoNumItor).size();index++)
			{
				reportSectionData middleData;
				rptSecData_t rptSeg;

				vector<point3D_t> aSecLeftLaneData,aSecRightLaneData;  //segID left or right lane data
				startRow = (*videoNumItor)[index].startLoc;
				endRow = (*videoNumItor)[index].endLoc;

				itr = (*rptDataItor).begin();
				leftItor = (*itr).begin();
				advance(leftItor,startRow);
				vector<point3D_t> ::iterator itrBegin =  leftItor;
				advance(leftItor,endRow-startRow);
				vector<point3D_t> ::iterator itrEnd =  leftItor;
				aSecLeftLaneData.insert( aSecLeftLaneData.end(),itrBegin,itrEnd );

				advance(itr,1);
				rightItor = (*itr).begin();
				advance(rightItor,startRow);
				vector<point3D_t> ::iterator itrBegin1 =  rightItor;
				advance(rightItor,endRow-startRow);
				vector<point3D_t> ::iterator itrEnd1 =  rightItor;
				aSecRightLaneData.insert(aSecRightLaneData.end(),itrBegin1,itrEnd1);

				// reverse data if reported direction is not the same as configuration
				if ((*videoNumItor)[index].reverseFlag == TRUE )
				{
					vector<point3D_t> tempaSecLaneData;
					tempaSecLaneData = aSecLeftLaneData;
					aSecLeftLaneData = aSecRightLaneData;
					aSecRightLaneData = tempaSecLaneData;
					reverse(aSecLeftLaneData.begin(),aSecLeftLaneData.end());
					reverse(aSecRightLaneData.begin(),aSecRightLaneData.end());
				}

				rptSeg.revDirFlag = (*videoNumItor)[index].reverseFlag;
				rptSeg.rptLaneData.push_back(aSecLeftLaneData);
				rptSeg.rptLaneData.push_back(aSecRightLaneData);

				middleData.sectionId = (*videoNumItor)[index].segId;
				middleData.rptSecData.push_back(rptSeg);
				// construct output section data
				if (!middleData.rptSecData.empty())
				{
					secData.push_back(middleData);
				}
			}
		} 
		return true;
	}
/*
	int CSecRptData2::extractSecData(IN list<vector<sampleSectionBody_t>> SecDataInfo,
		                             IN list<list<vector<point3D_t>>> rptData,
		                             IN vector<int> isValidData,
		                             OUT list<reportSectionData> &secData)
	{
		list<vector<point3D_t>> ::iterator itr;
		vector<point3D_t> ::iterator leftItor,rightItor;
		vector<secCfgInfo_t> ::iterator secConfigItor = _secCfgInfo.begin();

		int startRow = 0;
		int endRow = 0;

        // iterate all sections to get reported section data
		for(int sectionIndex = 0;sectionIndex < _secCfgInfo.size();sectionIndex++)
		{
			uint32 currSegID = _secCfgInfo[sectionIndex].secId;
			reportSectionData middleData;

            // for section containing valid data, iterate each group
			for(int dataIndex = 0;dataIndex < isValidData.size();dataIndex++)
			{

				list<vector<sampleSectionBody_t>>::iterator videoNumItor = SecDataInfo.begin();
				list<list<vector<point3D_t>>> ::iterator rptDataItor = rptData.begin();

				advance(videoNumItor,dataIndex);
				advance(rptDataItor,isValidData[dataIndex]);

				for(int index = 0;index< (*videoNumItor).size();index++)
				{
                    rptSecData_t rptSeg;

					vector<point3D_t> aSecLeftLaneData,aSecRightLaneData;  //segID left or right lane data
					if((*videoNumItor)[index].segId == currSegID)
					{
						startRow = (*videoNumItor)[index].startLoc;
						endRow = (*videoNumItor)[index].endLoc;

						itr = (*rptDataItor).begin();
						leftItor = (*itr).begin();
						advance(leftItor,startRow);
						vector<point3D_t> ::iterator itrBegin =  leftItor;
						advance(leftItor,endRow-startRow);
						vector<point3D_t> ::iterator itrEnd =  leftItor;
						aSecLeftLaneData.insert( aSecLeftLaneData.end(),itrBegin,itrEnd );

						advance(itr,1);
						rightItor = (*itr).begin();
						advance(rightItor,startRow);
						vector<point3D_t> ::iterator itrBegin1 =  rightItor;
						advance(rightItor,endRow-startRow);
						vector<point3D_t> ::iterator itrEnd1 =  rightItor;
						aSecRightLaneData.insert(aSecRightLaneData.end(),itrBegin1,itrEnd1);

                        // reverse data if reported direction is not the same as configuration
						if ((*videoNumItor)[index].reverseFlag == TRUE )
						{
							vector<point3D_t> tempaSecLaneData;
							tempaSecLaneData = aSecLeftLaneData;
							aSecLeftLaneData = aSecRightLaneData;
							aSecRightLaneData = tempaSecLaneData;
							reverse(aSecLeftLaneData.begin(),aSecLeftLaneData.end());
							reverse(aSecRightLaneData.begin(),aSecRightLaneData.end());
						}

						vector<point3D_t> ::iterator aSecLeftLaneItor;
						vector<point3D_t> ::iterator aSecRightLaneItor = aSecRightLaneData.begin();

                        // remove lane change points
						for(aSecLeftLaneItor = aSecLeftLaneData.begin();aSecLeftLaneItor!= aSecLeftLaneData.end();)
						{
                            {
                                aSecLeftLaneItor++;
                                aSecRightLaneItor++;
                            }
						}

                        rptSeg.revDirFlag = (*videoNumItor)[index].reverseFlag;
                        rptSeg.rptLaneData.push_back(aSecLeftLaneData);
                        rptSeg.rptLaneData.push_back(aSecRightLaneData);

                        middleData.sectionId = _secCfgInfo[sectionIndex].secId;
                        middleData.rptSecData.push_back(rptSeg);
					}
				}
			}

            // construct output section data
			if (!middleData.rptSecData.empty())
			{
				secData.push_back(middleData);
			}
			secConfigItor++;
		} 
		return true;
	}
*/
	int CSecRptData2::resampleData(IN  vector<point3D_t> singleRptData,
	                               OUT list<samplePoint_t> &samplePoint)
	{
		if( 0 >= _sampleInterval)
		{
			debugPrintf("resampleData:the sampleInterval is invalid! It should be larger than zero\n");
			return false;
		}

		if (!samplePoint.empty())
		{
			samplePoint.clear();
		}

		if ( USE_POINTNUM == _smpltype )
		{
			int rptDataLengh = singleRptData.size();
			samplePoint_t endSamplePoint;
			int pointNum;

			if ( _sampleInterval > rptDataLengh )
			{
				debugPrintf("resampleData:the sampleInterval is biger than the input video!; Please decrease it!\n");
				return false;
			}
			else
			{
				pointNum = (int)(ceil(double(rptDataLengh)/double(_sampleInterval)));
			}

			for (int i = 1;i < pointNum;i++)
			{
				samplePoint_t currSamplePoint;
				currSamplePoint.segId = 0;    //initialize segId to 0
				currSamplePoint.startLoc = (i-1)*_sampleInterval;
				currSamplePoint.leftLane.lon = singleRptData[(i-1)*_sampleInterval].lon;
				currSamplePoint.leftLane.lat = singleRptData[(i-1)*_sampleInterval].lat;

				samplePoint.push_back(currSamplePoint);
			}

			//for the tailer which length is not enough for a sample interval,
			//set the last video point to the last sample point 
			endSamplePoint.segId = 0;
			endSamplePoint.startLoc = rptDataLengh-1;
			endSamplePoint.leftLane.lat = singleRptData[rptDataLengh-1].lat;
			endSamplePoint.leftLane.lon = singleRptData[rptDataLengh-1].lon;
			samplePoint.push_back(endSamplePoint);
		} 
		else  // USE_METER == _smpltype
		{
			if ( _sampleInterval > _minSegLen )
			{
				debugPrintf("SecRptData : the sampleInterval is larger than minSegLen. Will decrease it to _minSegLen automatically !\n" );
				//return false;
			} 
			//else
			{
				_sampleInterval = _minSegLen;  //by meter
				int i;
				int startIdx = 0;
				int pointNum = 0;
				samplePoint_t currSamplePoint;
				int numOfsingleRpt = singleRptData.size();
				int count = singleRptData.size();
				while ( count-- )
				{
					double dist = 0;
					for ( i = startIdx; i < (numOfsingleRpt-1); i++ )
					{
						dist += getLength(singleRptData[i],singleRptData[i+1]);
						if ( dist >= _sampleInterval )
						{
							if ( pointNum == 0 )
							{
								pointNum++;
								currSamplePoint.segId = 0;    //initialize segId to 0
								currSamplePoint.startLoc = 0;
								currSamplePoint.leftLane.lon = singleRptData.front().lon;
								currSamplePoint.leftLane.lat = singleRptData.front().lat;
								samplePoint.push_back(currSamplePoint);
							}

							currSamplePoint.segId = 0;    //initialize segId to 0
							currSamplePoint.startLoc = i+1;
							currSamplePoint.leftLane.lon = singleRptData[i+1].lon;
							currSamplePoint.leftLane.lat = singleRptData[i+1].lat;
							samplePoint.push_back(currSamplePoint);

							startIdx = i+1;
							pointNum++;
							dist = 0;
							continue;
						}
						else if ( (!samplePoint.empty()) && ( i == (numOfsingleRpt - 2)) )
						{
							pointNum++;
							currSamplePoint.segId = 0;    //initialize segId to 0
							currSamplePoint.startLoc = numOfsingleRpt-1;
							currSamplePoint.leftLane.lon = singleRptData.back().lon;
							currSamplePoint.leftLane.lat = singleRptData.back().lat;
							samplePoint.push_back(currSamplePoint);
							break;
						}
					} //end calc dist

					if ( i == (numOfsingleRpt - 2) )
					{
						break;
					}

				} //end while
			} //end _sampleInterval legal judgment
		} //end USE_METER

		//check if has zero lan & lat value in sample point
		list<samplePoint_t>::iterator itrSamplePoint = samplePoint.begin();
		while ( itrSamplePoint != samplePoint.end())
		{
			if (itrSamplePoint->leftLane.lat == 0 && itrSamplePoint->leftLane.lon == 0)
			{
				debugPrintf("resampleData warning: this repData has zero longitude and latitude in left line.Please check it!\n");
				samplePoint.clear();
				return false;
				//break;
			} 
			itrSamplePoint++;
		}
		return true;
	}

	double CSecRptData2::getLength( IN point3D_t point1,
		                            IN point3D_t point2 )
	{
		double diff_x = abs(point2.lon - point1.lon);      
		double diff_y = abs(point2.lat - point1.lat);        
		double length = sqrt(long double(diff_x* diff_x+ diff_y*diff_y));
		return length;
	}

	double CSecRptData2::getAngle(IN point3D_t samplePoint,
		                          IN point3D_t sectionPoint,
		                          IN point3D_t connPoint)
	{
		double a = getLength(sectionPoint, samplePoint);
		double b = getLength(sectionPoint, connPoint);      
		double c = getLength(samplePoint,connPoint);     

		double cosVal =( a*a+ b*b - c*c) / (a * b * 2); 
		return cosVal;
	}

	int CSecRptData2::getSampleSectionID(IN samplePoint_t oneSamplePoint,
		                                 OUT uint32 &sampleSectionID)
	{
		//get the minDist sectionID 
		sampleSectionID = 0;
		vector<double> distance(_bodyPts3D.size());
		for (int i = 0;i < distance.size();i++)
		{
			distance[i] = getLength( oneSamplePoint.leftLane, _bodyPts3D[i] );
		}
		uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin() );
		double min_Dist = distance[min_pos];
		if ( min_Dist > MAX_POINT_TO_SEC_DIST )
		{
			debugPrintf("getSampleSectionID warning: the samplePoint is too far away from the configure road .Delete it!\n");
			sampleSectionID = -1;
			return -1;
		} 

		uint32 minDisPointID = min_pos;
		point3D_t minPt3D =  _bodyPts3D[min_pos];
		vector<uint32> connPtsId = _secPointInfo[min_pos].connPtsId;
		vector<point3D_t> connPts3D;
		for (int i = 0;i<connPtsId.size();i++)
		{
			uint32 connPtId  = connPtsId[i];
			connPts3D.push_back(_bodyPts3D[connPtId]);
		}
		vector<uint32> connSections = _secPointInfo[min_pos].connSecs;
		point3D_t samplePoint = oneSamplePoint.leftLane;
		if ( connPtsId.size()==1 )  //road start and end point
		{
			double cosVal = getAngle(samplePoint,minPt3D,connPts3D[0]);
			if (cosVal < 0)
			{
				sampleSectionID = 0;  //set road data before start and end section belongs section 0
			} 
			else
			{
				sampleSectionID = connSections[0];
			}
		} 
		else
		{
			int connPointsNum = connPts3D.size();
			vector<double> cosMat(connPointsNum);
			point3D_t connPt3D;
			for (int i = 0;i<connPointsNum;i++)
			{
				connPt3D = connPts3D[i];
				double cosVal = getAngle(samplePoint,minPt3D,connPt3D);
				cosMat[i] = cosVal;
			}
			uint32 minDegreeLoc = (uint32) ( max_element(cosMat.begin(),cosMat.end()) - cosMat.begin() );
			sampleSectionID = connSections[minDegreeLoc];

		}
		return true;
	}

	void CSecRptData2::findSamplePtsSecID(INOUT list<samplePoint_t> &samplePoint)
	{
		list<samplePoint_t>::iterator itrSamplePoint;
		itrSamplePoint = samplePoint.begin();

		while (itrSamplePoint != samplePoint.end())
		{
			samplePoint_t oneSamplePoint = *itrSamplePoint;
			uint32 sampleSectionID = 0;
			int tmp = getSampleSectionID( oneSamplePoint,sampleSectionID );
			if ( -1 == tmp )
			{
				//delete too far away point
				itrSamplePoint = samplePoint.erase(itrSamplePoint);
			} 
			else
			{
				itrSamplePoint->segId = sampleSectionID;
				itrSamplePoint++;
			}
		}
	}

	BOOL CSecRptData2::getSectionBoundry( IN list<samplePoint_t> samplePoint,
		                                  INOUT uint32 &seekStartIdx,
		                                  OUT sampleSectionBody_t &aSampleBodySection,
		                                  OUT BOOL &selectOverFlag )
	{
		//initialize aSampleBodySection
		sampleSectionBody_t initSampleBodySection = {0,0,0,0};

		if ((aSampleBodySection.segId ||aSampleBodySection.startLoc ||aSampleBodySection.endLoc || aSampleBodySection.reverseFlag)!=0 )
		{
			aSampleBodySection = initSampleBodySection;
		}

		//get the current aSampleBodySection sectionID
		uint32 currtID = 0;

		list<samplePoint_t>::iterator itrSamplePoint = samplePoint.begin();
		if (seekStartIdx == 0)
		{
			seekStartIdx = 1;
			aSampleBodySection = initSampleBodySection;
			selectOverFlag = 0;
			return true;
		} 
		else
		{
			advance(itrSamplePoint,seekStartIdx); //get the (seekStartRow)th data
			samplePoint_t currSamplePoint = *itrSamplePoint;
			currtID = currSamplePoint.segId;
		}

		//find the start and end boundary of sample body section
		uint32 sectionStartIdx = -1;
		uint32 sectionEndIdx = -1;
		uint32 changePointNum = 0;
		uint32 secType = DEFAULT_E;  //default secType

		if( 0 == currtID )
		{
			int numOfSamplePnts = samplePoint.size();
			if (seekStartIdx >= (numOfSamplePnts - 1))
			{
				selectOverFlag = true;
			} 
			else
			{
				selectOverFlag = false;
				seekStartIdx++;
			}
			return true;
		}

		int i,j;

		if (secType==0)
		{
			for ( i = seekStartIdx-1;i >= 0;i--)
			{
				itrSamplePoint = samplePoint.begin();
				advance(itrSamplePoint,i); //get the index = (i)th data
				samplePoint_t tmpSamplePoint = *itrSamplePoint;
				if (tmpSamplePoint.segId != currtID)
				{
					sectionStartIdx = tmpSamplePoint.startLoc;
					changePointNum = changePointNum + 1;
					break;
				}
			}

			for ( j = seekStartIdx+1;j < samplePoint.size();j++)
			{
				itrSamplePoint = samplePoint.begin();
				advance(itrSamplePoint,j); //get the index = (j)th data
				samplePoint_t tmpSamplePoint = *itrSamplePoint;
				if (tmpSamplePoint.segId != currtID)
				{
					sectionEndIdx = tmpSamplePoint.startLoc;
					seekStartIdx = j;
					changePointNum = changePointNum + 1;
					break;
				}
			}
		} 
		else
		{
			vector<uint32> parSeg = _parSeg[currtID-1];
			vector<uint32>::iterator itParSeg;

			for ( i = (seekStartIdx-1);i >= 0;i--)
			{
				itrSamplePoint = samplePoint.begin();
				advance(itrSamplePoint,i); //get the index = (i)th data
				samplePoint_t tmpSamplePoint = *itrSamplePoint;
				uint32 tmpSegId = tmpSamplePoint.segId;

				itParSeg = find(parSeg.begin(),parSeg.end(),tmpSegId);

				if ( itParSeg!=parSeg.end() )  // tmpSegId is in parSeg
				{
					continue;
				}
				else    // found a segId which is not in parSeg
				{
					sectionStartIdx = tmpSamplePoint.startLoc;
					changePointNum = changePointNum + 1;
					break;
				}

			}

			for ( j = seekStartIdx+1;j < samplePoint.size();j++)
			{
				itrSamplePoint = samplePoint.begin();
				advance(itrSamplePoint,j); //get the index = (j)th data
				samplePoint_t tmpSamplePoint = *itrSamplePoint;
				uint32 tmpSegId = tmpSamplePoint.segId;

				itParSeg = find(parSeg.begin(),parSeg.end(),tmpSegId);

				if ( itParSeg!=parSeg.end() )  // tmpSegId is in parSeg
				{
					continue;
				}
				else   // found a segId which is not in parSeg
				{
					sectionEndIdx = tmpSamplePoint.startLoc;
					seekStartIdx = j;
					changePointNum = changePointNum + 1;
					break;
				}
			}

			// adjust the current segID by the maximum probability of occurrence
			vector<uint32> interDiff;
			for (int k = (i+1); k<=(j-1); k++)
			{
				itrSamplePoint = samplePoint.begin();
				advance(itrSamplePoint,k); //get the index = (k)th data
				samplePoint_t tmpSamplePoint = *itrSamplePoint;
				uint32 tmpSegId = tmpSamplePoint.segId;
				interDiff.push_back(tmpSegId);
			}
			uint32 maxCount = 0;
			for (int m = 0;m < parSeg.size();m++)
			{
				uint32 repeatElem = parSeg[m];
				uint32 repeatCount= count(interDiff.begin(),interDiff.end(),repeatElem);
				if ( maxCount<repeatCount )
				{
					maxCount = repeatCount;
					currtID = repeatElem;
				}
			}
		}

		// read all resample points or not
		if (j >= samplePoint.size()) 
		{
			selectOverFlag = true;
		} 
		else
		{
			selectOverFlag = false;
		}

		//output:get section ID
		uint32 sectionID = 0; 
		if ( changePointNum == 2 )
		{
			sectionID = currtID;
			aSampleBodySection.startLoc = sectionStartIdx;
			aSampleBodySection.endLoc = sectionEndIdx;
			aSampleBodySection.segId = sectionID;
			aSampleBodySection.reverseFlag = 0;  //not set yet
		} 
		else
		{
			aSampleBodySection = initSampleBodySection;
			return true;
		} 
		return true;
	}

	double CSecRptData2::compareSegLen(IN vector<point3D_t> singleRptData, 
		                               IN sampleSectionBody_t oneSectionBody)
	{
		int dataIdx;
		double bodyDist = 0;
		int beginIdx = oneSectionBody.startLoc;
		int endIdx = oneSectionBody.endLoc;

		for ( dataIdx = beginIdx; dataIdx < endIdx; dataIdx++ )
		{
			bodyDist += getLength( singleRptData[dataIdx], singleRptData[dataIdx+1] );
		}

		uint32 segId = oneSectionBody.segId;
		int prePointId = _secCfgInfo[segId-1].prePointId;
		int nextPointId = _secCfgInfo[segId-1].nextPointId;

		double segLengh = getLength( _bodyPts3D[prePointId], _bodyPts3D[nextPointId] );

		return bodyDist/segLengh;
	}

	bool CSecRptData2::mergeShortSeg(IN  vector<point3D_t> singleRptData,
		                             INOUT  vector<sampleSectionBody_t> &sampleSectionBody,
									 INOUT int &mergeNO)
	{
		vector<sampleSectionBody_t>::iterator itrBody = sampleSectionBody.begin();
		double lenRatio;
		while ( itrBody!=sampleSectionBody.end() )
		{

			lenRatio = compareSegLen( singleRptData, *itrBody );

			uint32 curSegId,preSegId,nextSegId;
			char warningStr[MAX_PATH];

			//get the section left and right boundary
			point3D_t srcLeft = _secCfgInfo[itrBody->segId - 1].prePoint_ext;
			point3D_t srcRight = _secCfgInfo[itrBody->segId - 1].nextPoint_ext;
			//get section angle and compare
			double cfgAngleRad = atan2((srcRight.lat - srcLeft.lat),(srcRight.lon - srcLeft.lon ));

			double dataAngleRad = atan2((singleRptData[itrBody->startLoc].lat - singleRptData[itrBody->endLoc].lat),
				                        (singleRptData[itrBody->startLoc].lon - singleRptData[itrBody->endLoc].lon));
			double dataAngleDeg = dataAngleRad * 180 / CV_PI;
			double cfgAngleDeg = cfgAngleRad * 180 / CV_PI;
			dataAngleDeg = dataAngleDeg > 0 ? dataAngleDeg : (180+dataAngleDeg);
			cfgAngleDeg = cfgAngleDeg > 0 ? cfgAngleDeg : (180+cfgAngleDeg);
			double angleDiff = abs(cfgAngleDeg - dataAngleDeg) < (180-abs(cfgAngleDeg - dataAngleDeg)) ? abs(cfgAngleDeg - dataAngleDeg) : (180-abs(cfgAngleDeg - dataAngleDeg));
			double norAngleDiff = angleDiff < 90 ? 1-angleDiff/90 : 1-(180 - angleDiff)/90;
			double norLenRatio = lenRatio < 1 ? lenRatio : 1;

			//if ( lenRatio < THRESH_MATCH_SEG_TO_CFG_SEG_LENGH_RATIO )
			if ( lenRatio*norAngleDiff < THRESH_LENRATIO_MUL_ANGLEDIFF)
			{
				mergeNO++;

				if (  itrBody==sampleSectionBody.begin() )
				{
					curSegId = itrBody->segId;
					
					if ( sampleSectionBody.size()>1 )
					{
						nextSegId = (itrBody+1)->segId;
						(itrBody+1)->startLoc = itrBody->startLoc;

						sprintf_s( warningStr,"mergeShortSeg : merge short seg %d to seg %d !\n ", curSegId ,nextSegId );
					}
					else
					{
						sprintf_s( warningStr,"mergeShortSeg : delete short seg %d !\n ", curSegId  );
					}
				}
				else
				{
					curSegId = itrBody->segId;
					preSegId = (itrBody-1)->segId;
					(itrBody-1)->endLoc = itrBody->endLoc;

					if ( itrBody != sampleSectionBody.end()-1 )
					{
						(itrBody+1)->startLoc = itrBody->startLoc;
					}

					sprintf_s( warningStr,"mergeShortSeg : merge short seg %d to seg %d !\n ", curSegId , preSegId );
				}

				debugPrintf(warningStr);
				itrBody = sampleSectionBody.erase(itrBody);
			}
			else
			{
				itrBody++;
			}
		}
		return true;
	}

#if 1
	int CSecRptData2::mergeJumpSeg2( vector<sampleSectionBody_t> &sampleSectionBody, vector<point3D_t> singleRptData)
	{
		int changeSegNum = 0;
		vector<sampleSectionBody_t>::iterator itrBodyData = sampleSectionBody.begin();

		//add jump out while condition
		//add parSeg to compare segment
		while( itrBodyData < (sampleSectionBody.end()-1) )
		{
			uint32 currSecId = itrBodyData->segId;
			uint32 nextSecId = (itrBodyData+1)->segId;
			uint32 nextEndLoc = (itrBodyData+1)->endLoc;
			int curSegRepTimes = 0;
			
			if ( nextSecId == currSecId )
			{
				char warningStr[MAX_PATH];
				sprintf_s(warningStr,"rotAndCompare warning: merge repeated seg %d !\n ",currSecId);
				debugPrintf(warningStr);

				itrBodyData->endLoc = nextEndLoc;
				sampleSectionBody.erase(itrBodyData+1);

				continue;
			}

			vector<uint32> vecNextSecId,vecPrevSecId,connSecId;
			vecNextSecId = _secCfgInfo[currSecId-1].nextSegId;
			vecPrevSecId =  _secCfgInfo[currSecId-1].prevSegId;
			sort(begin(vecNextSecId), end(vecNextSecId));
			sort(begin(vecPrevSecId), end(vecPrevSecId));
			set_union(begin(vecNextSecId), end(vecNextSecId),begin(vecPrevSecId), end(vecPrevSecId), back_inserter(connSecId));


			if (find(connSecId.begin(), connSecId.end(), nextSecId) == connSecId.end())  
			{
				// nextSecId is not in connSecId
				char warningStr[MAX_PATH];
				sprintf_s(warningStr,"rotAndCompare warning: seg %d jump to seg %d !", currSecId, nextSecId);
				debugPrintf(warningStr);

				//compare with the two sections ,set the minDataDistY section to current section 
				changeSegNum++;
				vector<vector<point3D_t>> bodyData;
				vector<point3D_t> data0,data1;
				if ( 1==changeSegNum )
				{
					int startLoc = itrBodyData->startLoc;
					int endLoc = itrBodyData->endLoc;
					for (int i = startLoc; i <= endLoc; i++ )
					{
						data0.push_back(singleRptData[i]);
					}
					bodyData.push_back(data0);
				}
				int startLoc = (itrBodyData+1)->startLoc;
				int endLoc = (itrBodyData+1)->endLoc;
				for (int i = startLoc; i <= endLoc; i++ )
				{
					data1.push_back(singleRptData[i]);
				}
				bodyData.push_back(data1);

				vector<uint32> parSeg;
				parSeg.push_back(currSecId);
				parSeg.push_back(nextSecId);

				point3D_t srcLeft,srcRight,rotSrcLeft,rotNew;
				double theta;
				int dataIdx;
				for ( dataIdx = 0; dataIdx< bodyData.size(); dataIdx++ )
				{
					double minMeanDistY = MAX_MEAN_SEC_DIST_Y;
					vector<point3D_t> curData = bodyData[dataIdx];
					for (int i = 0;i<parSeg.size();i++)
					{
						uint32 currSeg = parSeg[i];

						//get the section left and right boundary
						srcLeft = _bodyPts3D[_secCfgInfo[currSeg-1].prePointId];
						srcRight = _bodyPts3D[_secCfgInfo[currSeg-1].nextPointId];
						//get section angle and compare
						theta = -atan2((srcRight.lat - srcLeft.lat),(srcRight.lon - srcLeft.lon));
						rotPoint(srcLeft,theta,rotSrcLeft);

						//get nearest point index
						double sumDistY = 0, dataMeanDistY = 0.0;
						for( int k = 0; k < curData.size(); k++ )
						{
							rotPoint(curData[k],theta,rotNew);
							sumDistY += abs(rotNew.lat - rotSrcLeft.lat);
						}

						if(0 != curData.size())
						{
							dataMeanDistY = sumDistY / curData.size();
						} 

						if ( dataMeanDistY < minMeanDistY )
						{
							minMeanDistY = dataMeanDistY;

							if ( 1 == changeSegNum && 0 == dataIdx )
							{
								if ( itrBodyData->segId != currSeg )
								{
									sprintf_s(warningStr," merge to seg %d !\n ", currSeg);
									debugPrintf(warningStr);
								}
								itrBodyData->segId = currSeg;
							}
							else
							{
								if ( (itrBodyData+1)->segId != currSeg )
								{
									sprintf_s(warningStr," merge to seg %d !\n ", currSeg);
									debugPrintf(warningStr);
								}
								(itrBodyData+1)->segId = currSeg;
							}					
						}
					}
				}

				if ( (itrBodyData+1)->segId == nextSecId )
				{
					curSegRepTimes++;
					if ( curSegRepTimes >5 )
					{
						sprintf_s(warningStr," fail to adjust seg %d !\n ", (itrBodyData+1)->segId);
						debugPrintf(warningStr);
						itrBodyData++;
						continue;
					}
				}

				else
				{
					continue;
				}
			}
			itrBodyData++;
		}	//end while

		return changeSegNum;
	} //end mergeJumpSeg
#endif

	int CSecRptData2::getSecBodyInfo( IN  list<samplePoint_t> samplePoint,
		                              IN  vector<point3D_t> singleRptData,
		                              OUT vector<sampleSectionBody_t> &sampleSectionBody )
	{
		if (!sampleSectionBody.empty())
		{
			sampleSectionBody.clear();
		}

		uint32 seekStartIdx = 0;
		int count = samplePoint.size();

		//merge sample point to body section
		while(count--)
		{
			sampleSectionBody_t aSampleBodySection;
			BOOL selectOverFlag = FALSE;

			getSectionBoundry(samplePoint,seekStartIdx,aSampleBodySection,selectOverFlag );

			if ( aSampleBodySection.segId != 0)
			{
				sampleSectionBody.push_back(aSampleBodySection);
			}
			if (selectOverFlag == 1)
			{
				break;
			}
		}

		if (!sampleSectionBody.empty())
		{
			//decide is there jumping section and merging the jumping section to correct section
			int changeSegNum = MAX_JUMP_SEG_NO;
			int itrNum = 0;
			while ( changeSegNum!=0 )
			{
				if ( itrNum > MAX_JUMP_SEG_NO )
				{
					debugPrintf("getSecBodyInfo : large gps offset!\n ");
					break;
				}
				changeSegNum = mergeJumpSeg2(sampleSectionBody,singleRptData);
				itrNum++;
			}

#if 1
			itrNum = 0;
			int mergeNO = MAX_JUMP_SEG_NO;
			while ( mergeNO!=0 )
			{
				if ( itrNum > MAX_JUMP_SEG_NO )
				{
					debugPrintf("getSecBodyInfo : exist incomplete section !\n ");
					break;
				}
				mergeNO = 0;
				mergeShortSeg( singleRptData, sampleSectionBody, mergeNO );
			}
#endif
		}

		return true;
	}

	double CSecRptData2::getOverlapScale( IN point3D_t dataPt,
		                                  IN uint32 segId )
	{
		point3D_t segPrePt = _bodyPts3D[_secCfgInfo[segId-1].prePointId];
		point3D_t segNextPt = _bodyPts3D[_secCfgInfo[segId-1].nextPointId];
		double preDist = getLength( segPrePt,dataPt);
		double nextDist = getLength( segNextPt,dataPt);
		double segPreScale,segNextScale;

		//decide the current point from which point recently 
		if ( preDist <= nextDist )
		{
			segPreScale = getLength( segPrePt,_secCfgInfo[segId-1].prePoint_ext);
			if ( segPreScale == 0 )
			{
				if ( _secPointInfo[_secCfgInfo[segId-1].prePointId].connPtsId.size() <= 2)
				{
					return 0;
				}
				else
				{
					return T_ROAD_OVERLAP_SCALE;
				}
			}
			else
			{
				return segPreScale;
			}
		} 
		else
		{
			segNextScale = getLength( segNextPt,_secCfgInfo[segId-1].nextPoint_ext);
			if ( segNextScale == 0 )
			{
				if ( _secPointInfo[_secCfgInfo[segId-1].nextPointId].connPtsId.size() <= 2)
				{
					return 0;
				}
				else
				{
					return T_ROAD_OVERLAP_SCALE;
				}
			} 
			else
			{
				return segNextScale;
			}
		}
	}

	int CSecRptData2::getSecOverlap(IN  vector<sampleSectionBody_t> sampleSectionBody,
		                            IN  vector<point3D_t> singleRptData,
		                            OUT list<sampleSectionOverlap_t> &sampleSectionOverlap)
	{
		if(!sampleSectionOverlap.empty())
		{
			sampleSectionOverlap.clear();
		}

		if(singleRptData.empty() || sampleSectionBody.empty())
		{
			return false;
		}

		for (int i = 0;i < sampleSectionBody.size();i++)
		{
			//get segID
			uint32 segId = sampleSectionBody[i].segId;
			sampleSectionOverlap_t SecOverlap;
			SecOverlap.segId = segId;

			//get the leftLine.x & leftLine.y of the sample body section boundary 
			point3D_t bodyLeft,bodyRight;
			uint32 leftIndex = sampleSectionBody[i].startLoc;
			bodyLeft = singleRptData[leftIndex];
			uint32 rightIndex = sampleSectionBody[i].endLoc;
			bodyRight = singleRptData[rightIndex];
			double leftOverlapScale = getOverlapScale( bodyLeft,segId );
			double rightOverlapScale = getOverlapScale( bodyRight,segId );

			//get sample overlap boundary
			int m,n;
			double leftDistance,rightDistance;
			point3D_t leftData,rightData;
			uint32 overlapLeftIndex = 0;
			uint32 overlapRightIndex = singleRptData.size() - 1;

			for ( m = leftIndex;m >= 0;m--)
			{
				leftData = singleRptData[m];

				leftDistance = getLength( bodyLeft,leftData );
				if ( leftDistance >= leftOverlapScale )
				{
					overlapLeftIndex = m;
					break;
				}
			}

			for ( n = rightIndex;n < singleRptData.size();n++)
			{
				rightData = singleRptData[n];

				rightDistance = getLength( bodyRight, rightData );
				if ( rightDistance >= rightOverlapScale )
				{
					overlapRightIndex = n;
					break;
				}
			}

			vector<point3D_t> overlapData;
			for ( int k = overlapLeftIndex;k <= overlapRightIndex;k++)
			{
				point3D_t currPt = singleRptData[k];
				overlapData.push_back(currPt);
			}
			SecOverlap.sampleOverlapData = overlapData;
			overlapData.clear();

			SecOverlap.startLoc = overlapLeftIndex;
			SecOverlap.endLoc = overlapRightIndex;

			sampleSectionOverlap.push_back(SecOverlap);
		}

		return true;
	}

	int CSecRptData2::rotPoint(IN point3D_t sourcePoint,IN double theta,OUT point3D_t &rotedPoint)
	{
		rotedPoint = sourcePoint;
		double x = sourcePoint.lon, y = sourcePoint.lat, realZ = 0, imagZ = 0;
		complex<double> thetaj(0, theta);
		complex<double> yj(0, y);
		realZ = real((x + yj) * exp(thetaj));
		imagZ = imag((x + yj) * exp(thetaj));
		rotedPoint.lon = realZ;
		rotedPoint.lat = imagZ;
		rotedPoint.alt = 0;
		return true;
	}

	int CSecRptData2::rotAndCompare(IN  list<sampleSectionOverlap_t> sampleSectionOverlap,
		                            IN  vector<sampleSectionBody_t> sampleSectionBody,
									IN  vector<point3D_t> singleRptData,
		                            OUT vector<sampleSectionBody_t> &singleMatchedDataInfo)
	{
		if(!singleMatchedDataInfo.empty())
		{
		   singleMatchedDataInfo.clear();
		}

		list<sampleSectionOverlap_t>::iterator sampleSecOverlapItor = sampleSectionOverlap.begin();

		uint32 bodyIdx = 0;
		//for each section, find its left and right overlap boundary
		while(sampleSecOverlapItor != sampleSectionOverlap.end())
		{
			vector<point3D_t> overlapData = (*sampleSecOverlapItor).sampleOverlapData;

			vector<uint32> parSeg = _parSeg[(*sampleSecOverlapItor).segId-1];
			//parSeg.push_back((*sampleSecOverlapItor).segId);

			int bodyLeftIdx = sampleSectionBody[bodyIdx].startLoc + int(_sampleInterval);
			int bodyRightIdx = sampleSectionBody[bodyIdx].endLoc - int(_sampleInterval);
			point3D_t bodyLeftPt = singleRptData[bodyLeftIdx];
			point3D_t bodyRightPt = singleRptData[bodyRightIdx];

			int dataSize = overlapData.size();
			sampleSectionBody_t aSecMatchData;
			point3D_t srcLeft,srcRight,rotSrcLeft,rotSrcRight;
			double theta;
			int leftIndex,rightIndex; 
			double minDataDistY = MAX_MEAN_SEC_DIST_Y;
			bool reverseFlag = FALSE;

			//compare one data section to several parallel sections
			uint32 matchedSegId;
			for (int i = 0;i<parSeg.size();i++)
			{
				uint32 currSeg = parSeg[i];

				//get the section left and right boundary
				srcLeft = _secCfgInfo[currSeg-1].prePoint_ext;
				srcRight = _secCfgInfo[currSeg-1].nextPoint_ext;
				//get section angle and compare
				theta = -atan2((srcRight.lat - srcLeft.lat),(srcRight.lon - srcLeft.lon ));
				rotPoint(srcLeft,theta,rotSrcLeft);
				rotPoint(srcRight,theta,rotSrcRight);

				//get nearest point index
				double sumDistY = 0;
				point3D_t rotNew;
				vector<double> leftDistX,rightDistX;
				for(int k = 0;k < dataSize;k++)
				{
					rotPoint(overlapData[k],theta,rotNew);
					leftDistX.push_back(abs(rotNew.lon - rotSrcLeft.lon));
					rightDistX.push_back(abs(rotNew.lon - rotSrcRight.lon));
					sumDistY += abs(rotNew.lat - rotSrcRight.lat);
				}

				double dataMeanDistY = sumDistY/dataSize;

				//set the min distance section to current segID
				if ( dataMeanDistY< minDataDistY )
				{
					minDataDistY = dataMeanDistY;

					matchedSegId = currSeg;

					leftIndex = min_element(leftDistX.begin(),leftDistX.end()) - leftDistX.begin();
					rightIndex = min_element(rightDistX.begin(),rightDistX.end()) - rightDistX.begin();
					if (leftIndex > rightIndex)
					{
						int tmpIndex = leftIndex;
						leftIndex = rightIndex;
						rightIndex = tmpIndex;
					}
				}
			}  // end parSeg loop	

			//compute the data angel and section config angle 
			double dataAngleDeg,cfgAngleDeg;
			double dataAngleRad,cfgAngleRad;

			srcLeft = _secCfgInfo[matchedSegId-1].prePoint_ext;
			srcRight = _secCfgInfo[matchedSegId-1].nextPoint_ext; 
			cfgAngleRad = atan2((srcRight.lat - srcLeft.lat),(srcRight.lon - srcLeft.lon ));

			//if this rptData section reversing,0 means the same direction,1 means the different direction
			point3D_t rotDataRight,rotDataLeft;
			rotPoint(bodyLeftPt,-cfgAngleRad,rotDataLeft);
			rotPoint(bodyRightPt,-cfgAngleRad,rotDataRight);
			reverseFlag = ( rotDataRight.lon - rotDataLeft.lon ) * ( rotSrcRight.lon - rotSrcLeft.lon ) > 0 ? FALSE : TRUE;

			if ( reverseFlag == FALSE )
			{
				dataAngleRad = atan2((bodyRightPt.lat - bodyLeftPt.lat),(bodyRightPt.lon - bodyLeftPt.lon ));
			} 
			else
			{
				dataAngleRad = atan2((bodyLeftPt.lat - bodyRightPt.lat),(bodyLeftPt.lon - bodyRightPt.lon ));
			}
			dataAngleDeg = dataAngleRad * 180 / CV_PI;
			cfgAngleDeg = cfgAngleRad * 180 / CV_PI;

			//output the matched section
			if ( minDataDistY != MAX_MEAN_SEC_DIST_Y && abs(dataAngleDeg-cfgAngleDeg) < MAX_INTER_ANGLE_IN_DEGREE )
			{
				// if found the Y min distance section which is smaller than MAX_MEAN_SEC_DIST_Y  
				aSecMatchData.segId = matchedSegId;
				aSecMatchData.startLoc = (*sampleSecOverlapItor).startLoc + leftIndex;
				aSecMatchData.endLoc = (*sampleSecOverlapItor).startLoc + rightIndex;
				aSecMatchData.reverseFlag = reverseFlag;
				singleMatchedDataInfo.push_back(aSecMatchData);
			} 
			else
			{ 
				char warningStr[MAX_PATH];
				sprintf_s(warningStr,"rotAndCompare warning: section %d is too far away from config. Deleted it!\n",(*sampleSecOverlapItor).segId);
				debugPrintf(warningStr);
			}

			sampleSecOverlapItor++;
			bodyIdx++;

		}  //end par seg compare
#if 0
		//decide is there jumping section and merging the jumping section to correct section
		int changeSegNum = MAX_JUMP_SEG_NO;
		int itrNum = 0;
		while ( changeSegNum!=0 )
		{
			if ( itrNum > MAX_JUMP_SEG_NO )
			{
				debugPrintf("rotAndCompare warning: large gps offset!\n ");
				break;
			}
			changeSegNum = mergeJumpSeg(singleMatchedDataInfo, sampleSectionBody, singleRptData);
			itrNum++;
		}

		int mergeNum = MAX_JUMP_SEG_NO;
		while ( mergeNum != 0 )
		{
			mergeNum = 0;
			mergeSameSeg( mergeNum, singleMatchedDataInfo );
		}
#endif
		
#if 1
		if ( singleMatchedDataInfo.empty() )
		{
			return true;
		} 
		else
		{
			//decide the last section is full section or not
			vector<sampleSectionBody_t>::iterator itrBodyData = sampleSectionBody.end()-1;
			vector<sampleSectionBody_t>::iterator itrMatchData = singleMatchedDataInfo.end()-1;
			int lastIdx;
			double matchDist = 0;
			int beginIdx = itrBodyData->startLoc;
			int endIdx = itrBodyData->endLoc;
			for ( lastIdx = beginIdx; lastIdx<endIdx; lastIdx++ )
			{
				matchDist += getLength( singleRptData[lastIdx], singleRptData[lastIdx+1] );
			}
			uint32 lastSegId = singleMatchedDataInfo.back().segId;
			int prePointId = _secCfgInfo[lastSegId-1].prePointId;
			int nextPointId = _secCfgInfo[lastSegId-1].nextPointId;
			point3D_t prevPt = _bodyPts3D[prePointId];
			point3D_t nextPt = _bodyPts3D[nextPointId];
			double segLengh = getLength( prevPt, nextPt );
			if ( matchDist/segLengh < LAST_SEG_TO_CFG_SEG_LENGH_RATIO )
			{
				itrMatchData = singleMatchedDataInfo.erase(itrMatchData);
				char warningStr[MAX_PATH];
				sprintf_s(warningStr,"rotAndCompare warning: delete the last section %d !\n ", lastSegId);
				debugPrintf(warningStr);
			}
		}
#endif
		
#if 0
		if ( singleMatchedDataInfo.empty() )
		{
			return true;
		} 
		else
		{
			//decide the first section is full section or not
			itrMatchData = singleMatchedDataInfo.begin();
			matchDist = 0;
			beginIdx = itrMatchData->startLoc;
			endIdx = itrMatchData->endLoc;
			for ( lastIdx = beginIdx; lastIdx<endIdx; lastIdx++ )
			{
				matchDist += getLength( singleRptData[lastIdx], singleRptData[lastIdx+1] );
			}
			lastSegId = singleMatchedDataInfo.front().segId;
			prePointId = _secCfgInfo[lastSegId-1].prePointId;
			nextPointId = _secCfgInfo[lastSegId-1].nextPointId;
			prevPt = _bodyPts3D[prePointId];
			nextPt = _bodyPts3D[nextPointId];
			segLengh = getLength( prevPt, nextPt );
			prevExtLen = getLength( prevPt, _secCfgInfo[lastSegId-1].prePoint_ext );
			nextExtLen = getLength( nextPt, _secCfgInfo[lastSegId-1].nextPoint_ext );
			segLengh += prevExtLen + prevExtLen;
			if ( matchDist/segLengh < MATCH_SEG_TO_CFG_SEG_LENGH_RATIO )
			{
				itrMatchData = singleMatchedDataInfo.erase(itrMatchData);
				char warningStr[MAX_PATH];
				sprintf_s(warningStr,"rotAndCompare warning: delete the last section %d !\n ", lastSegId);
				debugPrintf(warningStr);
			}
		}
#endif

		return true;
	}  //end rotAndCompare

	void CSecRptData2::mergeSameSeg( INOUT int &mergeNum,
		                             INOUT vector<sampleSectionBody_t> &singleMatchedDataInfo )
	{
		vector<sampleSectionBody_t>::iterator itrMatchData = singleMatchedDataInfo.begin();
		vector<sampleSectionBody_t>::iterator itrSameSegData;
		uint32 curSegId;
		while( itrMatchData != singleMatchedDataInfo.end() )
		{
			curSegId = itrMatchData->segId;
			itrSameSegData = itrMatchData + 1;
			while ( itrSameSegData != singleMatchedDataInfo.end() )
			{
				if ( curSegId == itrSameSegData->segId )
				{
					if ( itrSameSegData - itrMatchData < _secCfgInfo.size()/2 && itrSameSegData->reverseFlag == itrMatchData->reverseFlag )
					{
						itrMatchData->endLoc = itrSameSegData->endLoc;
						itrSameSegData = singleMatchedDataInfo.erase( itrMatchData+1,itrSameSegData+1 );
						mergeNum++;
					} 
				}
				else
				{
			        itrSameSegData++;
				}
			}
			itrMatchData++;
		}
	}

	int CSecRptData2::mergeJumpSeg(vector<sampleSectionBody_t> &singleMatchedDataInfo, vector<sampleSectionBody_t> sampleSectionBody, vector<point3D_t> singleRptData)
	{
		int changeSegNum = 0;
		vector<sampleSectionBody_t>::iterator itrMatchData = singleMatchedDataInfo.begin();
		int Idx = 0;
		while( itrMatchData < (singleMatchedDataInfo.end()-1) )
		{
			Idx++;
			uint32 currSecId = itrMatchData->segId;
			itrMatchData++;
			uint32 nextSecId = itrMatchData->segId;
			uint32 nextEndLoc = itrMatchData->endLoc;
			itrMatchData--;

			if ( nextSecId == currSecId )
			{
				char warningStr[MAX_PATH];
				sprintf_s(warningStr,"rotAndCompare warning: merge seg %d to seg %d !\n ", nextSecId,currSecId);
				debugPrintf(warningStr);

				itrMatchData->endLoc = nextEndLoc;
				itrMatchData = singleMatchedDataInfo.erase(++itrMatchData);
				itrMatchData--;
				//Idx++;
				continue;
			}

			vector<uint32> vecNextSecId,vecPrevSecId,connSecId;
			vecNextSecId = _secCfgInfo[currSecId-1].nextSegId;
			vecPrevSecId =  _secCfgInfo[currSecId-1].prevSegId;
			sort(begin(vecNextSecId), end(vecNextSecId));
			sort(begin(vecPrevSecId), end(vecPrevSecId));
			set_union(begin(vecNextSecId), end(vecNextSecId),begin(vecPrevSecId), end(vecPrevSecId), back_inserter(connSecId));


			if (find(connSecId.begin(), connSecId.end(), nextSecId) == connSecId.end())  
			{
				// nextSecId is not in connSecId

				char warningStr[MAX_PATH];
				sprintf_s(warningStr,"rotAndCompare warning: seg %d jump to seg %d !", currSecId, nextSecId);
				debugPrintf(warningStr);

				//compare with the two sections ,set the minDataDistY section to current section 
				changeSegNum++;
				vector<vector<point3D_t>> bodyData;
				vector<point3D_t> data0,data1;
				if ( 1==changeSegNum )
				{
					int startLoc = sampleSectionBody[Idx-1].startLoc;
					int endLoc = sampleSectionBody[Idx-1].endLoc;
					for (int i = startLoc; i <= endLoc; i++ )
					{
						data0.push_back(singleRptData[i]);
					}
					bodyData.push_back(data0);
				}
				int startLoc = sampleSectionBody[Idx].startLoc;
				int endLoc = sampleSectionBody[Idx].endLoc;
				for (int i = startLoc; i <= endLoc; i++ )
				{
					data1.push_back(singleRptData[i]);
				}
				bodyData.push_back(data1);

				vector<uint32> parSeg;
				parSeg.push_back(currSecId);
				parSeg.push_back(nextSecId);

				point3D_t srcLeft,srcRight,rotSrcLeft,rotNew;
				double theta;
				int dataIdx;
				for ( dataIdx = 0; dataIdx< bodyData.size(); dataIdx++ )
				{
					double minMeanDistY = MAX_MEAN_SEC_DIST_Y;
					vector<point3D_t> curData = bodyData[dataIdx];
					for (int i = 0;i<parSeg.size();i++)
					{
						uint32 currSeg = parSeg[i];

						//get the section left and right boundary
						srcLeft = _bodyPts3D[_secCfgInfo[currSeg-1].prePointId];
						srcRight = _bodyPts3D[_secCfgInfo[currSeg-1].nextPointId];
						//get section angle and compare
						theta = -atan2((srcRight.lat - srcLeft.lat),(srcRight.lon - srcLeft.lon));
						rotPoint(srcLeft,theta,rotSrcLeft);

						//get nearest point index
						double sumDistY = 0, dataMeanDistY = 0.0;
						//vector<double> leftDistX,rightDistX;
						for( int k = 0; k < curData.size(); k++ )
						{
							rotPoint(curData[k],theta,rotNew);
							sumDistY += abs(rotNew.lat - rotSrcLeft.lat);
						}

						if(0 != curData.size())
						{
							dataMeanDistY = sumDistY / curData.size();
						}				 

						if ( dataMeanDistY < minMeanDistY )
						{
							minMeanDistY = dataMeanDistY;

							if ( 1 == changeSegNum && 0 == dataIdx )
							{
								itrMatchData->segId = currSeg;
							}
							else
							{
								(itrMatchData+1)->segId = currSeg;
							}					
						}
					}
				}

				continue;
			}
		    itrMatchData++;
		}	//end while
		return changeSegNum;
	} //end mergeJumpSeg


} /* namespace ns_database */
