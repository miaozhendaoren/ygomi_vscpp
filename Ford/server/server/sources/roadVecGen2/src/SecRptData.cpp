/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  SecRptData.cpp
* @brief This is class implementation file for SecRptData, which extract 
*        sections from report new data according to section configure  
*
* Change Log:
*      Date                Who                   What
*      2015/09/11     Qian Xu,Shili Wang        Create
*******************************************************************************
*/

#include "SecRptData.h"
#include "VisualizationApis.h"
#include "configure.h"

namespace ns_database
{
	#define OVERLAP_SCALE        50
	#define SAMPLE_INTERVAL      20

	void debugPrintf(const string errorPrint)
	{
		#if VISUALIZATION_ON
		printf(errorPrint.c_str());
		#endif
	}

	int getALLBodySectionPoint(IN  list<segAttributes_t> secConfigList,
		                       OUT vector<point2D_t> &bodySectionPoint,
		                       OUT bool &closedLoopFlag)
	{
		if (!bodySectionPoint.empty())
		{
			bodySectionPoint.clear();
		}

		list<segAttributes_t>::iterator itrSecConfig;

		for(itrSecConfig = secConfigList.begin();itrSecConfig != secConfigList.end();itrSecConfig++)
		{
			segAttributes_t tmpSec = *itrSecConfig;
			point2D_t tmpBodySecPoint;
			tmpBodySecPoint.lat = tmpSec.ports[1].lat;
			tmpBodySecPoint.lon = tmpSec.ports[1].lon;
			bodySectionPoint.push_back(tmpBodySecPoint);
		}
		segAttributes_t firstSection,lastSection;
		firstSection = secConfigList.front();
		lastSection = secConfigList.back();

		if ((firstSection.ports[1].lat == lastSection.ports[2].lat)
			&&(firstSection.ports[1].lon == lastSection.ports[2].lon))
		{
			closedLoopFlag = 1;
			return true;
		} 
		else
		{
			closedLoopFlag = 0;
			point2D_t endSecPoint;
			segAttributes_t endSec = secConfigList.back();
			endSecPoint.lat = endSec.ports[2].lat;
			endSecPoint.lon = endSec.ports[2].lon;
			bodySectionPoint.push_back(endSecPoint);
			return true;
		}
	}

	double getLength(IN double point1_x,
		             IN double point1_y, 
		            IN double point2_x, 
		           IN double point2_y)
	{
		double diff_x = abs(point2_x - point1_x);      
		double diff_y = abs(point2_y - point1_y);        
		double length = sqrt(long double(diff_x* diff_x+ diff_y*diff_y));
		return length;
	}

	int getAngle(IN point3D_t samplePoint,
		IN point2D_t sectionPoint,
		IN point2D_t leftPoint,
		IN point2D_t rightPoint,
		OUT double &cos_left,
		OUT double &cos_right)
	{
		double a = getLength(sectionPoint.lon, sectionPoint.lat, samplePoint.lon, samplePoint.lat);
		double bl = getLength(sectionPoint.lon, sectionPoint.lat, leftPoint.lon, leftPoint.lat);      
		double cl = getLength(samplePoint.lon, samplePoint.lat,leftPoint.lon, leftPoint.lat);     
		double br =  getLength(sectionPoint.lon, sectionPoint.lat,rightPoint.lon, rightPoint.lat); 
		double cr =  getLength(samplePoint.lon, samplePoint.lat,rightPoint.lon, rightPoint.lat); 
		cos_left =( a*a+ bl*bl - cl*cl) / (a * bl * 2); 
		cos_right = ( a*a+ br*br - cr*cr) / (a * br * 2);
		return true;
	}

	int getSampleSectionID(IN samplePoint_t samplePointRow,
		                  IN vector<point2D_t> bodySectionPoint,
		                   OUT uint32 &sampleSectionID)
	{
		//get the minDist sectionID 
		sampleSectionID = 0;
		vector<double> distance(bodySectionPoint.size());
		for (int i = 0;i < distance.size();i++)
		{
			distance[i] = getLength(samplePointRow.leftLane.lon,samplePointRow.leftLane.lat,
				                    bodySectionPoint[i].lon,bodySectionPoint[i].lat);
		}
		uint32 min_pos = (uint32) ( min_element(distance.begin(),distance.end()) - distance.begin());

		//get prevSectionID & nextSectionID of the minDist sectionID
		uint32 minDisSectionID = min_pos+1;
		uint32 prevSectionID = minDisSectionID-1;
		uint32 nextSectionID = minDisSectionID+1;
		if (minDisSectionID == 1)
		{
			prevSectionID = bodySectionPoint.size();
		}
		if (minDisSectionID==bodySectionPoint.size())
		{
			nextSectionID = 1;
		}

		//compare two angles
		double cos_left = 1 ,cos_right = 1;

		getAngle(samplePointRow.leftLane,bodySectionPoint[min_pos],bodySectionPoint[prevSectionID-1],
			     bodySectionPoint[nextSectionID-1],cos_left,cos_right);

		if (cos_left < 0 && cos_right > 0)
		{
			sampleSectionID = minDisSectionID;
		} 
		else
		{
			if (cos_left > 0 && cos_right < 0)
			{
				sampleSectionID = prevSectionID;
			} 
			else
			{
				sampleSectionID = prevSectionID;
			}
		}
		return true;
	}

	BOOL getSectionBoundry(IN list<samplePoint_t> samplePoint,
		                   IN uint32 sectionNum,
		                   IN bool closedLoopFlag,
		                   INOUT uint32 &seekStartRow,
		                   OUT sampleSectionBody_t &aSampleBodySection,
		                   OUT BOOL &selectOverFlag)
	{
		//initial aSampleBodySection
		sampleSectionBody_t initSampleBodySection = {0,0,0};

		if ((aSampleBodySection.segId ||aSampleBodySection.startLoc ||aSampleBodySection.endLoc)!=0)
		{
			aSampleBodySection=initSampleBodySection;
		}

		//get the current aSampleBodySection sectionID
		uint32 currtID = 0;

		list<samplePoint_t>::iterator itrSamplePoint = samplePoint.begin();
		if (seekStartRow == 1)
		{
			seekStartRow = 2;
			aSampleBodySection = initSampleBodySection;
			selectOverFlag = 0;
			return true;
		} 
		else
		{
			advance(itrSamplePoint,seekStartRow-1); //get the (seekStartRow)th data
			samplePoint_t currSamplePoint = *itrSamplePoint;
			currtID = currSamplePoint.segId;
		}

		//find the start and end boundary of sample body section
		uint32 sectionStartRow = 0;
		uint32 sectionEndRow = 0;
		uint32 changePointNum = 0;
		uint32 prevSectionID,nextSectionID;
		int i,j;

		for ( i = seekStartRow-1;i >= 1;i--)
		{
			itrSamplePoint = samplePoint.begin();
			advance(itrSamplePoint,i-1); //get the (i)th data
			samplePoint_t tmpSamplePoint = *itrSamplePoint;
			if (tmpSamplePoint.segId != currtID)
			{
				sectionStartRow = tmpSamplePoint.startLoc;
				changePointNum = changePointNum + 1;
				prevSectionID = tmpSamplePoint.segId;
				break;
			}
		}

		for ( j = seekStartRow+1;j <= samplePoint.size();j++)
		{
			itrSamplePoint = samplePoint.begin();
			advance(itrSamplePoint,j-1); //get the (j)th data
			samplePoint_t tmpSamplePoint = *itrSamplePoint;
			if (tmpSamplePoint.segId != currtID)
			{
				sectionEndRow = tmpSamplePoint.startLoc;
				seekStartRow = j;
				changePointNum = changePointNum + 1;
				nextSectionID = tmpSamplePoint.segId;
				break;
			}

			//Is all section found
			if (j == samplePoint.size()) 
			{
				selectOverFlag = true;
			} 
			else
			{
				selectOverFlag = false;
			}

		}

		//output:get section ID
		uint32 sectionID = 0; 
		if ( changePointNum == 2 && prevSectionID != nextSectionID )
		{
			sectionID = currtID;
			aSampleBodySection.startLoc = sectionStartRow;
			aSampleBodySection.endLoc = sectionEndRow;
			aSampleBodySection.segId = sectionID;
		} 
		else
		{
			aSampleBodySection = initSampleBodySection;
			return true;
		}

		//delete the last section data if the source sectionList is a closed loop
		if ( sectionID == sectionNum && closedLoopFlag == 0 )
		{
			aSampleBodySection = initSampleBodySection;
		} 
		return true;
	}

	int rotPoint(IN point3D_t sourcePoint,IN double theta,OUT point3D_t &rotedPoint)
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

	CSecRptData::CSecRptData(void)
	{
	}


	CSecRptData::~CSecRptData(void)
	{
	}

	/******************************
	*get extracted section data out 
	******************************/

	int CSecRptData::segMultiRptData(IN  list<list<vector<point3D_t>>> rptData,
		                             IN  uint32 sampleInterval,
		                             IN  list<segAttributes_t> _segConfigList,
		                             OUT list<reportSectionData> &secData)
	{		
		if(!secData.empty())
		{
			secData.clear();
		}

		list<segAttributes_t> secConfigList;
		getSecConfigList(_segConfigList,secConfigList);

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

			getSingleRptDataInfo(*lanesItor,sampleInterval,secConfigList,singleMatchedDataInfo);
			if(!singleMatchedDataInfo.empty())
			{
				SecDataInfo.push_back(singleMatchedDataInfo);
				isValidData.push_back(count);
			}
			count++;
			rptDataItor++;
		}

		extractSecData(SecDataInfo,rptData,isValidData,secConfigList,secData);

		return true;
	}

	
	//Get section's segID,startRow,endRow of each rptData
	int CSecRptData::getSingleRptDataInfo(IN  vector<point3D_t> singleRptData,
		                                  IN  uint32 sampleInterval,
		                                  IN  list<segAttributes_t> secConfigList,
		                                  OUT vector<sampleSectionBody_t> &singleMatchedDataInfo)
	{
		if(!singleMatchedDataInfo.empty())
		{
			singleMatchedDataInfo.clear();
		}

		list<samplePoint_t> samplePoint;
		bool closedLoopFlag;
		int  configSecNum = secConfigList.size();

		vector<sampleSectionBody_t> sampleSectionBody;
		list<sampleSectionOverlap_t> sampleSectionOverlap;
		vector<sampleSectionBody_t> aMatchNZData;

		/*step1:re_sample reported data from vehicle.*/
		resampleData(singleRptData,sampleInterval,samplePoint);

		if(samplePoint.empty())
		{
			debugPrintf("resampleData:The sample point is empty\n");
			return false;
		}
		/*step2:find section ID for every sample point.*/
		findSamplePointSecID(secConfigList,samplePoint,closedLoopFlag);

		/*step3:get every coarse section body Info of one rptData.*/
		getSecBodyInfo(samplePoint,closedLoopFlag,configSecNum,sampleSectionBody);

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
		rotAndCompare(sampleSectionOverlap,secConfigList,singleMatchedDataInfo);
		return true;
	}

	int CSecRptData::extractSecData(IN list<vector<sampleSectionBody_t>> SecDataInfo,
		                            IN list<list<vector<point3D_t>>> rptData,
		                            IN vector<int> isValidData,
		                            IN list<segAttributes_t> secConfigList,
		                            OUT list<reportSectionData> &secData)
	{
		list<vector<point3D_t>> ::iterator itr;
		vector<point3D_t> ::iterator leftItor,rightItor;
		list<segAttributes_t> ::iterator secConfigItor = secConfigList.begin();

		int startRow = 0;
		int endRow = 0;

        // iterate all sections to get reported section data
		for(int sectionIndex = 1;sectionIndex <= secConfigList.size();sectionIndex++)
		{
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
					if((*videoNumItor)[index].segId == sectionIndex)
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

						/*if(((aSecLeftLaneData)[0].lon - (aSecLeftLaneData)[(aSecLeftLaneData).size()-1].lon)*((*secConfigItor).ports[0].lon - (*secConfigItor).ports[3].lon) < 0)
						{
							vector<point3D_t> tempaSecLaneData;
							tempaSecLaneData = aSecLeftLaneData;
							aSecLeftLaneData = aSecRightLaneData;
							aSecRightLaneData = tempaSecLaneData;
							reverse(aSecLeftLaneData.begin(),aSecLeftLaneData.end());
							reverse(aSecRightLaneData.begin(),aSecRightLaneData.end());
						}*/

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
#if (RD_LOCATION == RD_GERMAN_MUNICH_AIRPORT)
                            if (sectionIndex == 2)
                            {
                                if((*aSecLeftLaneItor).paintFlag == 2 || (*aSecRightLaneItor).paintFlag == 2)
                                {
                                    aSecLeftLaneData.clear();
                                    aSecRightLaneData.clear();
                                    break;
                                }else
                                {
                                	aSecLeftLaneItor++;
								    aSecRightLaneItor++;
                                }
                            }else
#endif
                            {
							    if((*aSecLeftLaneItor).paintFlag == 2 || (*aSecRightLaneItor).paintFlag == 2)
							    {
								    aSecLeftLaneItor = aSecLeftLaneData.erase(aSecLeftLaneItor);
								    aSecRightLaneItor = aSecRightLaneData.erase(aSecRightLaneItor);
							    }
							    else
							    {
								    aSecLeftLaneItor++;
								    aSecRightLaneItor++;
							    }
                            }
						}

                        rptSeg.revDirFlag = (*videoNumItor)[index].reverseFlag;
                        rptSeg.rptLaneData.push_back(aSecLeftLaneData);
                        rptSeg.rptLaneData.push_back(aSecRightLaneData);

                        middleData.sectionId = sectionIndex;
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

	BOOL CSecRptData::getSecConfigList(IN  list<segAttributes_t> _segConfigList,
		                               OUT list<segAttributes_t> &secConfigList)
	{
		if(!secConfigList.empty())
		{
			secConfigList.clear();
		}
		secConfigList = _segConfigList;
		list<segAttributes_t>::iterator oriSegItor;
		list<segAttributes_t>::iterator secConItor = secConfigList.begin();
		for( oriSegItor = _segConfigList.begin();oriSegItor != _segConfigList.end();oriSegItor++)
		{
			(*secConItor).ports[0] = (*oriSegItor).ports[2];
			(*secConItor).ports[1] = (*oriSegItor).ports[0];
			(*secConItor).ports[2] = (*oriSegItor).ports[1];
			(*secConItor).ports[3] = (*oriSegItor).ports[3];
			secConItor++;
		}
		return true;
	}

	int CSecRptData::resampleData(IN  vector<point3D_t> singleRptData,
		                          IN  uint32 sampleInterval,
	                              OUT list<samplePoint_t> &samplePoint)
	{
		if (!samplePoint.empty())
		{
			samplePoint.clear();
		}

		int videoLengh = singleRptData.size();
		samplePoint_t endSamplePoint;
		int pointNum;
		list<samplePoint_t>::iterator itrSamplePoint;

		if(sampleInterval == 0)
		{
			debugPrintf("resampleData:the sampleInterval is invalid! It should be larger than zero\n");
			return false;
		}
		else if (sampleInterval>videoLengh)
		{
			debugPrintf("resampleData:the sampleInterval is biger than the input video!; Please decrease it!\n");
			return false;
		}
		else
		{
			pointNum = (int)(ceil(double(videoLengh)/double(sampleInterval)));
		}

		for (int i = 1;i < pointNum;i++)
		{
			samplePoint_t currSamplePoint;
			currSamplePoint.segId = 0;
			currSamplePoint.startLoc = (i-1)*sampleInterval;
			currSamplePoint.leftLane.lon = singleRptData[(i-1)*sampleInterval].lon;
			currSamplePoint.leftLane.lat = singleRptData[(i-1)*sampleInterval].lat;

			samplePoint.push_back(currSamplePoint);
		}
		endSamplePoint.segId = 0;
		endSamplePoint.startLoc = videoLengh-1;
		endSamplePoint.leftLane.lat = singleRptData[videoLengh-1].lat;
		endSamplePoint.leftLane.lon = singleRptData[videoLengh-1].lon;
		samplePoint.push_back(endSamplePoint);

		itrSamplePoint = samplePoint.begin();
		while ( itrSamplePoint != samplePoint.end())
		{
			if (itrSamplePoint->leftLane.lat == 0 || itrSamplePoint->leftLane.lon == 0)
			{
				debugPrintf("resampleData warning: this repData has zero longitude or latitude in left lane.The results may not be very accuracy!\n");
				samplePoint.clear();
				/*itrSamplePoint->leftLane.lat = (--itrSamplePoint)->leftLane.lat;
				itrSamplePoint->leftLane.lon = itrSamplePoint->leftLane.lon;
				itrSamplePoint++;*/
				return false;
				break;
			} 
			itrSamplePoint++;
		}

		return true;
	}

	void CSecRptData::findSamplePointSecID(IN  list<segAttributes_t> secConfigList,
										  INOUT list<samplePoint_t> &samplePoint,
		                                  OUT bool &closedLoopFlag)
	{

		vector<point2D_t> bodySectionPoint;
		list<samplePoint_t>::iterator itrSamplePoint;
		itrSamplePoint = samplePoint.begin();

		getALLBodySectionPoint(secConfigList,bodySectionPoint,closedLoopFlag);

		while (itrSamplePoint != samplePoint.end())
		{
			samplePoint_t samplePointRow = *itrSamplePoint;
			uint32 sampleSectionID = 0;
			getSampleSectionID( samplePointRow, bodySectionPoint,sampleSectionID);
			(*itrSamplePoint).segId = sampleSectionID;
			itrSamplePoint++;
		}
	}

	int CSecRptData::getSecBodyInfo(IN  list<samplePoint_t> samplePoint,
		                            IN  bool closedLoopFlag,
		                            IN  int  configSecNum,
		                            OUT vector<sampleSectionBody_t> &sampleSectionBody)
	{
		if (!sampleSectionBody.empty())
		{
			sampleSectionBody.clear();
		}

		uint32 sectionNum;
		uint32 changeSectionSum = 0;
		uint32 seekStartRow = 1;
		uint32 SampleBodySectionNum = 0;
		int count = samplePoint.size();
		list<samplePoint_t>::iterator itrSamplePoint = samplePoint.begin();
		
		if (closedLoopFlag == 1)
		{
			sectionNum = configSecNum;
		} 
		else
		{
			sectionNum = configSecNum + 1;
		}

		for (int i = 1;i < samplePoint.size();i++)
		{
			uint32 currID = (*itrSamplePoint).segId;
			uint32 nextID = (*(++itrSamplePoint)).segId;
			uint32 currIDDiff = abs((long)nextID-(long)currID);
			if ( currIDDiff > 1 && currIDDiff < sectionNum-1 )
			{
				debugPrintf("getSecBodyInfo:the sampleInterval is biger than the minimum section distance!; Please decrease it and retry!\n");
				return -1;
			}

			if ( currIDDiff == sectionNum-1 )
			{
				currIDDiff = 1;
			}
			changeSectionSum = changeSectionSum + currIDDiff;
		}

		//Is input video is too short to section
		if (changeSectionSum < 2)
		{
			debugPrintf("getSecBodyInfo:this video is too short.It has not a matched data out!\n");
			return -2;
		}

		//merge sample point to body section
		while(count--)
		{
			sampleSectionBody_t aSampleBodySection;
			BOOL selectOverFlag = FALSE;

			getSectionBoundry(samplePoint,sectionNum,closedLoopFlag,seekStartRow,aSampleBodySection,selectOverFlag );

			if ( aSampleBodySection.segId != 0)
			{
				sampleSectionBody.push_back(aSampleBodySection);
			}
			if (selectOverFlag == 1)
			{
				break;
			}
		}
		return true;
	}

	int CSecRptData::getSecOverlap(IN  vector<sampleSectionBody_t> sampleSectionBody,
		                           IN  vector<point3D_t> singleRptData,
		                           OUT list<sampleSectionOverlap_t> &sampleSectionOverlap)
	{
		if(!sampleSectionOverlap.empty())
		{
			sampleSectionOverlap.clear();
		}
		
		uint32 overlapLeftIndex = 0;
		uint32 overlapRightIndex = 0;
		uint32 dataSectionNum = sampleSectionBody.size();
		list<vector<point3D_t>>::iterator itrVideoData;

		for (int i = 0;i < dataSectionNum;i++)
		{
			//get segID
			sampleSectionOverlap_t tmpSectionOverlap;
			tmpSectionOverlap.segId = sampleSectionBody[i].segId;

			//get the leftLine.x & leftLine.y of the sample body section boundary 
			point3D_t bodyLeft,bodyRight;

			uint32 leftShift = sampleSectionBody[i].startLoc;
			bodyLeft = singleRptData[leftShift];

			uint32 rightShift = sampleSectionBody[i].endLoc;
			bodyRight = singleRptData[rightShift];

			//get sample overlap boundary
			int leftIndex,rightIndex;

			for ( leftIndex = leftShift;leftIndex >= 0;leftIndex--)
			{
				point3D_t leftData = singleRptData[leftIndex];

				double leftDistance = getLength( bodyLeft.lon, bodyLeft.lat,leftData.lon, leftData.lat );
				overlapLeftIndex = 1;
				if ( leftDistance >= OVERLAP_SCALE )
				{
					overlapLeftIndex = leftIndex;
					break;
				}
			}

			for ( rightIndex = rightShift;rightIndex < singleRptData.size();rightIndex++)
			{
				point3D_t rightData = singleRptData[rightIndex];

				double rightDistance = getLength( bodyRight.lon, bodyRight.lat, rightData.lon,rightData.lat );
				overlapRightIndex = singleRptData.size();
				if ( rightDistance >= OVERLAP_SCALE )
				{
					overlapRightIndex = rightIndex;
					break;
				}
			}

			//vector<point3D_t>  tmpOverlapData;
			vector<point3D_t> cuttedMiddleLane;
			for ( uint32 k = overlapLeftIndex;k < overlapRightIndex;k++)
			{
				point3D_t cutLaneRow = singleRptData[k];
				cuttedMiddleLane.push_back(cutLaneRow);
			}
			tmpSectionOverlap.sampleOverlapData = cuttedMiddleLane;
			cuttedMiddleLane.clear();

			tmpSectionOverlap.startLoc = overlapLeftIndex;
			tmpSectionOverlap.endLoc = overlapRightIndex;

			sampleSectionOverlap.push_back(tmpSectionOverlap);
		}
		return true;
	}

	int CSecRptData::rotAndCompare(IN  list<sampleSectionOverlap_t> sampleSectionOverlap,
		                           IN  list<segAttributes_t> secConfigList,
		                           OUT vector<sampleSectionBody_t> &singleMatchedDataInfo)
	{
		if(!singleMatchedDataInfo.empty())
		{
		   singleMatchedDataInfo.clear();
		}

		list<sampleSectionOverlap_t>::iterator sampleSecOverlapItor = sampleSectionOverlap.begin();
		
		while(sampleSecOverlapItor != sampleSectionOverlap.end())
		{
			vector<point3D_t> CompareSection = (*sampleSecOverlapItor).sampleOverlapData;
			point3D_t srcLeft,srcRight,rotSrcLeft,rotSrcRight;
			list<segAttributes_t>::iterator segConItor,tmpIndex;
			vector<double> LeftDistMat,RightDistMat;
			sampleSectionBody_t aSecMatchData;
			int LeftIndex,RightIndex;
			point3D_t rotNew;
			double theta;

			for(segConItor = secConfigList.begin();segConItor != secConfigList.end();segConItor++)
			{
				if((*sampleSecOverlapItor).segId == (*segConItor).segId)
				{
					tmpIndex = segConItor;
					break;
				}
			}
			srcLeft = (*tmpIndex).ports[0];
			srcRight = (*tmpIndex).ports[3];

			//get section angle and compare
			theta = -atan((srcRight.lat - srcLeft.lat)/(srcRight.lon - srcLeft.lon + 0.0000000001));
			rotPoint(srcLeft,theta,rotSrcLeft);
			rotPoint(srcRight,theta,rotSrcRight);

			//get nearest point index
			point3D_t rotDataRight,rotDataLeft;
			for(int i = 0;i < CompareSection.size();i++)
			{
				rotPoint(CompareSection[i],theta,rotNew);
				LeftDistMat.push_back(abs(rotNew.lon - rotSrcLeft.lon));
				RightDistMat.push_back(abs(rotNew.lon - rotSrcRight.lon));
				if ( i == 0 )
				{
					rotDataLeft = rotNew;
				}
				if ( i==(CompareSection.size()-1) )
				{
					rotDataRight = rotNew;
				}
			}

			LeftIndex = min_element(LeftDistMat.begin(),LeftDistMat.end()) - LeftDistMat.begin();
			RightIndex = min_element(RightDistMat.begin(),RightDistMat.end()) - RightDistMat.begin();

			if (LeftIndex > RightIndex)
			{
				int tmpIndex = LeftIndex;
				LeftIndex = RightIndex;
				RightIndex = tmpIndex;
			}

			//is this rptData section reversing
			bool tempReverseFlag = FALSE;
			tempReverseFlag = ( rotDataRight.lon - rotDataLeft.lon ) * ( rotSrcRight.lon - rotSrcLeft.lon ) > 0 ? FALSE : TRUE;

			aSecMatchData.segId = (*sampleSecOverlapItor).segId;
			aSecMatchData.startLoc = (*sampleSecOverlapItor).startLoc + LeftIndex;
			aSecMatchData.endLoc = (*sampleSecOverlapItor).startLoc + RightIndex;
			aSecMatchData.reverseFlag = tempReverseFlag;
			singleMatchedDataInfo.push_back(aSecMatchData);
			sampleSecOverlapItor++;
		}
		return true;
	}
	
} /* namespace ns_database */