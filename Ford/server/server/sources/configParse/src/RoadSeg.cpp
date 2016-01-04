
#include <string>
#include <complex>
#include <vector>
#include "RoadSeg.h"
#include "XMLParser.h"
#include <math.h>

using std::vector;
namespace ns_roadsegment
{
    Segmentpoint::Segmentpoint()
    {
        pointId = 0;
        point_type = 0;
        conn_segId.clear();
        memset(&point,0,sizeof(point3D_t));
        memset(&point_extern,0,sizeof(point3D_t));
    }
    Segmentlane::Segmentlane()
    {
        laneId = 0;		  
		derection = 0;
		merge_left = 0;
		merge_right = 0;
        start_pos_ID = 0;
        end_pos_ID = 0;
        leftlineID = 0;
		rightlineID = 0; 
    }
    Segmentline::Segmentline()
    {
		lineId=0;
		linetype=0;
		connSegNumofLine=0;
		line_conn.clear();
	}
    RoadSegment::RoadSegment()
    {
        segId=0;
        segType=NORMAL_E;
        laneNum=0;
		lineNum=0;
		pointNum=0;
        _segpoint.clear();
        _seglane.clear();
        _segline.clear();
	}    
    uint8 RoadSegment::getLaneNum()
	{
	    return laneNum;
	}
    uint32 RoadSegment::insertLane(Segmentlane* pinsert)
    {
        uint32 idex = 0;
        while(idex < _seglane.size())
        {
            if(_seglane[idex].laneId > pinsert->laneId)
                return idex;
            idex++; 
        }
        return idex;
    }
    uint32 RoadSegment::insertLine(Segmentline* pinsert)
    {
        uint32 idex = 0;
        while(idex < _segline.size())
        {
            if(_segline[idex].lineId > pinsert->lineId)
                return idex;
            idex++; 
        }
        return idex;
    }
    uint32 RoadSegment::insertPoint(Segmentpoint* pinsert)
    {
        uint32 idex = 0;
        while(idex < _segpoint.size())
        {
            if(_segpoint[idex].pointId > pinsert->pointId)
                return idex;
            idex++; 
        }
        return idex;
    }
    bool RoadSegment::CheckLaneId()
    {
        if(_seglane.empty() || _seglane[0].laneId != 1)
        {
            return false;
        }

        if(_seglane.size() != _seglane[_seglane.size()-1].laneId - _seglane[0].laneId + 1)
            return false;
        else
            return true;
    }
    bool RoadSegment::CheckLineId()
    {
        if(_segline.empty() || _segline[0].lineId != 1)
        {
            return false;
        }

        if(_segline.size() != _segline[_segline.size()-1].lineId - _segline[0].lineId + 1)
            return false;
        else
            return true;
    }

    bool RoadSegment::CheckPointId()
    {
        if(_segpoint.empty() || _segpoint[0].pointId != 1)
        {
            return false;
        }

        if(_segpoint.size() != _segpoint[_segpoint.size()-1].pointId - _segpoint[0].pointId + 1)
            return false;
        else
            return true;
    }

    int All_RoadSegment::getSegNum()
    {
        if(_segmentnum < 0)
        {return -1;}
        return _segmentnum;        
    }

	//just two points in one segment
	bool All_RoadSegment::getSegRangPoint(uint32 SegId, vector<point3D_t> &point)
	{
		int index = getSegmentIndex(SegId);
        if(index < 0)return false;

        point.clear();
		//pre
		int pointindex=0;
		while(pointindex < _roadsegment[index].pointNum)
		{
			if(STARTPOINT == _roadsegment[index]._segpoint[pointindex].point_type)
			{
     	        point.push_back(_roadsegment[index]._segpoint[pointindex].point_extern);
     		    point.push_back(_roadsegment[index]._segpoint[pointindex].point);
			}
            pointindex++;
     	}
		
		//next
		pointindex=0;
		while(pointindex < _roadsegment[index].pointNum)
		{
		    if(ENDPOINT == _roadsegment[index]._segpoint[pointindex].point_type)
			{
		        point.push_back(_roadsegment[index]._segpoint[pointindex].point);
		        point.push_back(_roadsegment[index]._segpoint[pointindex].point_extern);
		    }
		}
		return true;
	}
	bool All_RoadSegment::getAllSegRangPointOrder(list<vector<point3D_t>> &points_all)
	{
        if(_segmentnum<=0)
        {
            return false;
        }

        uint32 index=0;
        points_all.clear();
        while(index<_segmentnum)
        {
          vector<point3D_t> pointsofSeg;
          pointsofSeg.clear();
          //pre
          int pointindex=0;
          while(pointindex < _roadsegment[index].pointNum)
          {
              if(STARTPOINT == _roadsegment[index]._segpoint[pointindex].point_type)
              {
                  pointsofSeg.push_back(_roadsegment[index]._segpoint[pointindex].point_extern);
                  pointsofSeg.push_back(_roadsegment[index]._segpoint[pointindex].point);
              }
              pointindex++;
          }
          //next
          pointindex=0;
          while(pointindex < _roadsegment[index].pointNum)
          {
              if(ENDPOINT == _roadsegment[index]._segpoint[pointindex].point_type)
              {
                  pointsofSeg.push_back(_roadsegment[index]._segpoint[pointindex].point);
                  pointsofSeg.push_back(_roadsegment[index]._segpoint[pointindex].point_extern);
              }
			  pointindex++;
          }
          points_all.push_back(pointsofSeg);
		  index++;
        }
            
        return true;
    }
	bool All_RoadSegment::getAllSegIdOrder(vector<int> &seg)
	{
	    if(_segmentnum <= 0) return false;

        seg.clear();
        for(uint32 i=0;i <_segmentnum;i++)
        {
            seg.push_back(_roadsegment[i].segId);
        }

		return true;
	}
	uint32 All_RoadSegment::getStartSegID()
	{
	    return _roadsegment[0].segId;
	}
	uint32 All_RoadSegment::getSegmentID(uint32 index)
	{
	    return _roadsegment[index].segId;
	}
	int32 All_RoadSegment::getSegmentIndex(uint32 SegId)
	{
	    uint32 firstSegId = _roadsegment[0].segId;
		uint32 LastSegId = _roadsegment[_segmentnum-1].segId;
		if((SegId < firstSegId)||(SegId >LastSegId)) 
		{
		    return -1;
		}

		return (SegId-firstSegId);
	}
	int32 All_RoadSegment::getLaneNumInSeg(uint32 SegId)
    {
        int index = getSegmentIndex(SegId);
		if(index>=0)
        return _roadsegment[index].getLaneNum();
        else
		return -1;	
    }	
	int32 All_RoadSegment::getLaneNumInSeg(uint32 SegId,bool revDirect)
	{
		int index = getSegmentIndex(SegId);
        if(index < 0)return -1;
		
		int32 Lanenum=0;
		int32 j =_roadsegment[index].laneNum;
		while(j)
		{
		    if(revDirect == _roadsegment[index]._seglane[j-1].derection)
		    {Lanenum++;}
			j--;
		}

		return Lanenum;
	}

	bool All_RoadSegment::getLineIdOfCurrentLane(int segId, int laneIndex, bool revDirect, int &lineL, int &lineR)
	{
		int index = getSegmentIndex(segId);
		if(index < 0)return false;

		int cnt=0;
		for(int i=0;i<_roadsegment[index].laneNum;i++)
		{
			if((revDirect == _roadsegment[index]._seglane[i].derection))
			{
				if(laneIndex == cnt)
				{
					lineL = _roadsegment[index]._seglane[i].leftlineID;
					lineR = _roadsegment[index]._seglane[i].rightlineID;
					return true;
				}
				cnt++;
			}
		}
		return false;
	}

    bool All_RoadSegment::getLineTypeOfCurrentLane(int segId, int laneIndex, bool revDirect, int &typeL, int &typeR)
    {
        int lineIdL = 0, lineIdR = 0;
        if (true == getLineIdOfCurrentLane(segId, laneIndex, revDirect, lineIdL, lineIdR))
        {
            int index = getSegmentIndex(segId);

            typeL = _roadsegment[index]._segline[lineIdL - 1].linetype;
            typeR = _roadsegment[index]._segline[lineIdR - 1].linetype;

            return true;
        }
        return false;
    }

	bool All_RoadSegment::getMatchedLaneIdx(int SegId, int  LlineId, int  RlineId, bool revDirect, int &laneIndex)
	{
		int index = getSegmentIndex(SegId);
		if(index < 0)return false;

		int cnt=0;
		for(int i=0;i<_roadsegment[index].laneNum;i++)
		{
			if((revDirect == _roadsegment[index]._seglane[i].derection))
			{
				if((LlineId == _roadsegment[index]._seglane[i].leftlineID)
					&&(RlineId == _roadsegment[index]._seglane[i].rightlineID))
				{
					laneIndex = cnt;
					return true;
				}

				if((RlineId == _roadsegment[index]._seglane[i].leftlineID)
					&&(LlineId == _roadsegment[index]._seglane[i].rightlineID))
				{
					laneIndex = cnt;
					return true;
				}
				cnt++;
			}
		}

		return false;
	}
	bool All_RoadSegment::getMatchedLaneIdx(int SegId, LINETYPE_MIX currentLaneType, bool revDirect, vector<int>&laneIndex)
    {
        int index = getSegmentIndex(SegId);
        if(index < 0)return false;

        laneIndex.clear();

        int lineLType = (((currentLaneType+3)%4)&0x2) >> 1;
		int lineRType = (currentLaneType & 0x2)>>1;
        int llineId;
		int rlineId;
		bool direction;
		int cnt=0;
		for(int i=0;i<_roadsegment[index].laneNum;i++)
		{
		    direction=_roadsegment[index]._seglane[i].derection;
			llineId = _roadsegment[index]._seglane[i].leftlineID;
			rlineId = _roadsegment[index]._seglane[i].rightlineID;

			if((direction==revDirect))
			{
				if((lineLType == _roadsegment[index]._segline[llineId-1].linetype)
				&&(lineRType == _roadsegment[index]._segline[rlineId-1].linetype))
			    {
					laneIndex.push_back(cnt);
				}
				cnt++;
			}
		}

		if(laneIndex.empty())
		{
			if(1 == cnt)
			{
				laneIndex.push_back(0);
				return true;
			}
			return false;
		}
		else
		{return true;}
	}

	bool All_RoadSegment::getExcatMatchedMainLaneIdx(int SegId, LINETYPE_MIX currentLaneType, bool revDirect, vector<int>&laneIndex)
	{
		int index = getSegmentIndex(SegId);
		if(index < 0)return false;

		laneIndex.clear();

		int lineLType = currentLaneType >> LEFT_LINE_TYPE_SHIFT;
		int lineRType = currentLaneType & 0xF;	
		if(INVALID == lineLType || INVALID == lineRType)
		{
			return false;
		}

		int llineId;
		int rlineId;
		bool direction;
		int cnt=0;
		for(int i=0;i<_roadsegment[index].laneNum;i++)
		{
			direction=_roadsegment[index]._seglane[i].derection;
			llineId = _roadsegment[index]._seglane[i].leftlineID;
			rlineId = _roadsegment[index]._seglane[i].rightlineID;

			if((direction==revDirect))
			{
				if((lineLType == _roadsegment[index]._segline[llineId-1].linetype)
					&&(lineRType == _roadsegment[index]._segline[rlineId-1].linetype))
				{
					laneIndex.push_back(cnt);
				}
				cnt++;
			}
		}

		if(laneIndex.empty())
		{
			if(1 == cnt)
			{
				laneIndex.push_back(0);
				return true;
			}
			return false;
		}
		else
		{return true;}
	}
	bool All_RoadSegment::getExcatMatchedMainLaneAndSideLaneIdx(int SegId, LINETYPE_MIX currentLaneType, LINETYPE_MIX neigbourLaneType, bool revDirect, vector<int>&laneIndex)
	{
		int index = getSegmentIndex(SegId);
		if(index < 0)return false;

		laneIndex.clear();

		if(INVALID_INVALID == currentLaneType || INVALID_INVALID == neigbourLaneType)
		{
			return false; 
		}

		int lineLType = currentLaneType >> LEFT_LINE_TYPE_SHIFT;
		int lineRType = currentLaneType & 0xF;
		int lineLType_N = neigbourLaneType >> LEFT_LINE_TYPE_SHIFT;
		int lineRType_N = neigbourLaneType & 0xF;
		int llineId;
		int llineId_N;
		int rlineId;
		int rlineId_N;
		bool direction;				
		int directLaneNum = getLaneNumInSeg(SegId, revDirect);
		int cnt=0;
		for(int i=1;i<_roadsegment[index].laneNum-1;i++)
		{
			direction=_roadsegment[index]._seglane[i].derection;
			llineId = _roadsegment[index]._seglane[i].leftlineID;
			rlineId = _roadsegment[index]._seglane[i].rightlineID;
			llineId_N = _roadsegment[index]._seglane[i].leftlineID-1;
			rlineId_N = _roadsegment[index]._seglane[i].rightlineID+1;

			if(direction==revDirect && ((cnt+1) < directLaneNum - 1))
			{
				cnt++;
				if(0 == llineId_N || 0 == rlineId_N)
				{
					continue;
				}

				if((lineLType == _roadsegment[index]._segline[llineId-1].linetype)
				&&(lineRType == _roadsegment[index]._segline[rlineId-1].linetype)
				&&(INVALID == lineLType_N || lineLType_N == _roadsegment[index]._segline[llineId_N-1].linetype)
				&&(INVALID == lineRType_N || lineRType_N == _roadsegment[index]._segline[rlineId_N-1].linetype))
				{
					laneIndex.push_back(cnt);
				}
			}
		}

		if(laneIndex.empty())
		{return false;}
		else
		{return true;}
	}

	bool All_RoadSegment::getMatchedLaneIdx(int SegId, LINETYPE_MIX currentLaneType, LINETYPE_MIX neigbourLaneType, bool revDirect, vector<int>&laneIndex)
	{
		int index = getSegmentIndex(SegId);
		if(index < 0)return false;

		laneIndex.clear();

		if(false == getExcatMatchedMainLaneIdx(SegId, currentLaneType, revDirect, laneIndex))
		{
			laneIndex.clear();
			return false;
		}

		if(1 == laneIndex.size())
		{
			return true;
		}
		else
		{
			if(true == getExcatMatchedMainLaneAndSideLaneIdx(SegId, currentLaneType, neigbourLaneType, revDirect, laneIndex)
				&& 1 == laneIndex.size())
			{
				return true;
			}
			laneIndex.clear();
			return false;
		}
	}

	bool All_RoadSegment::getBothSideMergeFlag(int SegId)
	{
		int index = getSegmentIndex(SegId);
        if(index < 0)return false;

	    int Max_lane_num = _roadsegment[index].laneNum;
	    int laneIdx=1;
		bool direct;
		uint32 rightStatus;
		if(Max_lane_num > 0)
		{
			direct = _roadsegment[index]._seglane[0].derection;
			rightStatus = _roadsegment[index]._seglane[0].merge_right;
		}
		else
		{
			return false;
		}

	    while(laneIdx < Max_lane_num)
	    {
			if(direct != _roadsegment[index]._seglane[laneIdx].derection)
			{
				if(_roadsegment[index]._seglane[laneIdx].merge_left != rightStatus)
				{
					printf("configure xml setionId = %d, lane merge error!\n",SegId);
					return false;
				}
				if(MERGE_NOT == _roadsegment[index]._seglane[laneIdx].merge_left)
				{
					return false;
				}else
				{
					return true;
				}
			}
			direct = _roadsegment[index]._seglane[laneIdx].derection;
			rightStatus = _roadsegment[index]._seglane[laneIdx].merge_right;
	        laneIdx++;
	    }
		
		return false;
	}

	uint32 All_RoadSegment::insertSegment(RoadSegment* pinsert)
    {
        uint32 idex = 0;
        while(idex < _roadsegment.size())
        {
            if(_roadsegment[idex].segId > pinsert->segId)
                return idex;
            idex++; 
        }
        return idex;
    }
	bool All_RoadSegment::CheckSegmentNum(uint32 couter)
	{
	   if(_segmentnum == couter)
	       return true;
	   else
	       return false;
	}
    bool All_RoadSegment::CheckSegmentID()
    {
        if(_roadsegment.size() != _segmentnum)
        {
            printf("Segment number error!/n");
            return false;
		}

		int numOfRoadSeg = _roadsegment.size();
        for(uint32 i=0;i<numOfRoadSeg-1;i++)
        {
            if(_roadsegment[i].segId > MAX_NUM_SEGMENT)
            {
                printf("Segment Id:%d beyond the range!/n",_roadsegment[i].segId);
                return false;
			}
            if(_roadsegment[i].segId >= _roadsegment[i+1].segId)
            {
                printf("SegId:%d , not orderd by incresed!",_roadsegment[i].segId);
                return false;
			}
        }
		return true;
    }
    uint32 All_RoadSegment::getgpsnum(uint32 index)
    {
        return (_roadsegment[index].pointNum);
    }
    point3D_t* All_RoadSegment::getgps(uint32 segid,uint8 gpsid)
    {
        return &(_roadsegment[segid]._segpoint[gpsid].point);
    }

	void All_RoadSegment::coordinateChange(point3D_t in, point3D_t ref, point3D_t &out)
	{
		double dif_x = in.lat - ref.lat;
		double dif_y = in.lon - ref.lon;
		double latitude = (ref.lat)*PI/180;
	
		out.lat = dif_x*COEFF_DD2METER; //latitude
		out.lon = dif_y*(111413*cos(latitude)-94*cos(3*latitude)); //longitude
	}
	
	void All_RoadSegment::AllSecPointConvert(point3D_t &point)
	{
	    //init reference point
		point3D_t GPSref;
        GPSref.lat = _reference_point.lat;
        GPSref.lon = _reference_point.lon;
		GPSref.alt = _reference_point.alt;
        GPSref.count=0;
        GPSref.paintFlag = 0;
        GPSref.paintLength = 0;
	    
        coordinateChange(point,GPSref, point);		
	    
	}

	void All_RoadSegment::CalcExtendPoint()
	{
        uint32 index = 0;
		vector<double> length_v;
		vector<double> cosAlpha_v;
		vector<double> sinAlpha_v;
		//record segment length : point 0->pre , point 1->next
		while(index < _segmentnum)
		{
		    double lon1 = _roadsegment[index]._segpoint[0].point.lon;
			double lon2 = _roadsegment[index]._segpoint[1].point.lon;
			double lat1 = _roadsegment[index]._segpoint[0].point.lat;
			double lat2 = _roadsegment[index]._segpoint[1].point.lat;
			double len = sqrt((lat2-lat1)*(lat2-lat1)+(lon2-lon1)*(lon2-lon1));

		    cosAlpha_v.push_back(fabs((lon2-lon1)/len));
			sinAlpha_v.push_back(fabs((lat2-lat1)/len));

            for(int i=0;i < 2;i++)
            {
				double max_lenth;
				if(0 == _roadsegment[index]._segpoint[i].point_type_extern_stort)
				{
					max_lenth = MAX_SECTION_EXT_LENTH;
				}
				else
				{
					max_lenth = MAX_SECTION_EXT_LENTH_SHORT;
				}
				
				if(len*LEN_RATE > max_lenth)
				{
					len = max_lenth;
				}
				else
				{
					len = len*LEN_RATE;
				}
                length_v.push_back(len);
			}
			
			index++;
		}

		//calc extern point
		index = 0;
		while(index < _segmentnum)
		{
		    double cosAlpha = cosAlpha_v[index];
			double sinAlpha = sinAlpha_v[index]; 
		    for(int i=0;i<2;i++)
		    {
		        // no or mul segment connection , there is no externs points
		        int pointIndex = _roadsegment[index]._segpoint[i].pointId;
		        if((2 == _secPointInfo[pointIndex].connSecs.size())
				    && (1 == _roadsegment[index]._segpoint[i].conn_segNum) 
				    && (0 == _roadsegment[index]._segpoint[i].point_type_extern))
		        {
                    int conSegIndex = _roadsegment[index]._segpoint[i].conn_segId[0] - 1;
		            
		            double len_neighbor = length_v[conSegIndex*2+i];
					double lon = _roadsegment[index]._segpoint[i].point.lon;
			        double lat = _roadsegment[index]._segpoint[i].point.lat;
					/**e_p1----p1-----p2---e_p2**/
					double lon1 = lon+len_neighbor*cosAlpha;//> lon
					double lon2 = lon-len_neighbor*cosAlpha;//<lon
					double lat1 = lat+len_neighbor*sinAlpha;
					double lat2 = lat-len_neighbor*sinAlpha;
                    //get another point
                    double lon_other = _roadsegment[index]._segpoint[(i+1)&1].point.lon;
			        double lat_other = _roadsegment[index]._segpoint[(i+1)&1].point.lat;

                    //the ext point must posion outside of segment
					if(lon<lon_other)//lon
					{
					    _roadsegment[index]._segpoint[i].point_extern.lon = lon2;
					}
					else
					{
					    _roadsegment[index]._segpoint[i].point_extern.lon = lon1;
					}

					if(lat<lat_other)//lat
					{
					    _roadsegment[index]._segpoint[i].point_extern.lat = lat2;
					}
					else
					{
					    _roadsegment[index]._segpoint[i].point_extern.lat = lat1;
					}

			    }
				else 
				{
					_roadsegment[index]._segpoint[i].point_extern = _roadsegment[index]._segpoint[i].point;
				}
		    }

            index++;
		}
		
	}

    bool All_RoadSegment::RearrangeSegmentId()
    {
        //check sgeID
        if(!CheckSegmentID())
        {
            return false;
		}
		
        /***Input Segment ID has been orded by increased***/
        vector<crossing_record_t> crossing_record_v;
		uint32 SegIndex = 0;//new segment ID
		while(SegIndex < _segmentnum)
		{
            uint32 segId_tmp = _roadsegment[SegIndex].segId;
            //modefidy segid
            if(_roadsegment[SegIndex].segId > (SegIndex+1))
            {
                _roadsegment[SegIndex].segId = SegIndex+1;
                //modify point connect segid
			    for(uint32 i=0;i<_segmentnum;i++)
			    {
                    //pre
                    for(uint32 j=0;j<_roadsegment[i]._segpoint[0].conn_segNum;j++)
                    {
                        if(segId_tmp == _roadsegment[i]._segpoint[0].conn_segId[j])
                        {
                            _roadsegment[i]._segpoint[0].conn_segId[j] = SegIndex+1;
                        }
                    }
                    //next
                    for(uint32 k=0;k<_roadsegment[i]._segpoint[1].conn_segNum;k++)
                    {
                        if(segId_tmp == _roadsegment[i]._segpoint[1].conn_segId[k])
                        {
                            _roadsegment[i]._segpoint[1].conn_segId[k] = SegIndex+1;
                        }
                    }
		 	    }
				//modify line connect segid
				for(uint32 i=0;i<_segmentnum;i++)
				{
				    int linenum = _roadsegment[i].lineNum;
					for(int j=0;j<linenum;j++)
					{
					    int lineconnnum = _roadsegment[i]._segline[j].connSegNumofLine;
                        for(int k=0;k<lineconnnum;k++)
                        {
                            if(segId_tmp == _roadsegment[i]._segline[j].line_conn[k].segId)
                            {
                                _roadsegment[i]._segline[j].line_conn[k].segId = SegIndex+1;
                                //break;
							}
                        }
					}
				}
            }
			
            //record crossing id
            if((CROSSING_E == _roadsegment[SegIndex].segType) 
			|| (CROSSING_RA_E == _roadsegment[SegIndex].segType)
            || (CROSSING_AR_E == _roadsegment[SegIndex].segType)
            || (TROAD_CROSSING_RA_E == _roadsegment[SegIndex].segType)
            || (TROAD_CROSSING_AR_E == _roadsegment[SegIndex].segType))
            {
                 crossing_record_t tmp;
                 tmp.new_id = SegIndex+1;
                 tmp.old_id = segId_tmp;
                 tmp.old_conn = _roadsegment[SegIndex].crossing_seg;
                 crossing_record_v.push_back(tmp);
            }

            SegIndex++;
		}

        //modify crossing type
        for(uint32 i = 0;i < crossing_record_v.size();i++)
        {
            //fecth id
            uint32 old_id_local = crossing_record_v[i].old_id;
            uint32 new_id_local = crossing_record_v[i].new_id;
            //search old id in all the crossing segment 
            for(uint32 j = 0 ;j < crossing_record_v.size();j++)
            {
                uint32 new_id_cross = crossing_record_v[j].new_id;
                for(uint32 k = 0 ;k < _roadsegment[new_id_cross-1].crossing_seg.size();k++)
                {
                    if(old_id_local == _roadsegment[new_id_cross-1].crossing_seg[k])
                    {
                        //modify
                        _roadsegment[new_id_cross-1].crossing_seg[k] = new_id_local;
                    }
                }
            }
        }

        return true;
	}
    void All_RoadSegment::GenerateSecCfgInfo()
    {
        secCfgInfo_t tmp;
		if(!_segmentInfo.empty())
		{
		    _segmentInfo.clear();
		}
		
		for(uint32 j=0;j<_segmentnum;j++)
    	{
            tmp.secId = _roadsegment[j].segId;
            tmp.secType = _roadsegment[j].segType;
            tmp.prePointId = _roadsegment[j]._segpoint[0].pointId;
            tmp.nextPointId = _roadsegment[j]._segpoint[1].pointId;
            tmp.prePoint_ext = _roadsegment[j]._segpoint[0].point_extern;
            tmp.nextPoint_ext = _roadsegment[j]._segpoint[1].point_extern;
            tmp.prevSegId.clear();
            tmp.nextSegId.clear();

            uint32 i=0;
			//pre segment			
            while(i < _roadsegment[j]._segpoint[0].conn_segId.size())
            {
                tmp.prevSegId.push_back(_roadsegment[j]._segpoint[0].conn_segId[i]);
                i++;
            }
            i=0;
			//next segment
            while(i < _roadsegment[j]._segpoint[1].conn_segId.size())
            {
                tmp.nextSegId.push_back(_roadsegment[j]._segpoint[1].conn_segId[i]);
                i++;
            }
           _segmentInfo.push_back(tmp);
    	}
		
        return ;
    }

    bool All_RoadSegment::GeneratsecPointInfo()
    {
        uint32 index = 0;//point ID
        while(index < _secPointInfo.size())
        {
            //traverse all the connect segment to get the conncet piont
            uint32 cnt=0;
            while(cnt < _secPointInfo[index].connSecs.size())
            {
                uint32 segIdex = _secPointInfo[index].connSecs[cnt] - 1;
                uint32 another_pointId;
                if(index == _segmentInfo[segIdex].prePointId) //pre or next point
                {
                    another_pointId = _segmentInfo[segIdex].nextPointId;
                }
                else
                {
                    another_pointId = _segmentInfo[segIdex].prePointId;
                }
                _secPointInfo[index].connPtsId.push_back(another_pointId);

                cnt++;
            }
            index++;
        }

        return true;
    }
	void All_RoadSegment::ArrangePointId()
    {
        for(uint32 i=0;i<_segmentnum;i++)
        {
            uint32 segmentId = _roadsegment[i].segId;

			for(int j=0;j<2;j++)
			{
                point3D_t point = _roadsegment[i]._segpoint[j].point;
			    uint32 index=0;
                while(index < _secPointInfo.size())
                {
                    if((point.lon == _points_v[index].lon)&&(point.lat == _points_v[index].lat))
                    {
                        break;
                    }
					index++;
                }

				//point not in the vector
			    if(index == _secPointInfo.size())
			    {
                    secPointInfo_t pointInf_tmp;
                    pointInf_tmp.pointId = index;
                    pointInf_tmp.connSecs.push_back(segmentId);
                    _secPointInfo.push_back(pointInf_tmp);
                    _points_v.push_back(point);
				}
				else
				{
                    //just push connection segment
                    _secPointInfo[index].connSecs.push_back(segmentId);
				}
				
			    //modify pointId of _roadsegment
				_roadsegment[i]._segpoint[j].pointId = index;
			}
        }   
		
        return ;
    }
    bool All_RoadSegment::AnalysisSegment(const char *path)
    {
        uint32 firstSegId = 0;
        
        //open XML
        TiXmlDocument XMLdoc(path);
        if(!XMLdoc.LoadFile())
	    {
		    cout<<"fail to load config file"<<endl;
		    return false;
	    }

        TiXmlElement* root = XMLdoc.RootElement();
	    TiXmlElement* segment_xml;		
	    segment_xml = root->FirstChildElement();
		//reference point
		_reference_point.lat = stringToNum<double>(segment_xml->GetText());
		segment_xml = segment_xml->NextSiblingElement();
		_reference_point.lon = stringToNum<double>(segment_xml->GetText());
		segment_xml = segment_xml->NextSiblingElement();
        //segnumber
        _segmentnum = stringToNum<uint32>(segment_xml->GetText());
        segment_xml = segment_xml->NextSiblingElement();

        int index = -1;
        int insert_idex;
		int cnt=0;
        while(NULL != segment_xml)
        {
            cnt++;
            RoadSegment seg_t;
			uint32 type;
            //segID
            TiXmlElement* nextElement = segment_xml->FirstChildElement();
		    seg_t.segId = stringToNum<uint16>(nextElement->GetText());
            //segType
            nextElement = nextElement->NextSiblingElement();
		    type = stringToNum<uint32>(nextElement->GetText());
			if(type >= DEFAULT_E)
			{
                printf("segment type error!\n");
				return false;
			}
			seg_t.segType = (segment_type_e)type;
			seg_t.crossing_seg.clear();
			if((CROSSING_E == seg_t.segType) || (seg_t.segType == CROSSING_RA_E) 
                || (seg_t.segType == CROSSING_AR_E) || (seg_t.segType == TROAD_CROSSING_RA_E) 
                || (seg_t.segType == TROAD_CROSSING_AR_E))
			{			
			    //seg_crossing_num
			    nextElement = nextElement->NextSiblingElement();
                int crossingseg_num = stringToNum<uint32>(nextElement->GetText());
				for(int i=0;i < crossingseg_num;i++)
				{				
				    //crossing_segId
				    nextElement = nextElement->NextSiblingElement();
				    uint32 crossing_segid = stringToNum<uint32>(nextElement->GetText());
					seg_t.crossing_seg.push_back(crossing_segid);
				}
			}
            //lanenum
            nextElement = nextElement->NextSiblingElement();
		    seg_t.laneNum = stringToNum<uint32>(nextElement->GetText());
            //lineNum
            nextElement = nextElement->NextSiblingElement();
		    seg_t.lineNum = stringToNum<uint32>(nextElement->GetText());
            //gpspoint number
            nextElement = nextElement->NextSiblingElement();
            seg_t.pointNum = stringToNum<uint32>(nextElement->GetText());
            
            //point element
            index = seg_t.pointNum;
            while(index)
            {
                Segmentpoint point_t;
                index--;
                nextElement = nextElement->NextSiblingElement();
                TiXmlElement* pointElement = nextElement->FirstChildElement();
                //point ID
                point_t.pointId = stringToNum<uint32>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
                //point type -- bit0 : point type , bit1:extern or not , bit2:short extern or not
                uint32 point_type= stringToNum<uint32>(pointElement->GetText());
				point_t.point_type = point_type & BIT0;
				point_t.point_type_extern = (point_type & BIT1) >> 1;
				point_t.point_type_extern_stort = (point_type & BIT2) >> 2;
                pointElement = pointElement->NextSiblingElement();
				//connect segment number
				point_t.conn_segNum = stringToNum<uint32>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
				uint32 i = 0;
				while(i < point_t.conn_segNum)
                {
                    //connect segmentID
                    point_t.conn_segId.push_back(stringToNum<uint32>(pointElement->GetText()));
                    pointElement = pointElement->NextSiblingElement();
                    i++;
                }
				
                //gps 
                point_t.point.lat = stringToNum<double>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
                point_t.point.lon = stringToNum<double>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
                //convert
                AllSecPointConvert(point_t.point);
				
                seg_t._segpoint.push_back(point_t);
            }
     
            //lane element
            index = seg_t.laneNum;
            while(index)
            {
                Segmentlane lane_t;
                index--;
                nextElement = nextElement->NextSiblingElement();
                TiXmlElement* laneElement = nextElement->FirstChildElement();
                //laneID
                lane_t.laneId = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //start position
                lane_t.start_pos_ID = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //stop position
                lane_t.end_pos_ID = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //direction
                lane_t.derection = stringToNum<bool>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //merge left 
                lane_t.merge_left = stringToNum<uint32>(laneElement->GetText()); 
                laneElement = laneElement->NextSiblingElement();
                //merge right
                lane_t.merge_right = stringToNum<uint32>(laneElement->GetText()); 
                laneElement = laneElement->NextSiblingElement();
                //left line
                lane_t.leftlineID = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //right line
                lane_t.rightlineID = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //inert lane and sort
                insert_idex = seg_t.insertLane(&lane_t);
                seg_t._seglane.insert(seg_t._seglane.begin()+insert_idex , lane_t);
            }
            //check lane
            if(!seg_t.CheckLaneId())
                return false;

            //line element
            index = seg_t.lineNum;
            while(index)
            {
                Segmentline line_t;
                index--;
                nextElement = nextElement->NextSiblingElement();
                TiXmlElement* lineElement = nextElement->FirstChildElement();
                //line ID
                line_t.lineId = stringToNum<uint32>(lineElement->GetText());
                lineElement = lineElement->NextSiblingElement();
                //line type
                line_t.linetype = stringToNum<uint32>(lineElement->GetText());
                lineElement = lineElement->NextSiblingElement();
				//con_segNum
				line_t.connSegNumofLine = stringToNum<uint32>(lineElement->GetText());
                lineElement = lineElement->NextSiblingElement();
				uint32 i = 0;
				while(i < line_t.connSegNumofLine)
				{				
				    ele_lineConn lineconn_tmp;
                    //connection segID
                    lineconn_tmp.segId = stringToNum<uint32>(lineElement->GetText());
                    lineElement = lineElement->NextSiblingElement();
                    //connection lineID 
                    uint32 lineconn = stringToNum<uint32>(lineElement->GetText());
                    lineconn_tmp.lineId = lineconn & ((1 << CONN_BIT_WIDE) - 1) ;
					lineconn_tmp.lineConnType = (lineconn & (1 << CONN_BIT_WIDE)) >> CONN_BIT_WIDE;
                    lineElement = lineElement->NextSiblingElement();
					line_t.line_conn.push_back(lineconn_tmp);
                    i++;
    			}
                //insert line and sort
                insert_idex = seg_t.insertLine(&line_t);
                seg_t._segline.insert(seg_t._segline.begin()+insert_idex , line_t);
            }
            //check line
            if(!seg_t.CheckLineId())
                return false;

            //insert segment and sort
            insert_idex = insertSegment(&seg_t);
            _roadsegment.insert(_roadsegment.begin()+insert_idex , seg_t);
            //point to next segmen 
		    segment_xml = segment_xml->NextSiblingElement();
        }

		//rearrange segment Id
        bool result = RearrangeSegmentId();
		if(!result)
		{return false;}
		//rearrange point Id
        ArrangePointId();
	    //generate the extern point
		CalcExtendPoint();
		//generate _segmentInfo
        GenerateSecCfgInfo();                
		//generate the points info
		GeneratsecPointInfo();
		//store the loopseg_ListInfo
		if(!getRoadLoopSegId_In(_loopseg_ListInfo))
		return false;
        if(!setLoopPointInfo())
        return false;

        return true;
    }
    bool All_RoadSegment::getsecPointInfo(vector<secPointInfo_t> &PointInfo)
    {
        PointInfo.clear();
        PointInfo = _secPointInfo;
        return true;
    }
    bool All_RoadSegment::getSecCfgInfo(vector<secCfgInfo_t> &SegInfo)
    {
        SegInfo.clear();
        SegInfo = _segmentInfo;
        return true;
	}
	bool All_RoadSegment::getsecPointVector(vector<point3D_t> &Point_vector)
	{
    	Point_vector.clear();
	    Point_vector = _points_v;
	    return true;
	}
    void All_RoadSegment::getRoadSegCfg_database(list<segAttributes_t> &segCfgList)
    {
        if(!segCfgList.empty())
        {segCfgList.clear();}
		
        segAttributes_t segmentElement;
		memset(&segmentElement , 0 , sizeof(segAttributes_t));
        
        uint32 index = 0;
        while(index < _segmentnum)
        {
            segmentElement.segId_used = 1;
            segmentElement.segId  = _roadsegment[index].segId;
            segmentElement.uiLaneNum_used = 1;
            segmentElement.uiLaneNum  = _roadsegment[index].laneNum;
            //0:pre 1:next
            segmentElement.ports[0] = _roadsegment[index]._segpoint[0].point;
            segmentElement.ports[1] = _roadsegment[index]._segpoint[1].point;
            segmentElement.ports[2] = _roadsegment[index]._segpoint[0].point_extern;
            segmentElement.ports[3] = _roadsegment[index]._segpoint[1].point_extern;
    
            segCfgList.push_back(segmentElement);
            index++;
        }
        
        return;
    }

	bool All_RoadSegment::getLeftOrRightneighbourLineId(int segId, int overline, int conSegId, int turnTo, int &conLineL, int &conLineR)
	{
		conLineL = 0;
		conLineR = 0;
		int segIndex = segId - 1;
		if(segIndex < 0)return false;

		int conSegIndex = conSegId - 1;
		if(conSegIndex < 0)return false;

		if (0 == turnTo)
		{
			return false;
		}

		if (segId == conSegId)
		{			
			for(int i=0;i<_roadsegment[segIndex].laneNum;i++)
			{
				if (1 == turnTo)
				{
					if (overline == _roadsegment[segIndex]._seglane[i].rightlineID)
					{
						conLineL = _roadsegment[segIndex]._seglane[i].leftlineID;
						conLineR = _roadsegment[segIndex]._seglane[i].rightlineID;
						return true;
					}
				} 
				else
				{
					if (overline == _roadsegment[segIndex]._seglane[i].leftlineID)
					{
						conLineL = _roadsegment[segIndex]._seglane[i].rightlineID;
						conLineR = _roadsegment[segIndex]._seglane[i].leftlineID;
						return true;
					}
				}
			}
			return false;
		} 

		vector<int> lines;
		int i=0;			
		while(i < _roadsegment[segIndex].lineNum)
		{
			if (overline == _roadsegment[segIndex]._segline[i].lineId)
			{
				for(uint32 j = 0; j < _roadsegment[segIndex]._segline[i].line_conn.size(); j++)     
				{
					if(conSegId == _roadsegment[segIndex]._segline[i].line_conn[j].segId)
					{
						int tempId = _roadsegment[segIndex]._segline[i].line_conn[j].lineId;
						lines.push_back(tempId);
					}
				}
			}
			i++;
		}

		if (lines.empty())
		{
			i = 0;
			while(i < _roadsegment[conSegIndex].lineNum)
			{

				for(uint32 j = 0; j < _roadsegment[conSegIndex]._segline[i].line_conn.size(); j++)     
				{
					if(segId == _roadsegment[conSegIndex]._segline[i].line_conn[j].segId && overline == _roadsegment[conSegIndex]._segline[i].line_conn[j].lineId)
					{
						int tempId = _roadsegment[conSegIndex]._segline[i].line_conn[j].lineId;
						lines.push_back(tempId);
					}
				}

				i++;
			}
		}

		for (int lineIndex = 0; lineIndex < lines.size(); lineIndex++)
		{
			for(int i=0;i<_roadsegment[conSegIndex].laneNum;i++)
			{
				if (1 == turnTo)
				{
					if(lines[lineIndex] == _roadsegment[conSegIndex]._seglane[i].rightlineID)
					{
						conLineL = _roadsegment[conSegIndex]._seglane[i].leftlineID;
						conLineR = _roadsegment[conSegIndex]._seglane[i].rightlineID;
						return true;
					}
				} 
				else
				{
					if(lines[lineIndex] == _roadsegment[conSegIndex]._seglane[i].leftlineID)
					{
						conLineL = _roadsegment[conSegIndex]._seglane[i].leftlineID;
						conLineR = _roadsegment[conSegIndex]._seglane[i].rightlineID;
						return true;
					}
				}
			}
		}

		return false;
	}

	bool All_RoadSegment::getConnectedLineId(int segId, int lineL, int lineR, int conSegId, int &conLineL, int &conLineR)
	{
		conLineL = 0;
		conLineR = 0;
		int segIndex = segId - 1;
		if(segIndex < 0)return false;

		int conIndex = conSegId - 1;
		if(conIndex < 0)return false;

		vector<int> findLinesL, findLinesR;
		int i=0;			
		while(i < _roadsegment[segIndex].lineNum)
		{
			if (lineL == _roadsegment[segIndex]._segline[i].lineId)
			{
				for(uint32 j = 0; j < _roadsegment[segIndex]._segline[i].line_conn.size(); j++)     
				{
					if(conSegId == _roadsegment[segIndex]._segline[i].line_conn[j].segId)
					{
						findLinesL.push_back(_roadsegment[segIndex]._segline[i].line_conn[j].lineId);
					}
				}
			}

			if (lineR == _roadsegment[segIndex]._segline[i].lineId)
			{
				for(uint32 j = 0; j < _roadsegment[segIndex]._segline[i].line_conn.size(); j++)     
				{
					if(conSegId == _roadsegment[segIndex]._segline[i].line_conn[j].segId)
					{
						findLinesR.push_back(_roadsegment[segIndex]._segline[i].line_conn[j].lineId);
					}
				}
			}

			i++;
		}

		if (findLinesL.empty() && findLinesR.empty())
		{
			int i = 0;
			while(i < _roadsegment[conIndex].lineNum)
			{
				for(uint32 j = 0; j < _roadsegment[conIndex]._segline[i].line_conn.size(); j++)     
				{
					if(segId == _roadsegment[conIndex]._segline[i].line_conn[j].segId && lineL == _roadsegment[conIndex]._segline[i].line_conn[j].lineId)
					{
						findLinesL.push_back(_roadsegment[conIndex]._segline[i].lineId);
					}

					if(segId == _roadsegment[conIndex]._segline[i].line_conn[j].segId && lineR == _roadsegment[conIndex]._segline[i].line_conn[j].lineId)
					{
						findLinesR.push_back(_roadsegment[conIndex]._segline[i].lineId);
					}
				}

				i++;
			}
		}
/*
		if ((findLinesL.empty() && findLinesR.empty())
			|| (findLinesL.empty() && 1 < findLinesR.size())
			|| (findLinesR.empty() && 1 < findLinesL.size()))
		{
			return false;
		}
*/
		if(findLinesL.empty() && 1 == findLinesR.size() )
		{
			for(int i = 0; i < _roadsegment[conIndex].laneNum; i++)
			{
				if(findLinesR.front() == _roadsegment[conIndex]._seglane[i].rightlineID)
				{
					conLineL = _roadsegment[conIndex]._seglane[i].leftlineID;
					conLineR = _roadsegment[conIndex]._seglane[i].rightlineID;
					return true;
				}
			}			
		}

		if(findLinesR.empty() && 1 == findLinesL.size() )
		{
			for(int i = 0; i < _roadsegment[conIndex].laneNum; i++)
			{
				if(findLinesL.front() == _roadsegment[conIndex]._seglane[i].leftlineID)
				{
					conLineL = _roadsegment[conIndex]._seglane[i].leftlineID;
					conLineR = _roadsegment[conIndex]._seglane[i].rightlineID;
					return true;
				}
			}			
		}

		for (int indexL = 0; indexL < findLinesL.size(); indexL++)
		{
			for(int i = 0; i < _roadsegment[conIndex].laneNum; i++)
			{
				if(findLinesL[indexL] == _roadsegment[conIndex]._seglane[i].leftlineID)
				{
					for (int indexR = 0; indexR < findLinesR.size(); indexR++)
					{
						if(findLinesR[indexR] == _roadsegment[conIndex]._seglane[i].rightlineID)
						conLineL = _roadsegment[conIndex]._seglane[i].leftlineID;
						conLineR = _roadsegment[conIndex]._seglane[i].rightlineID;
						return true;
					}
				}
			}
		}

		return false;
	}
/*
	bool All_RoadSegment::getPrevSegId(IN int currSegId, OUT vector<int> &prevSegId)
	{
		int index = getSegmentIndex(currSegId);
		if(index < 0)return false;

		prevSegId = _segmentInfo[index].prevSegId;
		return true;
	}

	bool All_RoadSegment::getNextSegId(IN int currSegId, vector<int> &nextSegId)
	{
		int index = getSegmentIndex(currSegId);
		if(index < 0)return false;

		nextSegId = _segmentInfo[index].nextSegId;
		return true;
	}
*/
    bool All_RoadSegment::getMatchedLinesInfo(int SegId, list<vector<lineConn>> &lineconn)
    {
        int index = 0;	
		int segIndex = SegId - 1;
		if(segIndex < 0)return false;

		if(!lineconn.empty())
		{lineconn.clear();}

        while(index < _roadsegment[segIndex].lineNum)
        {
            vector<lineConn> lineconn_v_tmp;  
			lineconn_v_tmp.clear();
            for(uint32 i=0;i<_roadsegment[segIndex]._segline[index].connSegNumofLine;i++)
            {
                lineConn lineconn_tmp;
                lineconn_tmp.SegID = SegId;
                lineconn_tmp.LineID = index+1;
                lineconn_tmp.connSegID = _roadsegment[segIndex]._segline[index].line_conn[i].segId;
                lineconn_tmp.connLineID = _roadsegment[segIndex]._segline[index].line_conn[i].lineId;
                lineconn_v_tmp.push_back(lineconn_tmp);
            }
            lineconn.push_back(lineconn_v_tmp);
            index++;
		}

        return true;
	}
	bool All_RoadSegment::getMatchedLinesInfoOfStartPoint(int SegId, list<vector<lineConn>> &lineconn , int type)
	{
		int segIndex = SegId - 1;
		if(segIndex < 0)return false;

		if(!lineconn.empty())
		{lineconn.clear();}

		//get point Id
		uint32 pointId = _segmentInfo[segIndex].prePointId;
        int i=0;		
		//traverse all the line		
        while(i < _roadsegment[segIndex].lineNum)
        {
            vector<lineConn> lineconn_v_tmp;
			lineconn_v_tmp.clear();
			//traverse all the line connect info
            for(uint32 j=0;j<_roadsegment[segIndex]._segline[i].line_conn.size();j++)     
            {
                //check line connect info is pre or not
                for(uint32 k=0;k<_segmentInfo[segIndex].prevSegId.size();k++)
                {
                    uint32 conn_segid = _segmentInfo[segIndex].prevSegId[k];
					if((conn_segid == _roadsegment[segIndex]._segline[i].line_conn[j].segId)
						&& (type == _roadsegment[segIndex]._segline[i].line_conn[j].lineConnType))
					{					
					    lineConn lineconn_tmp;
						lineconn_tmp.SegID = SegId;  
						lineconn_tmp.LineID = i+1;	  
						lineconn_tmp.connSegID = conn_segid; 
						lineconn_tmp.connLineID = _roadsegment[segIndex]._segline[i].line_conn[j].lineId;
						if(pointId == _segmentInfo[conn_segid-1].prePointId)
						{
							lineconn_tmp.connType = STARTPOINT;
						}
						else if(pointId == _segmentInfo[conn_segid-1].nextPointId)
						{
							lineconn_tmp.connType = ENDPOINT;
						}
						else //error
						{
							printf("The connect segment of %d do not exit!/n",SegId);
							return false;			   
						}
						lineconn_v_tmp.push_back(lineconn_tmp);
						
                        break;
					}
				}
            }
			
			lineconn.push_back(lineconn_v_tmp); 		              		
            i++;
        }
 	    return true;
	}	
	bool All_RoadSegment::getMatchedLinesInfoOfEndPoint(int SegId, list<vector<lineConn>> &lineconn , int type)
	{
		int segIndex = SegId - 1;
		if(segIndex < 0)return false;

		if(!lineconn.empty())
		{lineconn.clear();}

		//get point Id
		uint32 pointId = _segmentInfo[segIndex].nextPointId;
		//traverse all the line		
		int i=0;
		while(i < _roadsegment[segIndex].lineNum)
		{		
		    vector<lineConn> lineconn_v_tmp;
		    for(uint32 j=0;j<_roadsegment[segIndex]._segline[i].line_conn.size();j++)
		    {
                //check line connect info is next or not
                for(uint32 k=0;k<_segmentInfo[segIndex].nextSegId.size();k++)
                {
                    uint32 conn_segid = _segmentInfo[segIndex].nextSegId[k];
				    if((conn_segid == _roadsegment[segIndex]._segline[i].line_conn[j].segId) 
					  && (type == _roadsegment[segIndex]._segline[i].line_conn[j].lineConnType))
				    {
						lineConn lineconn_tmp;
						lineconn_tmp.SegID = SegId;  
						lineconn_tmp.LineID = i+1;	  
						lineconn_tmp.connSegID = conn_segid; 
						lineconn_tmp.connLineID = _roadsegment[segIndex]._segline[i].line_conn[j].lineId;
						if(pointId == _segmentInfo[conn_segid-1].prePointId)
						{
							lineconn_tmp.connType = STARTPOINT;
						}
						else if(pointId == _segmentInfo[conn_segid-1].nextPointId)
						{
							lineconn_tmp.connType = ENDPOINT;
						}
						else //error
						{
							printf("The connect segment of %d do not exit!/n",SegId);
							return false;			   
						}
						lineconn_v_tmp.push_back(lineconn_tmp);
						break;
					}
				}
			}
			
			lineconn.push_back(lineconn_v_tmp); 		              		
            i++;
		}

        return true;
	}
	void All_RoadSegment::getRefGPS(point3D_t & RefPoint)
	{
		RefPoint = _reference_point;
	}
	bool All_RoadSegment::getflyCrossId(uint32 flyoverSecId,vector<uint32> &flyCrossSecId)
	{
	
        if(flyoverSecId<=0)
        {
            printf("crossing road ID error!\n");
            return false;
		}

		flyCrossSecId = _roadsegment[flyoverSecId-1].crossing_seg;
		
	    return true;
    }

	void All_RoadSegment::bubble_sort(vector<uint32> &input)
	{
	    if(input.empty())
	    {
            return;
		}
		
        int pos = input.size()-1;

		while(pos>0)
		{
            int j = 0;
			for(int i = 0 ; i < pos ; i++)
			{
                if(input[i]>input[i+1])
                {
                    //swap
                    uint32 tmp = input[i];
                    input[i] = input[i+1];
                    input[i+1] = tmp;
                    j = i;
				}
			}
			pos = j;
		}
		
	}

	void All_RoadSegment::loop_generate_DFS(uint32 *visit , int segIndex ,vector<uint32> &loop_seg)//DEEP FIRST,generate a loop
	{
        //get segment iter
	    uint32 i = 0;
	    vector<vector<uint32>>::iterator input = _segLinkInfo_DFS.begin();
		input += segIndex;

        //record the segment
        loop_seg.push_back(segIndex+1);
	    visit[segIndex]=1;

        //DFS the link segment
	    for(i=0;i < input->size();i++)
	    {
            uint32 link_seg = input->at(i);
			if(0 == visit[link_seg-1])
			{
				loop_generate_DFS(visit , link_seg-1 , loop_seg);
			}
		}
        return;
    }

	bool  All_RoadSegment::getRoadLoopSegId(list<vector<uint32>> &loop_segList)
	{
		loop_segList = _loopseg_ListInfo;
		
		return true;
	}

    bool All_RoadSegment::getRoadLoopSegId_In(list<vector<uint32>> &loopseg_List)
    {
        uint32 i = 0;
        vector<uint32> link;
        //create link list
        for(i=0;i < _segmentInfo.size();i++)
        {
            link = _segmentInfo[i].prevSegId;
            link.insert(link.end(),_segmentInfo[i].nextSegId.begin(),_segmentInfo[i].nextSegId.end());
            _segLinkInfo_DFS.push_back(link);
        }

		//init visit
        uint32 *visit = new (uint32[_segmentnum]);//0: unvisit  1:visit
        if(NULL == visit)
		{
            return false;
		}
        memset(visit , 0 , sizeof(uint32)*_segmentnum);
		
        i=0;
		while(i < _segmentnum)
		{
		    if(0 == visit[i])
		    {
		        vector<uint32> loop_seg;
		        //generate a loop
		        loop_generate_DFS(visit , i , loop_seg);
				loopseg_List.push_back(loop_seg);
                i = 0;
				continue;
			}
            i++;
		}

        //sort
        list<vector<uint32>>::iterator iter = loopseg_List.begin();
        for(iter = loopseg_List.begin(); iter != loopseg_List.end() ; iter++)
        {
            bubble_sort(*iter);
        }

	    delete visit;
        return true;
    }

	
	void All_RoadSegment::getRoadSegmentInSky(vector<uint32> &SegmentInSky)
	{
	    uint32 index = 0;
        if(!SegmentInSky.empty())
        {SegmentInSky.clear();}

        while(index < _segmentnum)
		{
            if((CROSSING_AR_E == _roadsegment[index].segType) || (TROAD_CROSSING_AR_E == _roadsegment[index].segType))
            {
                SegmentInSky.push_back(index+1);
            }
            index++;
		}
		
        return;
	}

	bool All_RoadSegment::getPrevSegId(uint32 segmentId , vector<uint32> &prevec)
	{
	    if(segmentId > _roadsegment.size())
	    {
            return false;
		}

        if(!prevec.empty())
		{
		    prevec.clear();
        }

		prevec = _roadsegment[segmentId-1]._segpoint[0].conn_segId;

		return true;
	}
    
	bool All_RoadSegment::getNextSegId(uint32 segmentId , vector<uint32> &nextvec)
	{
	    if(segmentId > _roadsegment.size())
	    {
            return false;
		}

        if(!nextvec.empty())
        {
            nextvec.clear();
        }

		nextvec = _roadsegment[segmentId-1]._segpoint[1].conn_segId;

		return true;
	}

	
	bool All_RoadSegment::getSegmentType(uint32 segmentId , segment_type_e &type)
	{
	    if(segmentId > _roadsegment.size())
	    {
            return false;
		}

		type = _roadsegment[segmentId-1].segType;
		
        return true;
	}
    
	bool All_RoadSegment::getBodyPoint(uint32 segmentId , uint32 type , point3D_t &point)
	{
		if(segmentId > _roadsegment.size())
		{
			return false;
		}
		
		if(STARTPOINT == type)//pre
		{
			point = _roadsegment[segmentId-1]._segpoint[0].point;
		}
		else if(ENDPOINT == type)//next
		{
			point = _roadsegment[segmentId-1]._segpoint[1].point;
		}
		else
		{
			return false;
		}
		
		return true;
	}
    
    void All_RoadSegment::setRoadLoopFlag(list<vector<uint32>> &loopseg_List, list<segAttributes_t> &segCfgList)
    {
        uint8 loopIndex = 0;
        list<vector<uint32>>::iterator segListIter = loopseg_List.begin();
        while(segListIter != loopseg_List.end())
        {
            vector<uint32>::iterator segIdIter = (*segListIter).begin();
            while(segIdIter != (*segListIter).end())
            {
                uint32 segId = (*segIdIter); // get segID in loopseg_List
                list<segAttributes_t>::iterator segCfgIter = segCfgList.begin();
                while(segCfgIter != segCfgList.end())
                {
                    // if seg id is same, then set loop index to segCfgList
                    if(segId == segCfgIter->segId)
                    {
                        segCfgIter->loopIdx_used = 1;
                        segCfgIter->loopIdx = loopIndex;
                        break;
                    }
                    ++segCfgIter;
                }
                ++segIdIter;
            }
            ++segListIter;
            loopIndex++;
        }
    }

	bool All_RoadSegment::setLoopPointInfo()
	{
		if(_loopseg_ListInfo.empty())
		{
			return false;
		}

        if(!_looppoint_ListInfo.empty())
        {
            _looppoint_ListInfo.clear();
		}

        //init
		int size = _secPointInfo.size();
		int *point_used = new int[size];
		memset(point_used , 0 , sizeof(int)*size);
		
		list<vector<uint32>>::iterator iter;
		for(iter = _loopseg_ListInfo.begin();iter != _loopseg_ListInfo.end();iter++)
        {
            uint32 pointIndex = 0;
			vector<uint32> PointInfo_loop;
            for(int i=0; i < iter->size();i++)
            {
                uint32 segmentId = iter->at(i);
                //pre
                pointIndex = _roadsegment[segmentId-1]._segpoint[0].pointId;
                if(0 == point_used[pointIndex])
                {
                    point_used[pointIndex] = 1;
                    PointInfo_loop.push_back(pointIndex);
                }
                //next
                pointIndex = _roadsegment[segmentId-1]._segpoint[1].pointId;
                if(0 == point_used[pointIndex])
                {
                    point_used[pointIndex] = 1;
                    PointInfo_loop.push_back(pointIndex);
                }
            } 
            _looppoint_ListInfo.push_back(PointInfo_loop);
        }

		delete[] point_used;

		return true;
	}
	bool All_RoadSegment::getLoopPointInfo(uint32 loopId , vector<uint32> &PointInfo_loop)
	{
		if(_looppoint_ListInfo.empty())
		{
			return false;
		}

        //check  
		if(loopId >= _looppoint_ListInfo.size())
		{
            return false;
		}

        //find loop 
        list<vector<uint32>>::iterator iter = _looppoint_ListInfo.begin();
        for(int i=0; i < loopId;i++)
        {
            iter++;
        }

        if(!PointInfo_loop.empty())
        {
            PointInfo_loop.clear();
        }

		PointInfo_loop = *iter;
		
        return true;
	}
}
