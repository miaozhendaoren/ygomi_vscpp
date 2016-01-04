
#include <string>
#include <complex>
#include <vector>
#include "RoadSeg.h"
#include "XMLParser.h"

using std::vector;
namespace ns_roadsegment
{
    Segmentpoint::Segmentpoint()
    {
        pointId = 0;
        point_type = 0;
        conn_segId = 0;
        memset(&point,0,sizeof(point3D_t));
    }
    Segmentlane::Segmentlane()
    {
        laneId = 0;		  
		derection = 0;
		merge_left = 0;
		merge_right = 0;
        memset(&start_pos,0,sizeof(point3D_t));
        memset(&end_pos,0,sizeof(point3D_t));
		leftlineID = 0;
		rightlineID = 0; 
    }
    Segmentline::Segmentline()
    {
        lineId = 0;
		linetype = 0;
		connLineID = 0;
		connSegID = 0;
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
        if(_seglane[0].laneId != 1)
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
        if(_segline[0].lineId != 1)
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
        if(_segpoint[0].pointId != 1)
        {
            return false;
        }

        if(_segpoint.size() != _segpoint[_segpoint.size()-1].pointId - _segpoint[0].pointId + 1)
            return false;
        else
            return true;
    }

    uint32 All_RoadSegment::getSegmentNum()
    {
        return segmentnum;        
    }
	uint32 All_RoadSegment::getSegmentID(uint32 index)
	{
	    return _roadsegment[index].segId;
	}
	uint32 All_RoadSegment::getLaneofSegNum(uint32 index)
    {
        return _roadsegment[index].getLaneNum();
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
    bool All_RoadSegment::CheckSegmentID()
    {
        if(_roadsegment.size() != _roadsegment[_roadsegment.size()-1].segId - _roadsegment[0].segId + 1)
            return false;
        else
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
    uint32 All_RoadSegment::AnalysisSegment()
    {
        uint32 firstSegId = 0;
        
        //open XML
        TiXmlDocument XMLdoc("./config/DE_Airport_ParamConfig.xml");
        if(!XMLdoc.LoadFile())
	    {
		    cout<<"fail to load config file"<<endl;
		    return 0;
	    }

        TiXmlElement* root = XMLdoc.RootElement();
	    TiXmlElement* segment;
        //segnumber
	    segment = root->FirstChildElement();
        segmentnum = stringToNum<uint32>(segment->GetText());
        segment = segment->NextSiblingElement();

        int index = -1;
        int insert_idex;
        while(NULL != segment)
        {
            RoadSegment *p_seg_t = new RoadSegment;
            //segID
            TiXmlElement* nextElement = segment->FirstChildElement();
		    p_seg_t->segId = stringToNum<uint16>(nextElement->GetText());
            if(-1 == index)//first segment
            {firstSegId = p_seg_t->segId;}
            //segType
            nextElement = nextElement->NextSiblingElement();
		    p_seg_t->segType = stringToNum<uint32>(nextElement->GetText());
            //lanenum
            nextElement = nextElement->NextSiblingElement();
		    p_seg_t->laneNum = stringToNum<uint32>(nextElement->GetText());
            //lineNum
            nextElement = nextElement->NextSiblingElement();
		    p_seg_t->lineNum = stringToNum<uint32>(nextElement->GetText());
            //gpspoint number
            nextElement = nextElement->NextSiblingElement();
            p_seg_t->pointNum = stringToNum<uint32>(nextElement->GetText());
            
            //point element
            index = p_seg_t->pointNum;
            while(index)
            {
                Segmentpoint *p_point_t = new Segmentpoint;
                index--;
                nextElement = nextElement->NextSiblingElement();
                TiXmlElement* pointElement = nextElement->FirstChildElement();
                //point ID
                p_point_t->pointId = stringToNum<uint32>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
                //point type
                p_point_t->point_type = stringToNum<uint32>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
                //connect segmentID
                p_point_t->conn_segId = stringToNum<uint32>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
                //gps 
                p_point_t->point.lat = stringToNum<double>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();
                p_point_t->point.lon = stringToNum<double>(pointElement->GetText());
                pointElement = pointElement->NextSiblingElement();

                //inert lane and sort
                insert_idex = p_seg_t->insertPoint(p_point_t);
                p_seg_t->_segpoint.insert(p_seg_t->_segpoint.begin()+insert_idex , *p_point_t);
            }
            //check gps point
            if(!p_seg_t->CheckPointId())
                return false;

            //lane element
            index = p_seg_t->laneNum;
            while(index)
            {
                Segmentlane *p_lane_t = new Segmentlane;
                index--;
                nextElement = nextElement->NextSiblingElement();
                TiXmlElement* laneElement = nextElement->FirstChildElement();
                //laneID
                p_lane_t->laneId = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //start position
                p_lane_t->start_pos.lat = stringToNum<double>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                p_lane_t->start_pos.lon = stringToNum<double>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //stop position
                p_lane_t->end_pos.lat = stringToNum<double>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                p_lane_t->end_pos.lon = stringToNum<double>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //direction
                p_lane_t->derection = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //merge left 
                p_lane_t->merge_left = stringToNum<uint32>(laneElement->GetText()); 
                laneElement = laneElement->NextSiblingElement();
                //merge right
                p_lane_t->merge_right = stringToNum<uint32>(laneElement->GetText()); 
                laneElement = laneElement->NextSiblingElement();
                //left line
                p_lane_t->leftlineID = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                //right line
                p_lane_t->rightlineID = stringToNum<uint32>(laneElement->GetText());
                laneElement = laneElement->NextSiblingElement();
                
                //inert lane and sort
                insert_idex = p_seg_t->insertLane(p_lane_t);
                p_seg_t->_seglane.insert(p_seg_t->_seglane.begin()+insert_idex , *p_lane_t);
            }
            //check lane
            if(!p_seg_t->CheckLaneId())
                return false;

            //line element
            index = p_seg_t->lineNum;
            while(index)
            {
                Segmentline *p_line_t = new Segmentline;
                index--;
                nextElement = nextElement->NextSiblingElement();
                TiXmlElement* lineElement = nextElement->FirstChildElement();
                //line ID
                p_line_t->lineId = stringToNum<uint32>(lineElement->GetText());
                lineElement = lineElement->NextSiblingElement();
                //line type
                p_line_t->linetype = stringToNum<uint32>(lineElement->GetText());
                lineElement = lineElement->NextSiblingElement();
                //connection segID
                p_line_t->connSegID = stringToNum<uint32>(lineElement->GetText());
                lineElement = lineElement->NextSiblingElement();
                //connection lineID 
                p_line_t->connLineID = stringToNum<uint32>(lineElement->GetText());
                lineElement = lineElement->NextSiblingElement();

                //insert line and sort
                insert_idex = p_seg_t->insertLine(p_line_t);
                p_seg_t->_segline.insert(p_seg_t->_segline.begin()+insert_idex , *p_line_t);
            }
            //check line
            if(!p_seg_t->CheckLineId())
                return false;

            //insert segment and sort
            insert_idex = insertSegment(p_seg_t);
            _roadsegment.insert(_roadsegment.begin()+insert_idex , *p_seg_t);

            //point to next segmen 
		    segment = segment->NextSiblingElement();
        }

        //check segment
        if(!CheckSegmentID())
            return false;

        return firstSegId;
    }
}