
#pragma once
#include <stdio.h>   
#include "typeDefine.h"
#include "database.h"

using std::vector;
using namespace std;
using namespace ns_database;
/*
   segmentID , laneID , lineId , pintId should be continuous in the XML,
   and can be checked in the check func. All of them are incremental.
   The laneID and lineID count from 1. In the vector and would be orderd 
   when malloc the space. 
*/
namespace ns_roadsegment
{
    #define MAX_NUM_SEGMENTNUM 200
    #define MAX_NUM_LANE 10
    #define MAX_NUM_LINE 20
    #define SUCC  1
    #define FAIL  0

    class Segmentpoint
    {
    public:
        Segmentpoint();
        uint8 pointId;
        uint8 point_type;
        uint8 conn_segId;
        point3D_t point;
    };

    class Segmentlane
    {
    public:
        Segmentlane();
        uint8  laneId;		  
		uint8  derection;
		uint8  merge_left;
		uint8  merge_right;
		point3D_t start_pos;
		point3D_t end_pos;
		uint8  leftlineID;
		uint8  rightlineID; 
    };

    class Segmentline
    {
    public:
        Segmentline();
		uint8 lineId;
		uint8 linetype;
		uint8  connLineID;
		uint32 connSegID;
    };

    class RoadSegment
    {
    public:
        uint32 segId;
        uint8  segType;
        uint8  laneNum;
		uint8  lineNum;
		uint8  pointNum;

		vector<Segmentpoint> _segpoint;
		vector<Segmentlane> _seglane;
        vector<Segmentline> _segline;

	    uint8 getLaneNum();
        uint32 insertLane(Segmentlane* pinsert);
        uint32 insertLine(Segmentline* pinsert);
        uint32 insertPoint(Segmentpoint* pinsert);
        bool CheckLaneId();
        bool CheckLineId();
        bool CheckPointId();  
    };

    class All_RoadSegment
    {
    public:		
        uint32 getSegmentNum();
		uint32 getSegmentID(uint32 index);
		uint32 getLaneofSegNum(uint32 seg_id);
        uint32 getgpsnum(uint32 seg_id);
        point3D_t* getgps(uint32 seg_id,uint8 gpsid);
        uint32 insertSegment(RoadSegment* pinsert);
        bool CheckSegmentID();
        uint32 AnalysisSegment();
    private:
        uint32 segmentnum;
        vector<RoadSegment> _roadsegment;
    };
}

