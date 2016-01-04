
#pragma once
#include <stdio.h>   
#include "typeDefine.h"
#include "database.h"
#include "apiDataStruct.h"

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
    #define MAX_NUM_SEGMENT 200
	#define MAX_NUM_POINT 200
    #define MAX_NUM_LANE 10
    #define MAX_NUM_LINE 20
    #define SUCC  1
    #define FAIL  0
    #define MERGE 1
	#define MERGE_NOT 0
    #define STARTPOINT 0
	#define ENDPOINT  1
	#define MAX_SECTION_EXT_LENTH 50
    #define MAX_SECTION_EXT_LENTH_SHORT 5
	#define LEN_RATE 0.4
	#define SEGMENT_USED   1
	#define SEGMENT_UNUSED 0
    #define POINT_USED   1
	#define POINT_UNUSED 0	
    #define GPS_REFERENCE_LAT  48.350662000000000
	#define GPS_REFERENCE_LON  11.733637999999999
	#define GPS_REFERENCE_ALT  0
	#define LEFT_LINE_TYPE_SHIFT (4)
	#define REAL_LINE_CONNECTION 0
	#define VIRTUAL_LINE_CONNECTION 1
	#define CONN_BIT_WIDE           4
	#define BIT0   0x01
	#define BIT1   0x02
	#define BIT2   0x04
	
	enum LINETYPE
	{
		DASH = 0x0, 
		SOLID = 0x1, 
		INVALID = 0x2
	};

	enum LINETYPE_MIX
	{
		DASH_DASH = 0x00,
		DASH_SOLID = 0x01,
		SOLID_DASH = 0x10,
		SOLID_SOLID = 0x11,
		DASH_INVALID = 0x02,
		SOLID_INVALID = 0x12,
		INVALID_DASH = 0x20,
		INVALID_SOLID = 0x21,
		INVALID_INVALID = 0x22
	};

	typedef struct _elementoflineconn
	{
		uint32	segId;	  
		uint32 	lineId;
		uint32 	lineConnType;//0 : real connection  1: virtual connection
	} ele_lineConn;

    struct crossing_record_t
    {
            uint32 old_id;
            uint32 new_id;
            vector<uint32> old_conn;
    };

    class Segmentpoint
    {
    public:
        Segmentpoint();
        uint32 pointId;
        uint8 point_type;
		uint8 point_type_extern;
		uint8 point_type_extern_stort;
		uint32 conn_segNum;
		vector<uint32> conn_segId;
        point3D_t point;
		point3D_t point_extern;
    };

    class Segmentlane
    {
    public:
        Segmentlane();
        uint8  laneId;		  
		bool  derection;
		uint8  merge_left;
		uint8  merge_right;
		uint8  start_pos_ID;
		uint8  end_pos_ID;
		uint8  leftlineID;
		uint8  rightlineID; 
    };

    class Segmentline
    {
    public:
        Segmentline();
		uint8 lineId;
		uint8 linetype;
		uint32 connSegNumofLine;
		vector<ele_lineConn> line_conn;
    };

    class RoadSegment
    {
    public:
        RoadSegment();
        uint32 segId;
		vector<uint32> crossing_seg;
        segment_type_e segType;
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

	typedef struct _lineConn
	{
	    uint32   SegID;	   // segment ID of current line
	    uint32   LineID;	   // current line ID
	    uint32   connSegID;  // matched line ID   
	    uint32   connLineID;	// segment ID of matched line
	    uint32   connType;
	}lineConn;
	
    class All_RoadSegment
    {
    public:		
		All_RoadSegment()
        {
            _segmentnum = 0;
			memset(&_reference_point,0,sizeof(point3D_t));
			if(!_roadsegment.empty())
            {_roadsegment.clear();}
        } 
		
        bool AnalysisSegment(const char *path);//analysis XML to get road info
		uint32 getSegmentID(uint32 index);
        uint32 getgpsnum(uint32 seg_id);
		int32  getSegmentIndex(uint32 SegId);
        point3D_t* getgps(uint32 seg_id,uint8 gpsid);
        void getRefGPS(point3D_t &);
        /******tan yinhong need fuction******/
		/* In : SegId, Out: the connect info of all lines in current section. 
			   if a line in current section has no next matched line, then push an empty vector into the list.
					 if a line's matched lines is more than one, then its vector has multiple elements */
		//bool getPrevSegId(int currSegId, vector<int> &prevSegId);	
		//bool getNextSegId(int currSegId, vector<int> &nextSegId);	 
		bool getLeftOrRightneighbourLineId(int segId, int overline, int conSegId, int turnTo, int &conLineL, int &conLineR);
		bool getConnectedLineId(int segId, int lineL, int lineR, int conSegId, int &conLineL, int &conLineR);
		bool getMatchedLinesInfo(int SegId, list<vector<lineConn>> &);
		bool getMatchedLinesInfoOfStartPoint(int SegId, list<vector<lineConn>> & , int type);
		bool getMatchedLinesInfoOfEndPoint(int SegId, list<vector<lineConn>> & , int type);

		bool getPrevSegId(uint32 segmentId , vector<uint32> &prevec);
		bool getNextSegId(uint32 segmentId , vector<uint32> &nextvec);		
		bool getSegmentType(uint32 seg_id , segment_type_e &);
		bool getBodyPoint(uint32 segmentId , uint32 type , point3D_t &point);
		/******xshao need function******/
		bool getAllSegIdOrder(vector<int> &);		
		uint32 getStartSegID();
		int  getSegNum();  //-1 means error
		bool getSegRangPoint(uint32 SegId, vector<point3D_t> &);  //
		bool getAllSegRangPointOrder(list<vector<point3D_t>> &); //segment order get the four point, overlap start, body start, body end, overlap end
		int32 getLaneNumInSeg(uint32 SegId,bool revDirect); //get how many lanes in each direction(true means reverse direction, false means forward direction)
		int32 getLaneNumInSeg(uint32 SegId);                //get how many lanes
		//in each direction(true means reverse direction, false means forward direction), 0,1,2,3 means lane index, mismatch, return -1  
		bool getLineIdOfCurrentLane(int segId, int laneIndex, bool revDirect, int &lineL, int &lineR);
        bool getLineTypeOfCurrentLane(int segId, int laneIndex, bool revDirect, int &typeL, int &typeR);
		bool getMatchedLaneIdx(int SegId, int  LlineId, int  RlineId, bool revDirect, int &laneIndex); 
		bool getMatchedLaneIdx(int SegId, LINETYPE_MIX currentLaneType, bool revDirect, vector<int>&); 
		bool getMatchedLaneIdx(int SegId, LINETYPE_MIX currentLaneType, LINETYPE_MIX neigbourLaneType, bool revDirect, vector<int>&);
		/* it will return the forward direction and backward direction will merged or not
		    and it only are used under the condiftion that is lane with diff dircetion can not mixed
             */
		bool getBothSideMergeFlag(int SegId);      
		void getRoadSegCfg_database(list<segAttributes_t> &segCfgList);//get cfg for database
		bool getRoadLoopSegId(list<vector<uint32>> &loop_segList);//get loop segment id
		void getRoadSegmentInSky(vector<uint32> &SegmentInSky); //get all the crossing segment id in the sky
        void setRoadLoopFlag(std::list<std::vector<uint32>> &loopseg_List, std::list<segAttributes_t> &segCfgList); //set loop flag to seglist 
		/******Xu Qian need function******/
		bool getsecPointInfo(vector<secPointInfo_t> &PointInfo);//get points Info
		bool getSecCfgInfo(vector<secCfgInfo_t> &SegInfo);// get segments info
		bool getsecPointVector(vector<point3D_t> &Point_vector);//get point (lat,lon)
		bool getflyCrossId(uint32 flyoverSecId,vector<uint32> &flyCrossSecId);// get the matched flyover segment id
		bool getLoopPointInfo(uint32 loopId , vector<uint32> &PointInfo_loop);
	private:
		bool CheckSegmentID();
		bool CheckSegmentNum(uint32 couter);
	    uint32 insertSegment(RoadSegment* pinsert); //insert segment Id by increased
		void GenerateSecCfgInfo();	//generate _segmentInfo	
		bool GeneratsecPointInfo(); // generate _secPointInfo
		bool RearrangeSegmentId();//rearrange all the segId,and make segId continuous
		void coordinateChange(point3D_t in, point3D_t ref, point3D_t &out); 
		void AllSecPointConvert(point3D_t &point);//convert lat,lon to the standard point
        void CalcExtendPoint();//calc all the extern points
        void ArrangePointId(); // arrange the pointId by increased
	    void loop_generate_DFS(uint32 *visit , int pos , vector<uint32> &loop_seg);//DEEP FIRST,generate a loop
	    void bubble_sort(vector<uint32> &input);
		bool getExcatMatchedMainLaneIdx(int SegId, LINETYPE_MIX currentLaneType, bool revDirect, vector<int>&laneIndex);
		bool getExcatMatchedMainLaneAndSideLaneIdx(int SegId, LINETYPE_MIX currentLaneType, LINETYPE_MIX neigbourLaneType, bool revDirect, vector<int>&laneIndex);
		bool getRoadLoopSegId_In(list<vector<uint32>> &loopseg_List);
		bool setLoopPointInfo();//set loop point  
		
        uint32 _segmentnum;
		point3D_t _reference_point;
        vector<RoadSegment> _roadsegment;
		vector<secPointInfo_t> _secPointInfo;
		vector<point3D_t> _points_v;
		vector<secCfgInfo_t> _segmentInfo;
		//for DFS 
		vector<vector<uint32>> _segLinkInfo_DFS;

		//store the loopseg List,when intialize
		list< vector<uint32> > _loopseg_ListInfo;
		//store the loopseg List of point ,when intialize
		list< vector<uint32> > _looppoint_ListInfo;
    };
}

