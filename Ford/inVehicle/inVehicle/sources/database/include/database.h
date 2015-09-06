/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  database.h
* @brief Header file for database
*
* Change Log:
*      Date                Who             What
*      2015/1/7           Linkun Xu       Create
*******************************************************************************
*/

#ifndef DATABASE_H
#define DATABASE_H

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <windows.h>

#include "databaseDef.h"

namespace ns_database
{
	#define COEFF_DD2METER (111320.0)
    #define MAX_RALIABILITY 5
    #define PI  3.1415926536f

    // Basic structure
    enum resource_e : uint32
    {
        file_e,
        memory_e,
    };

    struct tlvCommon_t
    {
        typeId_e  typeId;
        uint8     usedFlag;
        tlvType_e tlvType;
        uint32    length;
        uint32    value;
    };

    struct tlvCfg_t
    {
        typeId_e  typeId;
        tlvType_e tlvType;
        uint32    length;
        uint16    startRev;
        uint16    endRev;
    };

    struct point3D_t
    {
        double lat;
        double lon;
        double alt;
        float  paintFlag;
        int    count;
    };

	struct point2D_t
    {
        double lat;
        double lon;
    };

    struct pointRelative3D_t
    {
        float x; // lon direction
        float y; // lat direction
        float z; // alt direction
    };

    struct lineAttributes_t
    {
        uint32 segmentId;
        uint32 lineId;
        float  width;
        uint8  lineStyle;
        uint32 segVersion;
        int numPoints;
		int count;
    };

    struct segAttributes_t
    {
        uint8  segId_used;//
        uint32 segId;//
        uint8  version_used;
        uint32 version;
        uint8  type_used;
        uint8  type;
        uint8  numPort_used;
        int32  numPort;
        uint8  ports_used;
        point3D_t ports[MAX_NUM_PORT];//
        uint8  links_used;
        uint32 links[MAX_NUM_PORT];
        uint8  roadLength_used;
        float  roadLength[MAX_NUM_ROADLEN];
        uint8  bridgeFlag_used;
        uint8  bridgeFlag;
        uint8  tunnelFlag_used;
        uint8  tunnelFlag;
        uint8  numFurniture_used;
        uint8  numFurniture;
        uint8  numDynamicData_used;
        uint8  numDynamicData;
        uint8  uiLaneNum_used;
        uint8  uiLaneNum;
    };
	 
    class furAttributes_t
    {
    public:
        furAttributes_t();
        ~furAttributes_t();
        void format();

        uint8  segId_used;
        uint32 segId;
        uint8  furId_used;
        uint32 furId;
        uint8  segVersion_used;
        uint32 segVersion;
        uint8  location_used;
        point3D_t location;
        uint8  angle_used;
        float  angle;
        uint8  type_used;
        uint16 type;
        uint8  side_used;
        uint32 side[2];
        uint8  sideFlag_used;
        uint8  sideFlag; // 1: right side, 2: left side, 3: both sides, 4: on the road
        uint8  offset_used;
        float  offset;
        uint8  reliabRating_used;
        uint8  reliabRating;
        uint8  boundary_used;
        std::vector<point3D_t> boundary;

    protected:
        //HANDLE _hMutexFile;
        //HANDLE _hMutexMemory;
    };

	struct	laneType_t{
		char  laneChangeFlag;
		char  linePaintFlagL;
		char  linePaintFlagR;
        char  lineStyle;
		uint32  laneId;
		float laneWidth;
		ns_database::point3D_t gpsL;
		ns_database::point3D_t gpsR;
	};

    class database
    {
    public:
        database();
        database(std::string inFileStr);
        ~database();

		double _distThreshFarFar;
        double _distThreshFar;
		double _distThreshMid;
		double _distThreshNear;

        double _angleThresh;

        // Public methods
        void readDb(IN  void** input,
                    IN  resource_e sourceFlag,
                    IN  int32 numByte = 0);

        void addSegmentTlv(IN uint8* tlvBuff, IN uint32 buffLen);

        void addAllVectorsInSegTlv(IN uint8* tlvBuff, IN uint32 buffLen);

        void getAllVectors(std::list<std::list<std::vector<point3D_t>>>& allLines, 
                                     std::list<std::list<lineAttributes_t>>& lineAttr);

        bool getAllVectorsAsync(std::list<std::list<std::vector<point3D_t>>>& allLines, 
                                std::list<std::list<lineAttributes_t>>& lineAttr);

        void getAllVectors_clear(std::list<std::list<std::vector<point3D_t>>>& allLines, 
                                 std::list<std::list<lineAttributes_t>>& lineAttr);

        void database::getAllVectorsTlv(IN resource_e sourceFlag, 
                                      OUT void** output, 
                                      OUT int32* length);

        void resetAllVectors(IN std::list<std::list<std::vector<point3D_t>>>& allLines, 
                             IN std::list<std::list<lineAttributes_t>>& lineAttr);

        void resetSegCfg(IN std::list<segAttributes_t> &segConfigList);

        void getSegmentByGps(IN  point3D_t*       gps, 
                             OUT uint8*           existFlag, 
                             OUT segAttributes_t* segmentAttr);

        void getSegmentById(IN  uint32 segmentIdIn, 
                            OUT uint8* existFlag, 
                            OUT segAttributes_t* segmentAttr);

        void convSegmentToTlv(IN segAttributes_t* segmentAttr, 
                              IN resource_e sourceFlag,
                              OUT void** output, 
                              OUT int32* length);

        void convFurnitureToTlv(IN furAttributes_t* furnitureAttr, 
                              IN resource_e sourceFlag,
                              OUT void** output, 
                              OUT int32* length);

        void getLookAheadView(IN point3D_t* gpsCurr, IN float distanceIn, OUT point3D_t* gpsAhead);

		void getLookAheadFurnitures(IN point3D_t* gpsCurrP, 
                                    IN float distanceInAhead, 
                                    IN float distanceInBack, 
                                    OUT std::list<furAttributes_t>& furnitureAttrList, 
                                    OUT std::list<point3D_t>& pointInRangeList);

        void readTlvToFurniturePublic(IN uint8* tlvBuff, 
                                      IN uint32 buffLen,
                                      OUT furAttributes_t &furAttr);
    protected:
        std::string _dbFileName;
        FILE* _dbFid;
        fpos_t _fPosition;
        void*  _mPosition;
        uint8* _mEndPosition;
        HANDLE _hMutexFile;
        HANDLE _hMutexMemory;

        static const int _numTlvPerPoint = 7;

        uint8 _dataTmpBuf[300];

        // TLV configuration info
        tlvCfg_t _tlvCfg_sec_a[secMax_e - secBase_e];
        tlvCfg_t _tlvCfg_header_a[header_max_e - header_base_e];
        tlvCfg_t _tlvCfg_seg_a[seg_max_e - seg_base_e];
        tlvCfg_t _tlvCfg_vec_a[vec_max_e - vec_base_e];
        tlvCfg_t _tlvCfg_fur_a[fur_max_e - fur_base_e];
        tlvCfg_t _tlvCfg_dataLine_a[data_lineMax_e - data_lineBase_e];
        tlvCfg_t _tlvCfg_dataPoint_a[data_pointMax_e - data_pointBase_e];

        // Database in memory
        tlvCommon_t _headerSecHeader;
        tlvCommon_t _segmentSecHeader;
        tlvCommon_t _vectorSecHeader;

        std::vector<tlvCommon_t> _header; // header element
        std::list<segAttributes_t> _segmentList; // segment list / segment element
        std::list<std::list<std::vector<point3D_t>>> _vectorList; // segment list / vector list / point list / point element
        std::list<std::list<lineAttributes_t>> _lineAttrList; // segment list / vector list
        //std::list<std::list<furAttributes_t>> _furnitureList; // segment list / furniture element

        // Private Methods
        // memory operation
        void initTlvCfg();

        void initParams();

        void initDb();

        void getTlvCfgbyId(typeId_e typeId, tlvCfg_t* tlvCfg, typeId_e* idBase);

        void getNearPointOnVector(IN point3D_t* gpsCurrP, 
                                  OUT uint32 *minSegId, 
                                  OUT int *minPointIdx, 
                                  OUT double *minDist);

        void setTvlCfg(tlvCfg_t* tlv,
                        typeId_e typeId,
                        tlvType_e tlvType,
                        uint32 length,
                        uint16 startRev,
                        uint16 endRev);

        void setTvlCommon(tlvCommon_t* tlv,
                        typeId_e typeId,
                        uint8 usedFlag,
                        tlvType_e tlvType,
                        uint32 length,
                        uint32 value);

        // methods to read dhd file
        int  read(OUT void* dstP, IN int size, IN int count, IN void** srcP, IN resource_e sourceFlag = file_e);

        int  write(OUT void** dstP, IN int size, IN int count, IN void* srcP, IN resource_e sourceFlag = file_e);

        void logPosition(IN void** input, IN resource_e sourceFlag);

        void resetPosition(IN void** input, IN resource_e sourceFlag);

        void readTlvToSegment(IN  void** input,
                              IN  resource_e sourceFlag);

        void readTlvToVector(IN  void** input,
                             IN  resource_e sourceFlag);

        void readTlvToPoint(IN  void** input,
                            IN  resource_e sourceFlag,
                            OUT point3D_t& pointElement, 
                            OUT int* numByteInBuff);

        void readTlvToFurniture(IN  void** input,
                            IN  resource_e sourceFlag,
						    OUT furAttributes_t* furnitureElement);

        void readTlv(IN  void** input,
                     IN  resource_e sourceFlag,
                     OUT tlvCommon_t* tlv, 
                     OUT typeId_e* idBase, 
                     OUT int* numByteRead, 
                     OUT uint8* dataBufP);

        void readTlvValue(IN tlvType_e tlvType, 
                          IN  void** input,
                          IN  resource_e sourceFlag,
                          OUT tlvCommon_t* tlv, 
                          OUT uint8* dataBufP);

        // methods to write dhd file
        void formatDbFile();

        void writeDbFile();

        int32 writeTlvCommon(OUT tlvCommon_t* tlv,
                             IN  void** input,
                             IN  resource_e sourceFlag);
    };

	std::string ID2Name(int target);

    bool checkGpsInRange(point3D_t* gpsA, point3D_t* gpsB, double distThreshMeter);

    bool checkRelGpsInRange(point3D_t* gpsA, point3D_t* gpsB, double distThreshMeter);

    bool checkAngleInRange(IN float angleA, IN float angleB, IN double angleThresh);

    bool checkTwoFurnitureSame(IN furAttributes_t* fur1,
                               IN furAttributes_t* fur2,
                               IN double distThreshMeter,
                               IN float angleThresh);

    bool checkTwoFurnitureSameNoRange(IN furAttributes_t* fur1,
                                      IN furAttributes_t* fur2, 
                                      IN float angleThresh);

    void calcAngle(IN point3D_t* gpsA, IN point3D_t* gpsB, OUT float* angle);

    void calcNormalAngle(IN point3D_t* gpsA, IN point3D_t* gpsB, OUT float* angle);

    void calcGpsDistance(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT double* distInMeter);

	void calcRelDistance(IN point3D_t* gpsPre, IN point3D_t* gpsCurr, OUT double* distInMeter);

    void calcRelativeLocation(IN point3D_t* standPoint, IN point3D_t* changePoint, OUT pointRelative3D_t* outPoint);

    void calcGpsFromRelativeLocation(IN point3D_t* standPoint, IN pointRelative3D_t* relPoint, OUT point3D_t* outGpsPoint);

	void calcDistancePointToLine(IN point3D_t* gpsLinePointA, 
		                            IN point3D_t* gpsLinePointB, 
									IN point3D_t* gpsPointC, 
							 		OUT double* distInMeter);
		
	void calcProjectPointOnLine(IN point3D_t* gpsLinePointMP, 
								IN point3D_t* gpsLinePointNP, 
								IN point3D_t* gpsPointPP, 
								OUT point3D_t* projectPointP);

	void calcGpsBackOnLine(IN point3D_t* gpsLinePointMP, 
							IN point3D_t* gpsLinePointNP, 
							IN double     backDist, 
							OUT point3D_t* gpsOutP);

    void calcGpsRoadSide(IN point3D_t* gpsPre, 
                         IN point3D_t* gpsCurr, 
                         IN point3D_t* gpsRef,
                         IN uint8 side, 
                         IN float widthInMeter, 
                         OUT point3D_t* gpsRight); // side: 1: right, 2: left

    int roadSideGpsGen(IN  point3D_t gpsPre, 
                       IN  point3D_t gps, 
                       IN  double distanceInMeterL,
                       IN  double distanceInMeterR,
                       OUT point3D_t *gpsOutL,
                       OUT point3D_t *gpsOutR);

}


#endif
