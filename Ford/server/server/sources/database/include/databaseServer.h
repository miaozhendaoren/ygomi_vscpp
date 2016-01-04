/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  databaseServer.h
* @brief Header file for database on server side
*
* Change Log:
*      Date                Who             What
*      2015/1/7           Linkun Xu       Create
*******************************************************************************
*/

#ifndef DATABASE_SERVER_H
#define DATABASE_SERVER_H

#include "database.h"

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <windows.h>

#include "databaseDef.h"
#include "apiDataStruct.h" // backgroundSectionData

namespace ns_database
{
    class furAttributesServer_t : public furAttributes_t
    {
    public:
        furAttributesServer_t(furAttributes_t* furAttrIn);
        furAttributesServer_t();
        ~furAttributesServer_t();

        void update(INOUT furAttributesServer_t* furAttrIn);
        void reduce(OUT furAttributesServer_t* furAttrIn);
        void format();
        bool checkFurId(uint32 furIdIn);
        uint32 getFurIdMaxIncrease();

    private:
        const static int MAX_HISTORY_NUM = 20;

        void calcReliability();
        void calcLocation();

        volatile static uint32 _furIdMax;

        std::list<bool> _detectedFlagHistory;
        std::list<point3D_t> _locationHistory;
    };

    class databaseServer : public database
    {
    public:
        databaseServer();
        databaseServer(std::string inFileStr);
        ~databaseServer();

        void initDb();

        void getFurnitureByGps(IN  point3D_t*  gpsP, 
                               IN  double distThreshMeter,
                               OUT std::list<furAttributesServer_t>& furnitureAttrList);

        void getFurnitureById(IN  uint32 segmentIdIn, 
                              IN  uint32 furnitureIdIn,
                              OUT uint8* existFlag, 
                              OUT furAttributesServer_t* furnitureAttr);

        void getFurnitureBySegId(IN  uint32 segmentIdIn, 
                                 OUT std::list<furAttributesServer_t>& furnitureAttrList);

        void getAllFurnitures(OUT std::list<std::list<furAttributesServer_t>>& furnitureListOut);

        void getSegNumOfFurniture(OUT int32 *numSegOfFur);

        void calcFurnitureHeight(IN point3D_t *locIn, IN uint8 sideFlagIn, OUT double *altOut);

        void resetSingleFurSegId(IN  furAttributesServer_t* furnitureIn, IN  uint32 newSegId);

        void resetAllFurSegId(void);

        bool resetFurnitureSideFlag(IN furAttributesServer_t* furnitureIn, IN uint32 segId);

        void addFurniture(IN furAttributes_t* furnitureIn,
                          OUT furAttributes_t* furnitureOut);

        void resetFurnitureRoadSideLoc();
        
        void resetFurnitureRoadSideLoc1();

        void resetFurnitureRoadSideLoc2();

        void resetFurnitureRoadSideLoc3();
        
        int getFurUpdateFlag();

        void setFurUpdateFlag(int flag);
    
        void resetFurUpdateFlag();

        bool getUpateMsgToTlv(OUT uint8 **tlvP, 
                            OUT uint32 *pduOutOffset, 
                            OUT std::vector<uint32> &vecPduOffset, 
                            OUT std::vector<uint32> &furPduOffset);

        void addFurnitureTlv(IN uint8* tlvBuff, 
                             IN uint32 buffLen,
                             OUT uint8*  tlvOutBuff,
                             OUT uint32* outBuffLen);

        void addFurnitureListTlv(IN uint8* tlvBuff, IN uint32 buffLen);

        void addFurnitureListAsIsTlv(IN uint8* tlvBuff, IN uint32 buffLen);

        void reduceFurnitureTlv(IN uint8* tlvBuff, 
                                IN uint32 buffLen,
                                OUT uint8*  tlvOutBuff,
                                OUT uint32* outBuffLen);

        void reduceFurnitureByFurId(IN  furAttributes_t* furnitureIn, 
                                    OUT furAttributes_t* furnitureOut);
        void readTlvToLaneInfo(IN  void** input,
                               IN  resource_e sourceFlag,
                               IN  int buffLen,
                               OUT std::list<laneType_t>* laneInfo);

        void resetFurniture();

        void mergeFurInSameRange(INOUT furAttributes_t& furnitureInOut);

        uint32 getFurnitureVersion();

        void getNewDataVec(std::list<std::vector<point3D_t>> &newDataVec);

        void getFurnitureTlvInSeg(IN  int32 segIdIn,
                                  IN  int32 maxPayloadLen,
                                  OUT uint8 *furnitureListP, 
                                  OUT int32 *msgLenOut, 
                                  OUT int32 *furNumOut);

        void setNewDataVec(std::list<std::vector<point3D_t>> &newDataVec);

        int getSegIdInFurList(int furListIndex, int *furSegId);

        bool loadFurFromFile(IN std::string fileName);

        bool saveFurToFile(IN std::string fileName);

        bool loadRoadVecFromFile(IN std::string fileName,
                                 IN bool bRevDir = false);

        bool saveRoadVecToFile(IN std::string fileName,
                               IN bool bRevDir = false);

		bool saveRoadVecAndFurToKml(IN std::list<std::list<std::vector<point3D_t>>> &allLines,
            						IN std::list<std::list<furAttributesServer_t>> &furnitureList,
									IN std::string fileName, 
									IN point3D_t &standPoint);

        void convBgRoadVecToTlv(IN std::list<backgroundSectionData> &bgVec, 
                                OUT uint8 *tlvBuf, 
                                OUT int *bufLen);

        bool convTlvToBgRoadVec(IN  uint8 *tlvBuf,
                                IN  int bufLen,
                                OUT std::list<backgroundSectionData> &bgVec);

    private:
        static const int _MAX_PAYLOAD_BYTE_NUM = (1024*1024);// 1M byte for furniture load/store furniture temp memory
        static const int _MAX_ROAD_POINT_BYTES = (1024*100000);// 100M byte for furniture load/store road vector temp memory

        std::list<std::list<furAttributesServer_t>> _furnitureList; // segment list / furniture element
        std::list<std::vector<point3D_t>> _newDataVec;
        int _furUpdateFlag;

        list<furAttributesServer_t>::iterator insertOneFurniture(IN furAttributesServer_t &furnitureServerIn);
    };
}


#endif
