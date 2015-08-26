/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  databaseServer.h
* @brief Header file for database in vehicle side
*
* Change Log:
*      Date                Who             What
*      2015/1/7           Linkun Xu       Create
*******************************************************************************
*/

#ifndef DATABASE_INVEHICLE_H
#define DATABASE_INVEHICLE_H

#include "database.h"

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <windows.h>

#include "databaseDef.h"

namespace ns_database
{
    class furAttributesInVehicle_t : public furAttributes_t
    {
    };

    class databaseInVehicle : public database
    {
    public:
        databaseInVehicle(std::string inFileStr);
        databaseInVehicle();
        ~databaseInVehicle();

        void initDb();

        void getNearPointOnVector(IN point3D_t* gpsCurrP, 
                                  OUT uint32 *minSegId, 
                                  OUT int *minPointIdx, 
                                  OUT double *minDist);

        void getFurnitureByGps(IN  point3D_t*  gpsP, 
							   IN  double distThresh,
                               OUT std::list<furAttributesInVehicle_t>& furnitureAttrList);

        void getFurnitureById(IN  uint32 segmentIdIn, 
                              IN  uint32 furnitureIdIn,
                              OUT uint8* existFlag, 
                              OUT furAttributesInVehicle_t* furnitureAttr);

        void getFurnitureBySegId(IN  uint32 segmentIdIn, 
							     OUT std::list<furAttributesInVehicle_t>& furnitureAttrList);

        void getAllFurnitures(OUT std::list<std::list<furAttributesInVehicle_t>>& furnitureListOut);

        void getLookAheadView(IN point3D_t* gpsCurr, IN float distanceIn, OUT point3D_t* gpsAhead);

		void getLookAheadFurnitures(IN point3D_t* gpsCurrP, 
                                    IN float distanceInAhead, 
                                    IN float distanceInBack, 
                                    OUT std::list<furAttributesInVehicle_t>& furnitureAttrList, 
                                    OUT std::list<point3D_t>& pointInRangeList);

        void addFurniture(IN furAttributesInVehicle_t* furnitureIn);

        void addFurnitureTlv(IN uint8* tlvBuff, 
                             IN uint32 buffLen);

        void addFurnitureListTlv(IN uint8* tlvBuff, IN uint32 buffLen);

		void reduceFurnitureTlv(IN uint8* tlvBuff, 
                                IN uint32 buffLen);

		void reduceFurnitureByFurId(IN  furAttributesInVehicle_t* furnitureAttr);

		void resetFurniture();

		void getLaneGpsTlv( 
					IN std::list<laneType_t> *laneInfo,
					IN resource_e sourceFlag, 
					OUT void** output, 
					OUT int32* length);
		void resetAllVectors(void);

        bool checkDbCompleteByGps(IN point3D_t *gpsIn);

    private:

        std::list<std::list<furAttributesInVehicle_t>> _furnitureList; // segment list / furniture element

    };
}


#endif
