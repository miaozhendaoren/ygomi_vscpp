#pragma once
#include <vector>
#include "typeDefine.h"
#include "Sign.h"
#include "Paint.h"
#include "Lane.h"
#include "DataBaseMathToolkit.h"
#include "DifferenceLane.h"
#include "DifferencePaint.h"
#include "DifferenceSign.h"

using namespace ns_database;

namespace ns_databasemanager{

    bool insertSignToMasterDB(Sign& );
    bool insertPaintToMasterDB(Paint& );
    bool insertLaneToMasterDB(Lane& );

    bool insertSignsToMasterDB(std::vector<Sign>& );
    bool insertPaintsToMasterDB(std::vector<Paint>& );
    bool insertLanesToMasterDB(std::vector<Lane>& );

    bool deletePaintInMasterDB(int32 paintID);
    bool deleteSignInMasterDB(int32 signID);
    bool deleteLaneInMasterDB(int32 laneID);

    bool deleteAllPaintsInMasterDB();
    bool deleteAllSignsInMasterDB();
    bool deleteAllLanesInMasterDB();

    bool updateSignInMasterDB(const Sign&);
    bool updatePaintInMasterDB(const Paint&);
    bool updateLaneInMasterDB(const Lane&);

    bool getPaintFromMasterDB(IN int32 id, OUT Paint&);
    bool getSignFromMasterDB(IN int32 id, OUT Sign&);
    bool getLaneFromMasterDB(IN int32 id, OUT Lane&);

    bool getPaintsAroundFromMasterDB(IN const Point3D& location, IN int radius, OUT std::vector<Paint>& );

    bool getSignsAroundFromMasterDB(IN const Point3D& location, IN int radius, OUT std::vector<Sign>&);
    bool getSignsAlongTheWayFromMasterDB(IN const Point3D& location, IN const Point3D& preposition, IN int distance, OUT std::vector<Sign>&);

    bool getLanesBelongedFromMasterDB(IN const Point3D& location, OUT std::vector<Lane>&);
    bool getLanesAroundFromMasterDB(IN const Point3D& location, OUT std::vector<Lane>&);
    bool getLanesAroundFromMasterDB(IN int32 laneId, OUT std::vector<Lane>&); 
    bool getNextLaneFromMasterDB(IN const Point3D& location, IN const Point3D& prePosition, OUT Lane&);

    bool getRelativePaintsFromMasterDB(IN const DifferencePaint diffPaint, OUT std::vector<Paint>&);
    bool getRelativeSignsFromMasterDB(IN const DifferenceSign diffSign, OUT std::vector<Sign>&);
    bool getRelativeLanesFromMasterDB(IN const DifferenceLane diffLane, OUT std::vector<Lane>&);

    //-Master DB-

    //+Difference DB+
    bool insertPaintToDifferenceDB(DifferencePaint&);
    bool insertSignToDifferenceDB(DifferenceSign&);
    bool insertLaneToDifferenceDB(DifferenceLane&);

    bool getAllPaintsFromDifferenceDB(std::vector<Paint>& );
    bool getAllSignsFromDifferenceDB(std::vector<Sign>& );
    bool getAllLanesFromDifferenceDB(std::vector<Lane>& );

    bool deletePaintInDifferenceDB(int32);
    bool deleteSignInDifferenceDB(int32);
    bool deleteLaneInDifferenceDB(int32);
    //-Difference DB-
}
