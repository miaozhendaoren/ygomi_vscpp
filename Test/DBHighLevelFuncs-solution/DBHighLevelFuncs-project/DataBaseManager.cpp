#include "DataBaseManager.h"
#include "log.h"
#include "dbtools.h"

using namespace ns_database;

namespace ns_databasemanager {
    //+Master DB+
    bool insertSignToMasterDB(Sign& sign) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return (0 != pMasterDBTools->insertSign(sign));
    }

    bool insertPaintToMasterDB(Paint& paint) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return (0 != pMasterDBTools->insertPaint(paint));
    }

    bool insertLaneToMasterDB(Lane& lane) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return (0 != pMasterDBTools->insertLane(lane));
    }

    bool insertSignsToMasterDB(std::vector<Sign>& signs) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->insertSigns(signs);
    }

    bool insertPaintsToMasterDB(std::vector<Paint>& paints) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->insertPaints(paints);
    }

    bool insertLanesToMasterDB(std::vector<Lane>& lanes) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->insertLanes(lanes);
    }

    bool deletePaintInMasterDB(int32 paintID) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->deletePaintById(paintID);
    }

    bool deleteSignInMasterDB(int32 signID) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->deleteSignById(signID);
    }

    bool deleteLaneInMasterDB(int32 laneID) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->deleteLaneById(laneID);
    }

    bool deleteAllPaintsInMasterDB() {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->deleteAllPaints();
    }

    bool deleteAllSignsInMasterDB() {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->deleteAllSigns();
    }

    bool deleteAllLanesInMasterDB() {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->deleteAllLanes();
    }

    bool updateSignInMasterDB(const Sign& sign) {
        //TODO
        return false;
    }

    bool updatePaintInMasterDB(const Paint& paint) {
        //TODO
        return false;
    }

    bool updateLaneInMasterDB(const Lane& lane) {
        //TODO
        return false;
    }

    bool getPaintFromMasterDB(IN int32 id, OUT Paint& outPaint) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->getPaintById(id, outPaint);
    }

    bool getSignFromMasterDB(IN int32 id, OUT Sign& outSign) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->getSignById(id, outSign);
    }

    bool getLaneFromMasterDB(IN int32 id, OUT Lane& outLane) {
        ns_dbmaster::DBTools* pMasterDBTools = ns_dbmaster::DBTools::getInstance();
        if(NULL == pMasterDBTools) {
            LOGE("Get MasterDBTools fail!");
            return false;
        }
        return pMasterDBTools->getLaneById(id, outLane);
    }

    bool getPaintsAroundFromMasterDB(IN const Point3D& location, IN int radius, OUT std::vector<Paint>& outPaints) {
        //TODO
        return false;
    }

    bool getSignsAroundFromMasterDB(IN const Point3D& location, IN int radius, OUT std::vector<Sign>& outSigns) {
        //TODO
        return false;
    }

    bool getSignsAlongTheWayFromMasterDB(IN const Point3D& location, IN const Point3D& preposition, IN int distance, OUT std::vector<Sign>& outSigns) {
        //TODO
        return false;
    }

    bool getLanesBelongedFromMasterDB(IN const Point3D& location, OUT std::vector<Lane>& outLanes) {
        //TODO
        return false;
    }

    bool getLanesAroundFromMasterDB(IN const Point3D& location, OUT std::vector<Lane>& outLanes) {
        //TODO
        return false;
    }

    bool getLanesAroundFromMasterDB(IN int32 laneId, OUT std::vector<Lane>& outLanes) {
        //TODO
        return false;
    }

    bool getNextLaneFromMasterDB(IN const Point3D& location, IN const Point3D& prePosition, OUT Lane& outLane) {
        //TODO
        return false;
    }


    bool getRelativePaintsFromMasterDB(IN const DifferencePaint diffPaint, OUT std::vector<Paint>& outPaints) {
        //TODO
        return false;
    }

    bool getRelativeSignsFromMasterDB(IN const DifferenceSign diffSign, OUT std::vector<Sign>& outSigns) {
        //TODO
        return false;
    }

    bool getRelativeLanesFromMasterDB(IN const DifferenceLane diffLane, OUT std::vector<Lane>& outLanes) {
        //TODO
        return false;
    }

    //-Master DB-

    //+Difference DB+

    bool insertPaintToDifferenceDB(DifferencePaint&) {
        //TODO
        return false;
    }

    bool insertSignToDifferenceDB(DifferenceSign&) {
        //TODO
        return false;
    }

    bool insertLaneToDifferenceDB(DifferenceLane&) {
        //TODO
        return false;
    }

    bool getAllPaintsFromDifferenceDB(std::vector<Paint>& ) {
        //TODO
        return false;
    }

    bool getAllSignsFromDifferenceDB(std::vector<Sign>& ) {
        //TODO
        return false;
    }

    bool getAllLanesFromDifferenceDB(std::vector<Lane>& ) {
        //TODO
        return false;
    }

    bool deletePaintInDifferenceDB(int32) {
        //TODO
        return false;
    }

    bool deleteSignInDifferenceDB(int32) {
        //TODO
        return false;
    }

    bool deleteLaneInDifferenceDB(int32) {
        //TODO
        return false;
    }
    //-Difference DB-

}
