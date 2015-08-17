#pragma once
#include <vector>
#include "typeDefine.h"

namespace ns_databasemanager {

/*
* define Point in (Three-dimensional) Cartesian Coordinate System
*/
class PointInCCS {
public:
    PointInCCS() :mX(0),mY(0),mZ(0){}
    PointInCCS(float x, float y, float z):mX(x), mY(y), mZ(z){}
    ~PointInCCS() {}
public:
    float getX() const {return mX;}
    float getY() const {return mY;}
    float getZ() const {return mZ;}

    void setX(float x) {mX = x;};
    void setY(float y) {mY = y;};
    void setZ(float z) {mZ = z;};
private:
    float mX;
    float mY;
    float mZ;
};



class DataBaseMathBase {
public:
    DataBaseMathBase(void);
    ~DataBaseMathBase(void);
public:
    /*
    * brief: judge if a point in the line MN
    * param: point
    * param: pointM
    * param: pointN
    * return -1: not in Line;
    *         0: on ray started from M
    *         1: on line segment MN;
    *         2: on ray started from N
    */
    static int pointInLine(const PointInCCS& point, const PointInCCS& pointM, const PointInCCS& pointN);

    /*
    * brief: judge if the origin point in the polygon
    * param: polygon
    * return true: in polygon
    *        false: not in polygon
    */
    static bool originInPolygon(const std::vector<PointInCCS>& polygon);

    /*
    * brief: judge if a point in the polygon
    * param: point
    * param: polygon
    * return true: in polygon
    *        false: not in polygon
    */
    static bool pointInPolygon(const PointInCCS& point,const std::vector<PointInCCS>& polygon);


    /*
    * brief: judge if a point in the Fan
    * param: point
    * param: fanApex
    * param: fanDirection: the direction of from point-fanDirection to point-fanApex will be the direction of fan;
    * return true: in fan
    *        false: not in the fan
    */
    static bool pointInFan(const PointInCCS& point, const PointInCCS& fanApex, const PointInCCS& fanDirection, uint32 fanRadius);

private:

    static bool floatEqual(float f1 , float f2);
};

}
