#pragma once

#include <math.h>
#include "typeDefine.h"
#include "Point3D.h"
#include "Lane.h"
#include "Paint.h"
#include "DataBaseMathBase.h"

#define PI  (3.1415926536f)

/*
*The radius of the semi-major axis of the Earth at the equator is 6,378,137.0 meters
*resulting in a circumference of 40,075,161.2 meters.
*/
#define RADIUS_EQUATOR (6378137.0)

/*
*measure
*Decimal degrees to radians
*/
#define MEASURE_DD2RADIAN(x) ((x)*PI/180)

/*
*coefficient
*On every longitude line, Decimal degrees to meters
*/
#define COEFF_DD2METER_ON_LON (111320.0)

/*
*coefficient
*On Latitude line, Decimal degrees to meters
*Latitude S ~ N: -90.0 ~ 90.0 degree
*/
#define COEFF_DD2METER_ON_LAT(lat) (2*PI*RADIUS_EQUATOR*cos(MEASURE_DD2RADIAN(lat))/360)


using namespace ns_database;

namespace ns_databasemanager {

class DataBaseMathToolkit {
public:
    DataBaseMathToolkit(void);
    ~DataBaseMathToolkit(void);

public:
    /*
    *Calculate distance between Two gps Points named M and N;
    */
	static void calcDistanceBetweenTwoPoints(IN const Point3D& gpsM, IN const Point3D& gpsN, OUT double& distInMeter);

    /*
    *Calculate a Lane to a polygon in CCS which put the standPoint as origin point;
    */
    static void calcLaneToCCSPolygon(IN const Point3D& standPoint, IN const Lane& lane, OUT std::vector<PointInCCS>& polygon);

    /*
    *Calculate the distance between a Point and a Paint, it will output the minimum distance and maximum distance;
    */
    static void calcDistanceBetweenPointAndPaint(IN const Point3D& standPoint, IN const Paint& paint, OUT uint32& miniDistance, OUT uint32& maxDistance);
private:
    /*
    *Put the standPoint(GPS point) as origin point (0,0,0)in (Three-dimensional) Cartesian Coordinate System,
    *convert the changePoint(GPS point) to a point in Cartesian Coordinate System.
    */
    static void calcGpsPointToCCSPoint(IN const Point3D& standPoint, IN const Point3D& changePoint, OUT PointInCCS& outPoint);

};

}
