//#include <stdio.h>
#include <math.h>
#include "log.h"
#include "typeDefine.h"
#include "DataBaseMathToolkit.h"





using namespace ns_database;

namespace ns_databasemanager {
    DataBaseMathToolkit::DataBaseMathToolkit(void) {}
    DataBaseMathToolkit::~DataBaseMathToolkit(void) {}

    /*
    *Calculate distance between Two gps Points named M and N;
    */
    void DataBaseMathToolkit::calcDistanceBetweenTwoPoints(IN const Point3D& gpsM, IN const Point3D& gpsN, OUT double& distInMeter) {
        WARN_IF(fabs(gpsN.getLatitude() - gpsM.getLatitude()) > 2);
        double distY = (gpsN.getLatitude() - gpsM.getLatitude()) * COEFF_DD2METER_ON_LON;
        double distX = (gpsN.getLongitude() - gpsM.getLongitude()) * COEFF_DD2METER_ON_LAT(gpsM.getLatitude());
        distInMeter = sqrt(distX * distX + distY * distY);
    }

     /*
    *Calculate a Lane to a polygon in CCS which put the standPoint as origin point;
    */
    void calcLaneToCCSPolygon(IN const Point3D& standPoint, IN const Lane& lane, OUT std::vector<PointInCCS>& polygon) {
    }

    /*
    *Calculate the distance between a Point and a Paint, it will output the minimum distance and maximum distance;
    */
    void calcDistanceBetweenPointAndPaint(IN const Point3D& standPoint, IN const Paint& paint, OUT uint32& miniDistance, OUT uint32& maxDistance) {
    }

    /*
    *Put the standPoint(GPS point) as origin point (0,0,0)in (Three-dimensional) Cartesian Coordinate System,
    *convert the changePoint(GPS point) to a point in Cartesian Coordinate System.
    */
    void DataBaseMathToolkit::calcGpsPointToCCSPoint(IN const Point3D& standPoint, IN const Point3D& changePoint, OUT PointInCCS& outPoint)
    {
        WARN_IF(fabs(changePoint.getLatitude() - standPoint.getLatitude()) > 2);
        double x = (changePoint.getLongitude() - standPoint.getLongitude()) * COEFF_DD2METER_ON_LAT(standPoint.getLatitude());
        double y = (changePoint.getLatitude() - standPoint.getLatitude()) * COEFF_DD2METER_ON_LON;
        double z = changePoint.getAltitude() - standPoint.getAltitude();

        outPoint.setX(x);
	    outPoint.setY(y);
	    outPoint.setZ(z);
    }

}
