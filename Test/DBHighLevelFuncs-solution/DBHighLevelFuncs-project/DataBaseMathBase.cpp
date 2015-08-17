#include <math.h>
#include "log.h"
#include "DataBaseMathBase.h"

namespace ns_databasemanager {

    DataBaseMathBase::DataBaseMathBase() {}
    DataBaseMathBase::~DataBaseMathBase() {}

        /*
    * brief: judge if a point in the line MN
    * param: point
    * param: pointM
    * param: pointN
    * return -1: not in Line;
    *         0: on ray started from M
    *         1: on line segment MN (include PointM and PointN);
    *         2: on ray started from N
    */
    int DataBaseMathBase::pointInLine(const PointInCCS& point, const PointInCCS& pointM, const PointInCCS& pointN) {
		float x = point.getX();
		float y = point.getY();
		float mx = pointM.getX();
		float my = pointM.getY();
		float nx = pointN.getX();
		float ny = pointN.getY();
		float distMN = sqrt((mx - nx)*(mx - nx) +(my - ny)*(my - ny));
        float distPM = sqrt((mx - x)*(mx - x) +(my - y)*(my - y));
		float distPN = sqrt((nx - x)*(nx - x) +(ny - y)*(ny - y));

		if(floatEqual((distPM+distPN),distMN)){
			return 1;
		}else if(floatEqual((distPM+distMN),distPN)){
			return 0;
		}else if(floatEqual((distPN+distMN),distPM)){
			return 2;
		}else{
			return -1;
		}
    }

    /*
    * brief: judge if the origin point in the polygon
    * param: polygon
    * return true: in polygon
    *        false: not in polygon
    */
    bool DataBaseMathBase::originInPolygon(const std::vector<PointInCCS>& polygon) {
        PointInCCS point(0, 0 ,0);
        return pointInPolygon(point, polygon);
    }

    /*
    * brief: judge if a point in the polygon
    * param: point
    * param: polygon
    * return true: in polygon
    *        false: not in polygon
    */
    bool DataBaseMathBase::pointInPolygon(const PointInCCS& point,const std::vector<PointInCCS>& polygon) {
        bool oddNodes = false;

	    int size = polygon.size();
	    int i = 0;
	    int j = size - 1;

        for(i = 0; i < size; i ++){
            const PointInCCS& pi = polygon[i];
		    const PointInCCS& pj = polygon[j];
            if(1 == pointInLine(point, pi, pj)) {
                return true;
            }

            float x = point.getX();
            float y = point.getY();
            float xi = pi.getX();
            float yi = pi.getY();
            float xj = pj.getX();
            float yj = pj.getY();

            if((y > yi && y <= yj) || (y > yj && y <=yi)) {
                if((xi + (y-yi)/(yj-yi)*(xj-xi)) > x) {
                    oddNodes=!oddNodes;
                }
            }

            j = i;
        }

        return oddNodes;
    }

    bool DataBaseMathBase::floatEqual(float f1 , float f2){
	    return fabs(f1 - f2) <= 0.01;
    }
}
