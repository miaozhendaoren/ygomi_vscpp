#pragma once
#include "typeDefine.h"

namespace ns_database {

class Point3D
{
public:
    Point3D(void);
    Point3D(float lon, float lat, float alt);
    ~Point3D(void);


public:
    void setId(int32 id);
    int32  getId() const;

    void setLongitude(float lon);
    void setLatitude(float lat);
    void setAltitude(float alt);

    float getLongitude() const;
    float getLatitude() const;
    float getAltitude() const;

    //Point3D& operator=(const Point3D&);
    void dump() const;
private:
    int32 mId;
    float mLon;
    float mLat;
    float mAlt;
};

}
