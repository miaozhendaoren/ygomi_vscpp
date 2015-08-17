#include <stdio.h>
#include "Point3D.h"

namespace ns_database {

Point3D::Point3D(void)
    :mId(0),mLon(0),mLat(0),mAlt(0)
{
}

Point3D::Point3D(float lon, float lat, float alt)
    :mId(0),mLon(lon),mLat(lat),mAlt(alt)
{
}
Point3D::~Point3D(void)
{
}

void Point3D::setId(int32 id){
    mId = id;
}

int32 Point3D::getId() const{
    return mId;
}

void Point3D::setLongitude(float lon) {
    mLon = lon;
}
void Point3D::setLatitude(float lat) {
    mLat = lat;
}
void Point3D::setAltitude(float alt) {
    mAlt = alt;
}

float Point3D::getLongitude() const {
    return mLon;
}

float Point3D::getLatitude() const {
    return mLat;
}

float Point3D::getAltitude() const {
    return mAlt;
}

/*
Point3D& Point3D::operator=(const Point3D&){
    printf("operator= for Point3D\n");
    return *this;
}
*/

void Point3D::dump() const {
    printf("Point3D: lon: %f, lat: %f, alt: %f\n", mLon, mLat, mAlt);
}

}
