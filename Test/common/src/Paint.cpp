#include "Paint.h"

namespace ns_database {
Paint::Paint(void)
    :mId(0)
{
}

Paint::Paint(Point3D landMark, Color_t color, const Surface& surface)
    :mId(0), mLandMark(landMark), mColor(color), mSurface(surface) 
{

}

Paint::~Paint(void)
{
}

void Paint::setId(int32 id) {
    mId = id;
}
int32 Paint::getId() const {
    return mId;
}

void Paint::setLandMark(const Point3D& landMark) {
    mLandMark = landMark;
}

void Paint::setColor(const Color_t color) {
    mColor = color;
}

void Paint::setSurface(const Surface& surface) {
    mSurface = surface;
}

Point3D Paint::getLandMark() const {
    return mLandMark;
}

Color_t Paint::getColor() const {
    return mColor;
}

Surface Paint::getSurface() const {
    return mSurface;
}

}
