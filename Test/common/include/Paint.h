#pragma once
#include "Surface.h"


struct Color_t
{
    uint8 r;
    uint8 g;
    uint8 b;
};

namespace ns_database {

class Paint
{
public:
    Paint(void);
    Paint(Point3D landMark, Color_t color, const Surface& surface);
    ~Paint(void);

public:
    void setId(int32 id);
    int32 getId() const ;

    void setLandMark(const Point3D& landMark);
    void setColor(const Color_t color);
    void setSurface(const Surface& surface);

    Point3D getLandMark() const ;
    Color_t getColor() const ;
    Surface getSurface() const;
private:
    int32   mId;
    Point3D mLandMark;
    Color_t mColor;
    Surface mSurface;
};

}
