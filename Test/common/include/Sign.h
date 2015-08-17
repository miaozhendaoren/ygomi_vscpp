#pragma once
#include "Line.h"

namespace ns_database {

enum SignType_e 
{
    //TODO
    SIGN_NULL
};

class Sign
{
public:
    Sign(void);
    Sign(Point3D landMark, SignType_e signType, const Line& centerVector);
    ~Sign(void);

public:
    void    setId(int32 id);
    int32   getId() const;

    void    setReliability(int16);
    int16   getReliability(int16) const;

    void   setLandMark(const Point3D& landMark);
    void   setSignType(const SignType_e signType);
    void   setCenterVector(const Line& centerVector);

    Point3D    getLandMark() const;
    SignType_e getSignType() const;
    Line       getCenterVector() const;

public:
    int32      mId;
    Point3D    mLandMark;    
    SignType_e mSignType;
    int16      mReliability;
    Line       mCenterVector;
};

}
