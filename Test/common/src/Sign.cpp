#include "Sign.h"

namespace ns_database {

Sign::Sign(void)
    :mId(0),mSignType(SIGN_NULL), mReliability(0)
{
}

Sign::Sign(Point3D landMark, SignType_e signType, const Line& centerVector)
    :mId(0), mLandMark(landMark), mSignType(signType), mCenterVector(centerVector), mReliability(0)
{
}

Sign::~Sign(void)
{
}

void Sign::setId(int32 id) {
    mId = id;
}
int32 Sign::getId() const {
    return mId;
}

void Sign::setReliability(int16 reliability) {
    mReliability = reliability;
}

int16 Sign::getReliability(int16) const {
    return mReliability;
}

void  Sign::setLandMark(const Point3D& landMark) {
    mLandMark = landMark;
}

void  Sign::setSignType(const SignType_e signType) {
    mSignType = signType;
}

void  Sign::setCenterVector(const Line& centerVector) {
    mCenterVector = centerVector;
}

Point3D Sign::getLandMark() const {
    return mLandMark;
}


SignType_e Sign::getSignType() const {
    return mSignType;
}

Line Sign::getCenterVector() const {
    return mCenterVector;
}

}
