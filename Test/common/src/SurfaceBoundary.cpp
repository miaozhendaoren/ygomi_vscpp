#include "SurfaceBoundary.h"
namespace ns_database {

SurfaceBoundary::SurfaceBoundary(void)
    :mId(0)
{
}

SurfaceBoundary::SurfaceBoundary(bool isClosure, bool isImputed, const Line& lineInfo)
    :Line(lineInfo), mId(0), mIsClosure(isClosure), mIsImputed(isImputed)
{
}

SurfaceBoundary::SurfaceBoundary(bool isClosure, bool isImputed, const Line& lineInfo, const std::vector<PaintID>& lineOrigin)
    :Line(lineInfo), mIsClosure(isClosure), mIsImputed(isImputed), mOrigin(lineOrigin)
{
}
    
SurfaceBoundary::~SurfaceBoundary(void)
{
}


void  SurfaceBoundary::setId(int32 id){
    mId = id;
}

int32 SurfaceBoundary::getId() const {
    return mId;
}

void  SurfaceBoundary::setClosure(bool isClosure) {
    mIsClosure =  isClosure;
}

void  SurfaceBoundary::setImputed(bool isImputed) {
    mIsImputed = isImputed;
}

void  SurfaceBoundary::setOrigin(const std::vector<PaintID>& origin) {
    mOrigin.clear();
    mOrigin = origin;
}

bool SurfaceBoundary::isClosure() const {
    return mIsClosure;
}

bool SurfaceBoundary::isImputed() const {
    return mIsImputed;
}

std::vector<PaintID> SurfaceBoundary::getOrigin() const {
    return mOrigin;
}

}
