#include "Surface.h"
namespace ns_database {

Surface::Surface(void)
    :mId(0)
{
}

Surface::Surface(const std::vector<SurfaceBoundary>& surfaceBoundaries)
    :mId(0), mSurfaceBoundaries(surfaceBoundaries)
{
}

Surface::~Surface(void)
{
} 

void  Surface::setId(int32 id) {
    mId = id;
}

int32 Surface::getId() const  {
    return mId;
}

void Surface::setSurfaceBoundaries(const std::vector<SurfaceBoundary>& surfaceBoundaries) {
    mSurfaceBoundaries.clear();
    mSurfaceBoundaries = surfaceBoundaries;
}

std::vector<SurfaceBoundary> Surface::getSurfaceBoundaries() const {
    return mSurfaceBoundaries;
}

}

