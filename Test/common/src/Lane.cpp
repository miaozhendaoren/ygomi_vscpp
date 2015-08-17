#include "Lane.h"
namespace ns_database {

Lane::Lane(void)
    :mId(0)
{
}

Lane::Lane(const Surface& surface)
    :mId(0), mSurface(surface)
{
}


Lane::~Lane(void)
{
}


void Lane::setId(int32 id) {
    mId = id;
}

int32 Lane::getId() const {
    return mId;
}

void Lane::setSurface(const Surface& surface) {
    mSurface = surface;
}

Surface Lane::getSurface() const {
    return mSurface;
}


}
