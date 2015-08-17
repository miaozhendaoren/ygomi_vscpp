#include "Difference.h"
namespace ns_database {

Difference::Difference(void)
    :mId(0), mDifferenceId(0), mVehicleId(0)
{
}

Difference::Difference(int32 differenceId, int32 vehicleId, Date_t date, Time_t time)
    :mId(0), mDifferenceId(differenceId), mVehicleId(vehicleId), mDate(date), mTime(time)
{
}

Difference::~Difference(void)
{
}

int32 Difference::getId() const {
    return mId;
}

int32 Difference::getDifferenceId() const {
    return mDifferenceId;
}

int32 Difference::getVehicleId() const {
    return mVehicleId;
}

Date_t Difference::getDate() const {
    return mDate;
}

Time_t Difference::getTime() const {
    return mTime;
}

void Difference::setId(int32 id) {
    mId = id;
}

void Difference::setDifferenceId(int32 differenceId) {
    mDifferenceId = differenceId;
}

void Difference::setVehicleId(int32 vehicleId) {
    mVehicleId = vehicleId;
}

void Difference::setDate(Date_t date) {
    mDate = date;
}

void Difference::setTime(Time_t time) {
    mTime = time;
}

}
