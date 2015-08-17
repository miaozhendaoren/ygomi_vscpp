#include "DifferenceLane.h"

namespace ns_database {

DifferenceLane::DifferenceLane(void)
    :mId(0), mDifferenceId(0), mActionType(CREATE)
{
}

DifferenceLane::DifferenceLane(Lane lane, int32 differenceId, ActionType_e actionType)
    :Lane(lane), mId(0), mDifferenceId(differenceId), mActionType(actionType)
{
}

DifferenceLane::~DifferenceLane(void)
{
}

int32 DifferenceLane::getId() const {
    return mId;
}

int32 DifferenceLane::getDifferenceId() const {
    return mDifferenceId;
}

ActionType_e DifferenceLane::getActionType() const {
    return mActionType;
}

void DifferenceLane::setId(int32 id) {
    mId = id;
}

void DifferenceLane::setDifferenceId(int32 differenceId) {
    mDifferenceId = differenceId;
}

void DifferenceLane::setActionType(ActionType_e actionType) {
    mActionType = actionType;
}

}
