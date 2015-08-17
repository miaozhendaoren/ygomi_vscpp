
#include "DifferenceSign.h"

namespace ns_database {

DifferenceSign::DifferenceSign(void)
    :mId(0), mDifferenceId(0), mActionType(CREATE)
{
}

DifferenceSign::DifferenceSign(Sign sign, int32 differenceId, ActionType_e actionType)
    :Sign(sign), mId(0), mDifferenceId(differenceId), mActionType(actionType)
{
}

DifferenceSign::~DifferenceSign(void)
{
}

int32 DifferenceSign::getId() const {
    return mId;
}

int32 DifferenceSign::getDifferenceId() const {
    return mDifferenceId;
}

ActionType_e DifferenceSign::getActionType() const {
    return mActionType;
}

void DifferenceSign::setId(int32 id) {
    mId = id;
}

void DifferenceSign::setDifferenceId(int32 differenceId) {
    mDifferenceId = differenceId;
}

void DifferenceSign::setActionType(ActionType_e actionType) {
    mActionType = actionType;
}

}
