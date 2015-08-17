#include "DifferencePaint.h"

namespace ns_database {

DifferencePaint::DifferencePaint(void)
    :mId(0), mDifferenceId(0), mActionType(CREATE)
{
}

DifferencePaint::DifferencePaint(Paint paint, int32 differenceId, ActionType_e actionType)
    :Paint(paint), mId(0), mDifferenceId(differenceId), mActionType(actionType)
{
}

DifferencePaint::~DifferencePaint(void)
{
}

int32 DifferencePaint::getId() const {
    return mId;
}

int32 DifferencePaint::getDifferenceId() const {
    return mDifferenceId;
}

ActionType_e DifferencePaint::getActionType() const {
    return mActionType;
}

void DifferencePaint::setId(int32 id) {
    mId = id;
}

void DifferencePaint::setDifferenceId(int32 differenceId) {
    mDifferenceId = differenceId;
}

void DifferencePaint::setActionType(ActionType_e actionType) {
    mActionType = actionType;
}

}
