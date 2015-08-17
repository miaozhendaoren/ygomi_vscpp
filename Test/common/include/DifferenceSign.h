#pragma once

#include "sign.h"
#include "Difference.h"

namespace ns_database {

class DifferenceSign :
    public Sign
{
public:
    DifferenceSign(void);
    DifferenceSign(Sign sign, int32 differenceId, ActionType_e actionType);
    ~DifferenceSign(void);

public:
    int32 getId() const;
    int32 getDifferenceId() const;
    ActionType_e getActionType() const;

    void setId(int32);
    void setDifferenceId(int32);
    void setActionType(ActionType_e);

private:
    int32 mId;
    int32 mDifferenceId;
    ActionType_e mActionType;
};

}

