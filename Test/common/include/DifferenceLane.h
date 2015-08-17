#pragma once
#include "lane.h"
#include "Difference.h"

namespace ns_database {

class DifferenceLane :
    public Lane
{
public:
    DifferenceLane(void);
    DifferenceLane(Lane lane, int32 differenceId, ActionType_e actionType);
    ~DifferenceLane(void);
    
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
