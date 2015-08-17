#pragma once
#include "paint.h"
#include "Difference.h"

namespace ns_database {

class DifferencePaint :
    public Paint
{
public:
    DifferencePaint(void);
    DifferencePaint(Paint paint, int32 differenceId, ActionType_e actionType);
    ~DifferencePaint(void);
    
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
