#pragma once
#include "Surface.h"

namespace ns_database {

class Lane
{
public:
    Lane(void);
    Lane(const Surface& );
    ~Lane(void);

public:
    void setId(int32 id);
    int32 getId() const ;
    
    void setSurface(const Surface& surface);
    Surface getSurface() const ;
private:
    int32 mId;
    Surface mSurface;
};

}

