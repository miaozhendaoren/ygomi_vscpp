#pragma once

#include <vector>
#include "SurfaceBoundary.h"

namespace ns_database {

class Surface
{
public:
    Surface(void);
    Surface(const std::vector<SurfaceBoundary>& surfaceBoundaries);
    ~Surface(void);
    
public:
    void  setId(int32 id);
    int32 getId() const;

    void setSurfaceBoundaries(const std::vector<SurfaceBoundary>& surfaceBoundaries) ;
    std::vector<SurfaceBoundary> getSurfaceBoundaries() const;
private:
    int32 mId;
    std::vector<SurfaceBoundary> mSurfaceBoundaries;
};

}
