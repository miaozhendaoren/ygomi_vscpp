#pragma once
#include <vector>
#include "line.h"

typedef int32 PaintID;

namespace ns_database {

class SurfaceBoundary :
    public Line
{
public:
    SurfaceBoundary(void);
    SurfaceBoundary(bool isClosure, bool isImputed, const Line& lineInfo);
    SurfaceBoundary(bool isClosure, bool isImputed, const Line& lineInfo, const std::vector<PaintID>& lineOrigin);
    ~SurfaceBoundary(void);

public:
    void  setId(int32 id);
    int32 getId() const ;

    void  setClosure(bool );
    void  setImputed(bool );
    void  setOrigin(const std::vector<PaintID>& );

    bool  isClosure() const;
    bool  isImputed() const;
    std::vector<PaintID>  getOrigin() const;
private:
    int32 mId;
    bool mIsClosure;
    bool mIsImputed;
    std::vector<PaintID> mOrigin;
};

}
