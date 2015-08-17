#pragma once

#include <vector>
#include "Point3D.h"

namespace ns_database {

class Line
{
public:
    Line(void);
    Line(const std::vector<Point3D>& points);
    Line(const Point3D& startPoint, const Point3D& endPoint);
    ~Line(void);

public:
    void  setId(int32 id);
    int32 getId() const;

    void setPoints(const std::vector<Point3D>&);
    std::vector<Point3D> getPoints() const;
private:
    int32 mId;
    std::vector<Point3D> mPoints;
};

}
