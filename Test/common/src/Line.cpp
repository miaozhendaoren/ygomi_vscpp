#include "Line.h"


namespace ns_database {

Line::Line(void)
    :mId(0)
{
}

Line::Line(const std::vector<Point3D>& points)
    :mId(0), mPoints(points)
{
}

Line::Line(const Point3D& startPoint, const Point3D& endPoint){
    mPoints.push_back(startPoint);
    mPoints.push_back(endPoint);
}

Line::~Line(void)
{
}

void  Line::setId(int32 id) {
    mId = id;
}

int32 Line::getId() const {
    return 0;
}

void Line::setPoints(const std::vector<Point3D>& points) {
    mPoints.clear();
    mPoints = points;
}

std::vector<Point3D> Line::getPoints() const {
    return mPoints;
}

}
