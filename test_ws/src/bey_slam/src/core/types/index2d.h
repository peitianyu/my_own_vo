#ifndef __TYPES_INDEX2D_H__
#define __TYPES_INDEX2D_H__

#include"point2d.h"

struct Index2D
{
    int x;
    int y;
    double resolution;

    Index2D(double _resolution);

    Index2D(int _x = 0, int _y = 0, double _resolution = 0.05f);

    Index2D(const Point2D &p, double _resolution = 0.05f) ;

    Point2D ToPoint2D() const;

    Index2D operator+(const Index2D &other) const;

    Index2D operator-(const Index2D &other) const;

    bool operator==(const Index2D &other) const;
};

#endif // __TYPES_INDEX2D_H__