#ifndef __TYPES_POINT2D_H__
#define __TYPES_POINT2D_H__

#include<iostream>

struct Point2D
{
    double x;
    double y;

    Point2D(double x = 0.f, double y = 0.f);

    friend std::ostream& operator<<(std::ostream& os, const Point2D& point)
    {
        os << " " << point.x << " " << point.y;
        return os;
    }

    double Range() const;

    Point2D operator+(const Point2D &other) const;

    Point2D operator-(const Point2D &other) const;

    Point2D operator*(const double &other) const;
};

#endif // __TYPES_POINT2D_H__