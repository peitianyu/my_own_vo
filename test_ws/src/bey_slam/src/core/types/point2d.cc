#include "point2d.h"
#include <cmath>

Point2D::Point2D(double x, double y) : x(x), y(y) {}

double Point2D::Range() const
{
    return std::sqrt(x * x + y * y);
}

Point2D Point2D::operator+(const Point2D &other) const
{
    return Point2D(x + other.x, y + other.y);
}

Point2D Point2D::operator-(const Point2D &other) const
{
    return Point2D(x - other.x, y - other.y);
}

Point2D Point2D::operator*(const double &other) const
{
    return Point2D(x * other, y * other);
}
