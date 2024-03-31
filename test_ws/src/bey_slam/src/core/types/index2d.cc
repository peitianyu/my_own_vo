#include "index2d.h"
#include "math.h"
#include <cmath>
#include <assert.h>

Index2D::Index2D(int _x, int _y, double _resolution) : x(_x), y(_y), resolution(_resolution) {}

Index2D::Index2D(double _resolution):Index2D(0.f, 0.f, _resolution) {}

Index2D::Index2D(const Point2D &p, double _resolution) : resolution(_resolution)
{
    x = std::round(p.x / resolution);
    y = std::round(p.y / resolution);
}

Point2D Index2D::ToPoint2D() const
{
    return Point2D(x * resolution, y * resolution);
}

Index2D Index2D::operator+(const Index2D &other) const
{
    assert(equal(resolution, other.resolution));
    return Index2D(x + other.x, y + other.y, resolution);
}

Index2D Index2D::operator-(const Index2D &other) const
{
    assert(equal(resolution, other.resolution));
    return Index2D(x - other.x, y - other.y, resolution);
}

bool Index2D::operator==(const Index2D &other) const
{
    return x == other.x && y == other.y && (equal(resolution, other.resolution));
}
