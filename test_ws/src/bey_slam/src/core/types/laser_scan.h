#ifndef __TYPES_LASER_SCAN_H__
#define __TYPES_LASER_SCAN_H__

#include "point2d.h"
#include <vector>
#include <iostream>

class LaserScan
{
public:
    LaserScan() = default;

    LaserScan(const std::vector<Point2D> &points);

    void PushBack(const Point2D &point);

    void PushBack(double x, double y);

    const std::vector<Point2D>& GetPoints() const;
protected:
    std::vector<Point2D> m_points;
};

class TimedLaserScan : public LaserScan
{
public:
    TimedLaserScan() = default;

    TimedLaserScan(const double& time, const std::vector<Point2D> &points);

    double GetTime() const;

    void SetTime(const double& time);
private:
    double m_time;
};

#endif // __TYPES_LASER_SCAN_H__