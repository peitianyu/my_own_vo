#include "types/laser_scan.h"
#include <cmath>

LaserScan::LaserScan(const std::vector<Point2D> &points) : m_points(points) {}

void LaserScan::PushBack(const Point2D &point) { m_points.push_back(point); }

void LaserScan::PushBack(double x, double y) { m_points.emplace_back(x, y); }

const std::vector<Point2D>& LaserScan::GetPoints() const { return m_points; }

TimedLaserScan::TimedLaserScan(const double& time, const std::vector<Point2D> &points) : LaserScan(points), m_time(time) {}

double TimedLaserScan::GetTime() const { return m_time; }

void TimedLaserScan::SetTime(const double& time) { m_time = time; }
