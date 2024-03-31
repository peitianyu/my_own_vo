#ifndef __TYPES_POSE2D_H__
#define __TYPES_POSE2D_H__

#include<cmath> 
#include<iostream>
#include"point2d.h"
#include"radian.h"

struct Pose2D
{
    Point2D m_pos;
    Radian m_theta;

    Pose2D(Point2D p = Point2D(), double t = 0.f);

    Pose2D(const double& x, const double& y, const double& t);

    friend std::ostream& operator<<(std::ostream& os, const Pose2D& pose)
    {
        os << " " << pose.m_pos.x << " " << pose.m_pos.y << " " << pose.m_theta.value();
        return os;
    }

    double x() const { return m_pos.x; }
    double y() const { return m_pos.y; }
    double theta() const { return m_theta.value(); }

    double norm(Pose2D new_pose) const;

    Pose2D TransformFrom(const Pose2D &new_pose);

    Pose2D TransformAdd(const Pose2D &d_pose);

    Point2D TransformFrom(const Point2D &point);

    Point2D TransformAdd(const Point2D &d_point);
};

struct TimedPose2D
{
    double time_stamp;
    Pose2D pose;

    TimedPose2D(double time_stamp = 0, Pose2D pose = Pose2D());

    TimedPose2D(double time_stamp, double x, double y, double theta);

    TimedPose2D TransformFrom(const TimedPose2D &new_pose);

    TimedPose2D TransformAdd(const TimedPose2D &d_pose);

    friend std::ostream& operator<<(std::ostream& os, const TimedPose2D& timed_pose)
    {
        os << timed_pose.time_stamp << timed_pose.pose;
        return os;
    }
};

#endif // __TYPES_POSE2D_H__