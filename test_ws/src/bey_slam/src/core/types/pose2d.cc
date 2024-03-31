#include"pose2d.h"


Pose2D::Pose2D(Point2D p, double t) : m_pos(p), m_theta(t) {}

Pose2D::Pose2D(const double& x, const double& y, const double& t) : Pose2D(Point2D(x, y), t) {}

double Pose2D::norm(Pose2D new_pose) const
{
    double dx = new_pose.m_pos.x - m_pos.x;
    double dy = new_pose.m_pos.y - m_pos.y;
    double dt = Radian(new_pose.m_theta.value() - m_theta.value()).value();

    return std::sqrt(dx * dx + dy * dy + dt * dt);
}

Pose2D Pose2D::TransformFrom(const Pose2D &new_pose)
{
    Point2D d_pos = new_pose.m_pos - m_pos;
    return Pose2D(d_pos.x * std::cos(m_theta.value()) + d_pos.y * std::sin(m_theta.value()), 
                    -d_pos.x * std::sin(m_theta.value()) + d_pos.y * std::cos(m_theta.value()), 
                    new_pose.m_theta.value() - m_theta.value());
}

Pose2D Pose2D::TransformAdd(const Pose2D &d_pose)
{
    double d_x = d_pose.m_pos.x * std::cos(m_theta.value()) - d_pose.m_pos.y * std::sin(m_theta.value());
    double d_y = d_pose.m_pos.x * std::sin(m_theta.value()) + d_pose.m_pos.y * std::cos(m_theta.value());

    return Pose2D(m_pos.x + d_x, m_pos.y + d_y, m_theta.value() + d_pose.m_theta.value());
}

Point2D Pose2D::TransformFrom(const Point2D &point)
{
    Point2D d_pos = point - m_pos;
    return Point2D(d_pos.x * std::cos(m_theta.value()) + d_pos.y * std::sin(m_theta.value()), 
                    -d_pos.x * std::sin(m_theta.value()) + d_pos.y * std::cos(m_theta.value()));
}

Point2D Pose2D::TransformAdd(const Point2D &d_point)
{
    double d_x = d_point.x * std::cos(m_theta.value()) - d_point.y * std::sin(m_theta.value());
    double d_y = d_point.x * std::sin(m_theta.value()) + d_point.y * std::cos(m_theta.value());

    return Point2D(m_pos.x + d_x, m_pos.y + d_y);
}

TimedPose2D::TimedPose2D(double time_stamp, Pose2D pose)
    : time_stamp(time_stamp), pose(pose) {}

TimedPose2D::TimedPose2D(double time_stamp, double x, double y, double theta)
    : time_stamp(time_stamp), pose(x, y, theta) {}

TimedPose2D TimedPose2D::TransformFrom(const TimedPose2D &new_pose)
{
    return TimedPose2D(new_pose.time_stamp - time_stamp, pose.TransformFrom(new_pose.pose));
}

TimedPose2D TimedPose2D::TransformAdd(const TimedPose2D &d_pose)
{
    return TimedPose2D(time_stamp + d_pose.time_stamp, pose.TransformAdd(d_pose.pose));
}