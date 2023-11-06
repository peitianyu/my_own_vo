#ifndef __POSE3D_H__
#define __POSE3D_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

using Point3d = Eigen::Vector3d;
class Pose3d 
{
public:
    Pose3d() : q_(Eigen::Quaterniond::Identity()), t_(Eigen::Vector3d::Zero()) {}
    Pose3d(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) : q_(q), t_(t) {}
    Pose3d(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) : q_(R), t_(t) {}
    Pose3d(const Eigen::Matrix3d &R, const double& base_line) : q_(R), t_(Eigen::Vector3d::Zero()) { t_(0) = base_line; }
    Pose3d(const Pose3d &other) : q_(other.q_), t_(other.t_) {}

    Pose3d operator*(const double& s) const {
        return Pose3d(q_.slerp(s, Eigen::Quaterniond::Identity()), t_ * s);
    }

    Eigen::Quaterniond & q() { return q_; }
    Eigen::Vector3d & t() { return t_; }
    const Eigen::Quaterniond & q() const { return q_; }
    const Eigen::Vector3d & t() const { return t_; }

    Pose3d trans_from(const Pose3d &base) const  { return Pose3d((q_ * base.q_.conjugate()).normalized(), base.q_.toRotationMatrix().transpose() * (base.t_ - t_)); }
    Pose3d trans_add(const Pose3d &other) const { return Pose3d((q_ * other.q_).normalized(), q_.toRotationMatrix() * other.t_ + t_);}
    
    Point3d trans_add(const Point3d& p) const { return q_.toRotationMatrix() * p + t_; }
    Point3d trans_from(const Point3d& p) const { return q_.toRotationMatrix().transpose() * (p - t_); }

    friend std::ostream & operator << (std::ostream &out, const Pose3d &pose) {
        Eigen::Vector3d euler_angles = pose.q_.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
        out << "r: " << euler_angles.transpose() << " t: " << pose.t_.transpose();
        return out;
    }
private:
    Eigen::Quaterniond q_;
    Eigen::Vector3d t_;
};

#endif // __POSE3D_H__