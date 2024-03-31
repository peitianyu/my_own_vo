#ifndef __TYPES_IMU_H__
#define __TYPES_IMU_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Imu
{
    double m_time;
    Eigen::Vector3d m_eular;
    Eigen::Vector3d m_acceleration;
    Eigen::Vector3d m_angular_velocity;

    Imu();

    Imu(double time, const Eigen::Vector3d& eular, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity);
};

#endif // __TYPES_IMU_H__