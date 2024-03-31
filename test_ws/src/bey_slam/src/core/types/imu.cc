#include "imu.h"

Imu::Imu() : m_time(0.0), 
            m_eular(Eigen::Vector3d::Zero()), 
            m_acceleration(Eigen::Vector3d::Zero()), 
            m_angular_velocity(Eigen::Vector3d::Zero()) {}

Imu::Imu(double time, const Eigen::Vector3d& eular, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity)
    : m_time(time), m_eular(eular), m_acceleration(acceleration), m_angular_velocity(angular_velocity) {}