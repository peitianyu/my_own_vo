#ifndef __UTILS_READ_DATA_H__
#define __UTILS_READ_DATA_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "types/pose2d.h"
#include "types/imu.h"
#include "types/laser_scan.h"
#include "types/dataque.h"

class ReadData
{
public:
    ReadData();

    DataQue<TimedLaserScan, 5>& GetScanData();
    DataQue<Imu, 20>& GetImuData();
    DataQue<TimedPose2D, 20>& GetOdomData();
private:
    void ScanCallback(const sensor_msgs::PointCloud::ConstPtr& scan_msg);

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
private:
    Eigen::Vector3d GetEular(const Eigen::Quaterniond& q, const Eigen::Vector3d& prev_eular);

    void ThetaNormalize(Eigen::Vector3d& theta);
private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_scan_subscriber;
    ros::Subscriber m_imu_subscriber;
    ros::Subscriber m_odom_subscriber;

    DataQue<TimedLaserScan, 5> m_scan_data;
    DataQue<Imu, 20> m_imu_data;
    DataQue<TimedPose2D, 20> m_odom_data;
};



#endif // __UTILS_READ_DATA_H__