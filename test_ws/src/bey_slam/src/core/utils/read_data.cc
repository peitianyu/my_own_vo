#include "read_data.h"


ReadData::ReadData()
{
    m_scan_subscriber = m_node_handle.subscribe("undistortion_scan", 10, &ReadData::ScanCallback, this);
    m_imu_subscriber = m_node_handle.subscribe("imu", 30, &ReadData::ImuCallback, this);
    m_odom_subscriber = m_node_handle.subscribe("odom", 30, &ReadData::OdomCallback, this);
}

DataQue<TimedLaserScan, 5>& ReadData::GetScanData() { return m_scan_data; }
DataQue<Imu, 20>& ReadData::GetImuData() { return m_imu_data; }
DataQue<TimedPose2D, 20>& ReadData::GetOdomData() { return m_odom_data; }

void ReadData::ScanCallback(const sensor_msgs::PointCloud::ConstPtr& scan_msg)
{
    double timestamp = scan_msg->header.stamp.toSec();
    std::vector<Point2D> points;
    for(int i = 0; i < scan_msg->points.size(); i++)
        points.push_back(Point2D(scan_msg->points[i].x, scan_msg->points[i].y));
    m_scan_data.PushBack(TimedLaserScan(timestamp, points));
}

void ReadData::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    double timestamp = imu_msg->header.stamp.toSec();
    Eigen::Quaterniond q(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    static Eigen::Vector3d prev_eular = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d euler = GetEular(q, prev_eular);
    prev_eular = euler;
    Eigen::Vector3d angular_velocity(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    Eigen::Vector3d linear_acceleration(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);

    m_imu_data.PushBack(Imu(timestamp, euler, linear_acceleration, angular_velocity));
}

void ReadData::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    double timestamp = odom_msg->header.stamp.toSec();
    Eigen::Quaterniond q(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    static Eigen::Vector3d prev_eular = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d euler = GetEular(q, prev_eular);
    prev_eular = euler;
    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;

    m_odom_data.PushBack(TimedPose2D(timestamp, Pose2D(x, y, euler[2])));
}

Eigen::Vector3d ReadData::GetEular(const Eigen::Quaterniond& q, const Eigen::Vector3d& prev_eular) // roll pitch yaw
{
    Eigen::Vector3d curr_eular = q.toRotationMatrix().eulerAngles(0, 1, 2);

    // FIXME: 假设两帧imu的yaw角之间差距不可能超过弧度0.34
    Eigen::Vector3d d_eular = curr_eular - prev_eular;
    for(uint i = 0; i < 3; i++){   
        if(d_eular[i] > 2.8 && d_eular[i] < 6) {curr_eular[i] -= M_PI;}
        else if(d_eular[i] < -2.8 && d_eular[i] > -6) {curr_eular[i] += M_PI;}
    }
    // 归一化
    ThetaNormalize(curr_eular);

    return curr_eular;
}

void ReadData::ThetaNormalize(Eigen::Vector3d& theta)
{
    for(uint i = 0; i < 3; i++)
    {
        while (theta[i] > M_PI) theta[i] -= 2 * M_PI;
        while (theta[i] < -M_PI) theta[i] += 2 * M_PI;
    }
}
