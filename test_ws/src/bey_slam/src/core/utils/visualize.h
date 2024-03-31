#ifndef __VISUALIZE_H__
#define __VISUALIZE_H__

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>

#include "grid_map/grid_map.h"
#include "types/pose2d.h"


class Visualize
{
public:
    Visualize();

    void PublishTrajectory(const TimedPose2D& slam_pose);

    void PublishGridMap(const GridMap& map);

    void PublishScan(TimedPose2D slam_pose, const std::vector<Point2D>& scan)
    {
        m_scan_msg.header.stamp = ros::Time(slam_pose.time_stamp);
        for(const Point2D& p: scan){
            Point2D point = slam_pose.pose.TransformAdd(p);
            geometry_msgs::Point32 msg_point;
            msg_point.x = point.x;
            msg_point.y = point.y;
            m_scan_msg.points.push_back(msg_point);
        }
            
        m_scan_pub.publish(m_scan_msg);
    }
private:
    int Index2Int(const Index2D& index);
private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_slam_traj_pub;
    ros::Publisher m_map_pub;
    ros::Publisher m_scan_pub;

    sensor_msgs::PointCloud m_scan_msg;
    nav_msgs::Path m_path_msg;
    nav_msgs::OccupancyGrid m_map_msg;
};


#endif // __VISUALIZE_H__