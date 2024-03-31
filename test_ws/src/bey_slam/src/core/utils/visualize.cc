#include "visualize.h"



Visualize::Visualize()
{
    m_path_msg.header.frame_id = "base_link";
    m_scan_msg.header.frame_id = "base_link";
    m_map_msg.header.frame_id = "base_link";
    m_map_msg.info.resolution = 0.05; // 10 / 0.05 = 200

    m_slam_traj_pub = m_node_handle.advertise<nav_msgs::Path>("trajectory",1, true); 
    m_map_pub = m_node_handle.advertise<nav_msgs::OccupancyGrid>("map",1, true); 
    m_scan_pub = m_node_handle.advertise<sensor_msgs::PointCloud>("slam_scan",1, true);
}

void Visualize::PublishTrajectory(const TimedPose2D& slam_pose)
{
    m_path_msg.header.stamp = ros::Time(slam_pose.time_stamp);

    uint traj_size = m_path_msg.poses.size()+1;
    m_path_msg.poses.resize(traj_size);

    m_path_msg.poses[traj_size-1].header.stamp = ros::Time(slam_pose.time_stamp);
    m_path_msg.poses[traj_size-1].header.frame_id = "map";
    m_path_msg.poses[traj_size-1].pose.position.x = slam_pose.pose.x();
    m_path_msg.poses[traj_size-1].pose.position.y = slam_pose.pose.y();
    m_path_msg.poses[traj_size-1].pose.position.z = 0;
    m_path_msg.poses[traj_size-1].pose.orientation = tf::createQuaternionMsgFromYaw(slam_pose.pose.theta());
    
    m_slam_traj_pub.publish(m_path_msg);
}

void Visualize::PublishGridMap(const GridMap& map)
{
    m_map_msg.header.stamp = ros::Time::now();
    m_map_msg.info.width = map.GetHalfSize()[0]*2;
    m_map_msg.info.height = map.GetHalfSize()[1]*2;
    m_map_msg.info.origin.position.x = -10;
    m_map_msg.info.origin.position.y = -10;

    std::vector<int8_t> data;
    data.resize(m_map_msg.info.width * m_map_msg.info.height, 50);
    Eigen::Matrix2i map_limit = map.GetMapLimit();
    for(int i = map_limit(0, 0); i <= map_limit(0, 1); ++i){
        for(int j = map_limit(1, 0); j <= map_limit(1, 1); ++j){
            Index2D index(i, j);
            int index_int = Index2Int(index);
            data[index_int] = int(100 / (1.0 + std::exp(-map.GetData()(i, j))));
        }
    }
    m_map_msg.data = data;
    
    m_map_pub.publish(m_map_msg);
}

int Visualize::Index2Int(const Index2D& index)
{
    return index.x + index.y * m_map_msg.info.width;
}
