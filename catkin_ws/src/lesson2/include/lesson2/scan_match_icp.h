#ifndef LESSON2_SCAN_MATCH_ICP
#define LESSON2_SCAN_MATCH_ICP
#include <iostream>
#include <deque>
// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "laser_geometry/laser_geometry.h"

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
// pcl_ros
#include <pcl_ros/point_cloud.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
class ScanMatchICP
{
    // 使用PCL中点的数据结构 pcl::PointXYZ
    typedef pcl::PointXYZ PointT;
    // 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
    typedef pcl::PointCloud<PointT> PointCloudT;
private:
    tf::TransformListener listener;
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Subscriber odom_subscriber_; // 声明一个Subscriber
    ros::Subscriber imu_subscriber_; // 声明一个Subscriber
    ros::Publisher transformed_pointcloud_pub_;
    ros::Time last_icp_time_;               // 上次的时间戳
    ros::Time current_time_;
    geometry_msgs::Twist latest_velocity_;
    bool is_first_scan_;    // 判断是否是第一个雷达数据
    bool is_first_odom_;    // 判断是否是第一个odom数据
    bool is_first_imu_;    // 判断是否是第一个odom数据
    std::deque<Eigen::Matrix4f> transformations_;
    // std::deque<Eigen::Matrix4f> predit_transform_;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud_;    // 当前帧雷达数据
    // pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_;       // 前一帧雷达数据



    //根据预测位姿转换后的点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud_vg;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::Twist> syncpolicy;//时间戳对齐规则
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_;//时间同步器
    
    tf2_ros::Buffer tfBuffer_; //坐标转换监听者缓存区
    tf2_ros::TransformListener tf_listener_;//坐标转换监听者
    tf2_ros::TransformBroadcaster tf_broadcaster_;//坐标转换发布者

    tf2::Transform base_to_laser_;    
    tf2::Transform laser_to_base_; 

    tf2::Transform base_in_odom_;           // base_link在odom坐标系下的坐标
    tf2::Transform base_in_odom_keyframe_;  // base_link在odom坐标系下的keyframe的坐标
    bool initialized_;                      // 是否初始化的标志

    std::vector<double> a_cos_;             // 保存下来雷达各个角度的cos值
    std::vector<double> a_sin_;             // 保存下来雷达各个角度的sin值    
    // icp算法
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;

    // void ScanMatchWithICP(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    // void GetPrediction(double &prediction_change_x, double &prediction_change_y, double &prediction_change_angle, double dt);
    // void CreateTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);
    void ScanMatchWithICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered_, const std_msgs::Header &header,Eigen::Affine3f &predit_pose);
    void ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void ApplyStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);
    void Predicted_pose(const nav_msgs::OdometryConstPtr& odom_msg,const sensor_msgs::ImuConstPtr& imu_msg,Eigen::Affine3f& predit_pose,double dt);
    void VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_v, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud_v);
public:
    ScanMatchICP();
    ~ScanMatchICP();
    // ,tf::TransformListener& listener
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& odom_msg,const sensor_msgs::ImuConstPtr& imu_msg);
};


#endif // LESSON2_SCAN_MATCH_ICP