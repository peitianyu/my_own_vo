//imu融合里程计预测位姿scan_to_sacn构建子图版本，scan_to_sacn
#include "lesson2/scan_match_icp.h"
#include <chrono>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
using namespace message_filters;
using namespace std;
nav_msgs::Odometry prev_odom;
nav_msgs::Odometry current_odom;
sensor_msgs::Imu prev_imu;
sensor_msgs::Imu current_imu;
pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud_vg(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZ>());
Eigen::Affine3f predit_pose = Eigen::Affine3f::Identity();
ScanMatchICP::ScanMatchICP() : private_node_("~"), tf_listener_(tfBuffer_),listener()
{   
    ROS_INFO_STREAM("\033[1;32m----> Scan Match with PLICP started.\033[0m");
    
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_subscriber_;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_subscriber_;
    message_filters::Subscriber<sensor_msgs::Imu>* imu_subscriber_;
    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry,sensor_msgs::Imu> MySyncPolicy;
    laser_scan_subscriber_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_handle_,"/scan",1);
    odom_subscriber_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_handle_,"/odom",1);
    imu_subscriber_ = new message_filters::Subscriber<sensor_msgs::Imu>(node_handle_,"/imu",1);
 
    message_filters::Synchronizer<MySyncPolicy>* sync;
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*laser_scan_subscriber_,*odom_subscriber_,*imu_subscriber_);
    sync->registerCallback(boost::bind(&ScanMatchICP::ScanCallback,this,_1,_2,_3));
    
    transformed_pointcloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud", 1);
   
    // current_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // last_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // cloud_filtered_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    submap_cloud = boost::shared_ptr<PointCloudT>(new PointCloudT());
    submap_cloud_vg = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // submap_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    is_first_scan_ = true;
    is_first_odom_ = true;
    is_first_imu_ = true;
   
}

ScanMatchICP::~ScanMatchICP()
{

    ROS_INFO("Destroying ScanMatchICP");
}

void ScanMatchICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::ImuConstPtr& imu_msg)
{
    cout<<"--------------------------"<<"第"<<scan_msg->header.seq-648<<endl;
    cout<<"-----Callback-----"<<endl;
    cout<<"雷达时间:"<<scan_msg->header.stamp<<endl;
    cout<<"里程时间:"<<odom_msg->header.stamp<<endl;
    cout<<"imu时间:"<<imu_msg->header.stamp<<endl;

    // 对第一帧数据进行特殊处理
    if (is_first_scan_ == true)
    {
        // 进行第一帧数据的处理,只转换数据类型 并 保存到current_pointcloud_
        ConvertScan2PointCloud(scan_msg);
        
        ApplyStatisticalOutlierRemoval(current_pointcloud_,cloud_filtered_);
        last_icp_time_= scan_msg->header.stamp;
        *last_pointcloud_ = *cloud_filtered_; 
        // *submap_cloud = *cloud_filtered_;
        std::cout<<"第一帧点云数量"<<cloud_filtered_->size()<<std::endl;
        is_first_scan_ = false;
        // return;
    }
    else    
    
        // 在将新一帧数据转换到当前帧之前,
        // 先将current_pointcloud_赋值到last_pointcloud_进行保存
     { 
        // 改了这里
    // *last_pointcloud_ = *transformed_source_cloud;
    *last_pointcloud_=*cloud_filtered_;

    ConvertScan2PointCloud(scan_msg);
    ros::Time current_time_= scan_msg->header.stamp;
    
    cout<<"当前帧时间戳："<<current_time_<<endl;
    cout<<"上一帧时间戳："<<last_icp_time_<<endl;
    double dt = (current_time_-last_icp_time_).toSec();
    cout<<"dt:"<<dt<<endl;
    ApplyStatisticalOutlierRemoval(current_pointcloud_,cloud_filtered_);

    // Eigen::Affine3f predit_pose = Eigen::Affine3f::Identity();
    Predicted_pose(odom_msg,imu_msg,predit_pose,dt);
    // 调用ICP进行计算
    ScanMatchWithICP(cloud_filtered_,scan_msg->header,predit_pose);
    
    last_icp_time_ = current_time_;
    
    }
    //发布点云数据
    // VoxelGrid(submap_cloud, submap_cloud_vg);
    sensor_msgs::PointCloud2 final_cloud_msg;
    pcl::toROSMsg(*submap_cloud,final_cloud_msg);
    cout<<"最终发布的当上一帧点云"<<submap_cloud->size()<<endl;
    final_cloud_msg.header = scan_msg->header;
    std::cout<<"雷达坐标"<<scan_msg->header.frame_id<<std::endl;

    transformed_pointcloud_pub_.publish(final_cloud_msg);
}


void ScanMatchICP::ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{

    PointCloudT::Ptr cloud_msg = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // 对容器进行初始化
    cloud_msg->points.resize(scan_msg->ranges.size());

    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 首先声明一个 cloud_msg第i个点的 引用
        pcl::PointXYZ &point_tmp = cloud_msg->points[i];
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
            continue;

        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if (range > scan_msg->range_min && range < scan_msg->range_max)
        {
            // 获取第i个点对应的角度
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // 获取第i个点在笛卡尔坐标系下的坐标
            point_tmp.x = range * cos(angle);
            point_tmp.y = range * sin(angle);
            point_tmp.z = 0.0;
        }
    }

    // 高度为1的情况下, width即为所有点的个数
    cloud_msg->width = scan_msg->ranges.size();
    cloud_msg->height = 1;
    cloud_msg->is_dense = true; // not contains nans

    // 将scan_msg的消息头 赋值到 pcl::PointCloud<pcl::PointXYZ>的消息头
    pcl_conversions::toPCL(scan_msg->header, cloud_msg->header);

    // 将转换完的数据赋值到current_pointcloud_中保存下来
    *current_pointcloud_ = *cloud_msg;
}
void ScanMatchICP::ApplyStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(20); // 设置邻域点数
    sor.setStddevMulThresh(0.6); // 设置标准差倍数
    sor.filter(*output_cloud);
}
void ScanMatchICP::VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_v, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud_v)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input_cloud_v);
    vg.setLeafSize(0.1f, 0.1f, 0.1f); // 体素滤波器，单位m
    vg.filter(*output_cloud_v);

}
void ScanMatchICP::Predicted_pose(const nav_msgs::OdometryConstPtr& odom_msg,const sensor_msgs::ImuConstPtr& imu_msg, Eigen::Affine3f& predit_pose,double dt)
{

    cout<<"-----Predicted_pose-----"<<endl;
    current_odom=*odom_msg;

    cout<<"cur:"<<current_odom.pose.pose.position.x<<endl;
    cout<<"pre:"<<prev_odom.pose.pose.position.x<<endl;
    tf2::Quaternion prev_quaternion,current_quaternion;
    tf2::fromMsg(prev_odom.pose.pose.orientation,prev_quaternion);
    tf2::fromMsg(current_odom.pose.pose.orientation,current_quaternion);
    // tf2::Quaternion rotation_diff = current_quaternion * prev_quaternion.inverse();
    double dx = current_odom.pose.pose.position.x-prev_odom.pose.pose.position.x;
    double dy = current_odom.pose.pose.position.y-prev_odom.pose.pose.position.y;
    double dz = current_odom.pose.pose.position.z-prev_odom.pose.pose.position.z;
        if(is_first_odom_ == true)
    {
        dx=0.0;
        dy=0.0;
        is_first_odom_ = false;
    }
    cout<<"dx:"<<dx<<endl;
    cout<<"dy:"<<dy<<endl;

    geometry_msgs::Pose pose_diff;
    pose_diff.position.x = dx;
    pose_diff.position.y = dy;
    pose_diff.position.z = dz;
    
    // pose_diff.orientation = tf2::toMsg(rotation_diff);
   
    predit_pose.translation()<<pose_diff.position.x,
                               pose_diff.position.y,
                               pose_diff.position.z;
    // Eigen::Quaternionf q(pose_diff.orientation.w,pose_diff.orientation.x,pose_diff.orientation.y,pose_diff.orientation.z);
    // predit_pose.rotate(q);
    prev_odom = current_odom;

    //imu数据处理
   
    current_imu=*imu_msg;

    
 
    double yaw   = prev_imu.angular_velocity.z*dt;
    if(is_first_imu_ == true)
    {
        yaw=0.0;
        is_first_imu_ = false;
    }
        // 计算四元数表示的旋转
   
    predit_pose(0,0)=cos(yaw);
    predit_pose(0,1)=-sin(yaw);
    predit_pose(1,0)=sin(yaw);
    predit_pose(1,1)=cos(yaw);
  
    prev_imu = current_imu;
    // cout<<predit_pose.matrix()<<endl;
}
/*
 * 调用ICP进行帧间位姿的计算
 */
void ScanMatchICP::ScanMatchWithICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered_, const std_msgs::Header &header,Eigen::Affine3f &predit_pose)
{
    // std::cout<<predit_pose.matrix()<<std::endl;

    // //转换后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f predit_pose1=Eigen::Matrix4f::Identity();
    predit_pose1= predit_pose.matrix();
    std::cout<<"当前传入的点云："<<cloud_filtered_->size()<<std::endl;
    std::cout<<"上一帧传入的点云："<<last_pointcloud_->size()<<std::endl;
    cout<<predit_pose1.matrix()<<endl;
    pcl::transformPointCloud(*cloud_filtered_, *transformed_source_cloud, predit_pose1);
    // ICP 输入数据,输出数据的设置,还可以进行参数配置,这里使用默认参宿

    icp_.setInputSource(transformed_source_cloud);//把transformed_source_cloud传进来
    icp_.setInputTarget(last_pointcloud_);
    icp_.setTransformationEpsilon(1e-10);//前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件
    icp_.setMaxCorrespondenceDistance(0.15);
    icp_.setEuclideanFitnessEpsilon(0.001);
    icp_.setMaximumIterations(100);
    icp_.setUseReciprocalCorrespondences(true);    
    // 开始迭代计算

    icp_.align(*unused_result);

    // std::cout << "has converged:" << icp_.hasConverged() << " score: " << icp_.getFitnessScore() << std::endl;

    // 如果迭代没有收敛,不进行输出
    if (icp_.hasConverged() == false)
    {
        std::cout << "not Converged" << std::endl;
        return;
    }
    else
    {
        // // 收敛了之后, 获取坐标变换
        Eigen::Matrix4f transfrom;
        transfrom = icp_.getFinalTransformation();
        // pcl::transformPointCloud(*transformed_source_cloud,unused_result,transfrom);
        //
        Eigen::Matrix4f Transfrom;
        Transfrom=transfrom*predit_pose1;
        //
        Eigen::Matrix4f combined_transform = Eigen::Matrix4f::Identity();
        transformations_.push_back(Transfrom);

        for(int i =0;i<transformations_.size();++i)
        {
            combined_transform *= transformations_[i];
            // std::cout<<"----------"<<std::endl;
        }

        pcl::transformPointCloud(*cloud_filtered_,*unused_result,combined_transform);
        *submap_cloud += *unused_result;
        
        //体素滤波
        // cout<<"子图滤波前的点数"<<submap_cloud->size()<<endl;
        // VoxelGrid(submap_cloud,submap_cloud_vg);
        // cout<<"子图滤波后的点数"<<submap_cloud_vg->size()<<endl;
     
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson2_scan_match_icp_node"); // 节点的名字
    ScanMatchICP scan_match_icp;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}