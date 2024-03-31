//imu融合里程计预测位姿scan_to_sacn构建子图版本，scan_to_sacn
#include "lesson2/scan_match_icp.h"
#include <chrono>
#include <pcl/registration/icp.h>
using namespace message_filters;
using namespace std;
nav_msgs::Odometry prev_odom;
nav_msgs::Odometry current_odom;
sensor_msgs::Imu prev_imu;
sensor_msgs::Imu current_imu;
/*
 * 构造函数
 */

pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
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


    // 
    transformed_pointcloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud", 1);
   
    current_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    last_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_filtered_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    submap_cloud = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // submap_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    is_first_scan_ = true;
    is_first_odom_ = true;
    is_first_imu_ = true;
   
}
/*
 * 析构函数
 */
ScanMatchICP::~ScanMatchICP()
{

    ROS_INFO("Destroying ScanMatchICP");
}

/*
 * 回调函数 进行数据处理
 */
// 

void ScanMatchICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::ImuConstPtr& imu_msg)
{
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
        // last_icp_time_= cloud_filtered_->header.stamp;
        *last_pointcloud_ = *cloud_filtered_; 
        std::cout<<"第一帧点云数量"<<last_pointcloud_->size()<<std::endl;
        is_first_scan_ = false;
        // return;
    }
    else    
    {
        // 在将新一帧数据转换到当前帧之前,
        // 先将current_pointcloud_赋值到last_pointcloud_进行保存
        // *last_pointcloud_ = *current_pointcloud_;   
        // 改了这里
         *last_pointcloud_ = *transformed_source_cloud;
        // *last_pointcloud_ = *submap_cloud;
          
    }
    // 进行数据类型转换
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

    //发布点云数据
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*submap_cloud,transformed_cloud_msg);
    transformed_cloud_msg.header = scan_msg->header;
    std::cout<<"雷达坐标"<<scan_msg->header.frame_id<<std::endl;

    transformed_pointcloud_pub_.publish(transformed_cloud_msg);
}

/*
 * 将LaserScan消息类型转换为PCL的pcl::PointCloud类型
 */
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

void ScanMatchICP::Predicted_pose(const nav_msgs::OdometryConstPtr& odom_msg,const sensor_msgs::ImuConstPtr& imu_msg, Eigen::Affine3f& predit_pose,double dt)
{

    cout<<"-----Predicted_pose-----"<<endl;
    current_odom=*odom_msg;

    
   
   
    cout<<"cur:"<<current_odom.pose.pose.position.x<<endl;
    cout<<"pre:"<<prev_odom.pose.pose.position.x<<endl;
    tf2::Quaternion prev_quaternion,current_quaternion;
    tf2::fromMsg(prev_odom.pose.pose.orientation,prev_quaternion);
    tf2::fromMsg(current_odom.pose.pose.orientation,current_quaternion);
    tf2::Quaternion rotation_diff = current_quaternion * prev_quaternion.inverse();
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
    cout<<"dz:"<<dz<<endl;
    geometry_msgs::Pose pose_diff;
    pose_diff.position.x = dx;
    pose_diff.position.y = dy;
    pose_diff.position.z = dz;
    pose_diff.orientation = tf2::toMsg(rotation_diff);

    predit_pose.translation()<<pose_diff.position.x,
                               pose_diff.position.y,
                               pose_diff.position.z;
    // Eigen::Quaternionf q(pose_diff.orientation.w,
    //                      pose_diff.orientation.x,
    //                      pose_diff.orientation.y,
    //                      pose_diff.orientation.z);
    // predit_pose.rotate(q);

    
    // cout<<predit_pose.matrix()<<endl;

  
    prev_odom = current_odom;

  //imu数据处理
   
 
    current_imu=*imu_msg;
    tf2::Quaternion curr_imu_orientation;
  
    tf2::fromMsg(current_imu.orientation,curr_imu_orientation);
    std::cout << "curr_imu_orientation: " << curr_imu_orientation.x() << ", " << curr_imu_orientation.y() << ", " << curr_imu_orientation.z() << ", " << curr_imu_orientation.w() << std::endl;


        // 获取 IMU 的角速度
    tf::Vector3 angular_velocity(imu_msg->angular_velocity.x,
                                  imu_msg->angular_velocity.y,
                                  imu_msg->angular_velocity.z);

    // 根据角速度计算旋转增量
    // tf2::Quaternion delta_rotation;
    tf2::Quaternion prev_imu_orientation;
    tf2::fromMsg(prev_imu.orientation,prev_imu_orientation);
    // delta_rotation.setRotation(tf2::Vector3(angular_velocity.x(),
    //                                        angular_velocity.y(),
    //                                        angular_velocity.z()),
    //                            dt);
        //计算两帧之间的旋转
    tf2::Quaternion delta_rotation  = curr_imu_orientation*prev_imu_orientation.inverse();
    std::cout << "last_imu_orientation: " << prev_imu_orientation.x() << ", " << prev_imu_orientation.y() << ", " << prev_imu_orientation.z() << ", " << prev_imu_orientation.w() << std::endl;
    std::cout << "delta_rotation: " << delta_rotation.x() << ", " << delta_rotation.y() << ", " << delta_rotation.z() << ", " << delta_rotation.w() << std::endl;
    // 更新上一次 IMU 数据的时间戳和姿态
    Eigen::Quaternionf q(delta_rotation.getW(),
                         0.0,
                         0.0,
                         delta_rotation.getZ());

    predit_pose.rotate(q);

    prev_imu = current_imu;
    cout<<predit_pose.matrix()<<endl;

 

}
/*
 * 调用ICP进行帧间位姿的计算
 */
void ScanMatchICP::ScanMatchWithICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud, const std_msgs::Header &header,Eigen::Affine3f &predit_pose)
{
// std::cout<<predit_pose.matrix()<<std::endl;

    // //转换后的点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*filtered_cloud, *transformed_source_cloud, predit_pose);
    // ICP 输入数据,输出数据的设置,还可以进行参数配置,这里使用默认参宿
    std::cout<<"滤波后传入的点云："<<filtered_cloud->size()<<std::endl;
    std::cout<<"滤波转换后传入的点云："<<transformed_source_cloud->size()<<std::endl;
    icp_.setInputSource(transformed_source_cloud);//把transformed_source_cloud传进来
    icp_.setInputTarget(last_pointcloud_);
    // icp_.setInputTarget(submap_cloud);
    icp_.setTransformationEpsilon(1e-10);
    icp_.setMaxCorrespondenceDistance(0.15);
    icp_.setEuclideanFitnessEpsilon(0.001);
    icp_.setMaximumIterations(100);
    icp_.setUseReciprocalCorrespondences(true);    
    // 开始迭代计算

    icp_.align(unused_result);

    // std::cout << "has converged:" << icp_.hasConverged() << " score: " << icp_.getFitnessScore() << std::endl;

    // 如果迭代没有收敛,不进行输出
    if (icp_.hasConverged() == false)
    {
        std::cout << "not Converged" << std::endl;
        return;
    }
    else
    {
        // 收敛了之后, 获取坐标变换
        Eigen::Matrix4f transfrom;
        transfrom = icp_.getFinalTransformation();
        pcl::transformPointCloud(*transformed_source_cloud,unused_result,transfrom);

        
        Eigen::Matrix4f combined_transform = Eigen::Matrix4f::Identity();
        transformations_.push_back(transfrom);

        for(int i =0;i<transformations_.size();++i)
        {
            combined_transform *= transformations_[i];
            // std::cout<<"----------"<<std::endl;
        }
        pcl::transformPointCloud(*transformed_source_cloud,unused_result,combined_transform);

        *submap_cloud += unused_result;
        // *submap_cloud+=*transformed_source_cloud;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson2_scan_match_icp_node"); // 节点的名字
    ScanMatchICP scan_match_icp;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}