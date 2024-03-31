/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//3月7号早上版本
#include "lesson2/scan_match_icp.h"
#include <chrono>
#include <pcl/registration/icp.h>
/*
 * 构造函数
 */
ScanMatchICP::ScanMatchICP()
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Scan Match with ICP started.\033[0m");

    laser_scan_subscriber_ = node_handle_.subscribe(
        "front_scan", 1, &ScanMatchICP::ScanCallback, this);

    // 第一帧数据的标志
    is_first_scan_ = true;
    transformed_pointcloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud", 1);
    // 指针的初始化
    current_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    last_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_filtered_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    submap_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
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
void ScanMatchICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // step1 进行数据类型转换
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); 
    // ApplyStatisticalOutlierRemoval(current_pointcloud_,cloud_filtered_);
    // 对第一帧数据进行特殊处理
    if (is_first_scan_ == true)
    {
        // 进行第一帧数据的处理,只转换数据类型 并 保存到current_pointcloud_
        ConvertScan2PointCloud(scan_msg);
        // ApplyStatisticalOutlierRemoval(current_pointcloud_,cloud_filtered_);
        is_first_scan_ = false;
        return;
    }
    else    
    {
        // 在将新一帧数据转换到当前帧之前,
        // 先将current_pointcloud_赋值到last_pointcloud_进行保存
        // *last_pointcloud_ = *current_pointcloud_;   
        //改了这里
         *last_pointcloud_ = *current_pointcloud_;  
    }
    // 进行数据类型转换
    ConvertScan2PointCloud(scan_msg);
    // ApplyStatisticalOutlierRemoval(current_pointcloud_,cloud_filtered_);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    std::cout << "\n转换数据格式用时: " << time_used.count() << " 秒。" << std::endl;

    // step2 使用ICP计算 雷达前后两帧间的坐标变换
    start_time = std::chrono::steady_clock::now();

    // 调用ICP进行计算
    ScanMatchWithICP(current_pointcloud_,scan_msg->header);

    end_time = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    std::cout << "ICP计算用时: " << time_used.count() << " 秒。" << std::endl;



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
    // PointCloudT::Ptr的数据类型为boost::shared_ptr
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
        sor.setMeanK(50); // 设置邻域点数
        sor.setStddevMulThresh(1.0); // 设置标准差倍数
        sor.filter(*output_cloud);
    }
/*
 * 调用ICP进行帧间位姿的计算
 */
void ScanMatchICP::ScanMatchWithICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud, const std_msgs::Header &header)
{
    // 设置初始位姿
    // Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
    // initial_transform(0, 3) = 0.1;  // 通过odom获得
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::transformPointCloud(*current_pointcloud_, *transformed_source_cloud, initial_transform);

    // ICP 输入数据,输出数据的设置,还可以进行参数配置,这里使用默认参宿
    icp_.setInputSource(filtered_cloud);//把transformed_source_cloud传进来
    icp_.setInputTarget(last_pointcloud_);
    icp_.setTransformationEpsilon(1e-10);
    icp_.setMaxCorrespondenceDistance(1);
    icp_.setEuclideanFitnessEpsilon(0.001);
    icp_.setMaximumIterations(35);
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

        // 将Eigen::Affine3f转换成x, y, theta, 并打印出来
        // float x, y, z, roll, pitch, yaw;
        // pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);
        // std::cout << "transfrom: (" << x << ", " << y << ", " << yaw * 180 / M_PI << ")" << std::endl;
        pcl::transformPointCloud(*last_pointcloud_,unused_result,transfrom);

        /////
        Eigen::Matrix4f combined_transform = Eigen::Matrix4f::Identity();
        transformations_.push_back(transfrom);
    
        for(int i =0;i<transformations_.size();++i)
        {
            combined_transform *= transformations_[i];
            // std::cout<<"----------"<<std::endl;
        }
        pcl::transformPointCloud(*last_pointcloud_,unused_result,combined_transform);

        *submap_cloud += unused_result;
    }


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson2_scan_match_icp_node"); // 节点的名字
    ScanMatchICP scan_match_icp;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}