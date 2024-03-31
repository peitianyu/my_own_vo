#include "pose_predict.h"

PosePredict::PosePredict(uint max_size) : m_max_size(max_size) {}

void PosePredict::AddNewOdom(const TimedPose2D& odom)
{
    if(!m_odom_que.empty() && m_odom_que.back().time_stamp >= odom.time_stamp)
        return ;
    
    m_odom_que.push_back(odom);
    if(m_odom_que.size() > m_max_size)
        m_odom_que.erase(m_odom_que.begin());
}

bool PosePredict::AddSlamPose(const TimedPose2D& slam_pose)
{
    m_slam_pose = slam_pose;
    return FindNearestOdom(m_mark_odom, slam_pose.time_stamp);
}

bool PosePredict::GetPredictPose(const double& scan_time, TimedPose2D& prior_pose)
{
    if(m_odom_que.empty()) return false;
        
    TimedPose2D nearest_odom;
    if(FindNearestOdom(nearest_odom, scan_time)){
        prior_pose = m_slam_pose.TransformAdd(m_mark_odom.TransformFrom(nearest_odom));
        return true;
    }else{
        return false;
    }
}

void PosePredict::Reset()
{
    m_odom_que.clear();
    m_slam_pose = TimedPose2D();
    m_mark_odom = TimedPose2D();
}

bool PosePredict::FindNearestOdom(TimedPose2D& nearest_odom, double scan_time)
{
    if(m_odom_que.empty()) return false;

    TimedPose2D prev_odom;
    TimedPose2D next_odom;
    for(uint i = 0; i < m_odom_que.size(); i++){
        if(m_odom_que[i].time_stamp < scan_time)
            prev_odom = m_odom_que[i];

        if(m_odom_que[i].time_stamp >= scan_time) 
            next_odom = m_odom_que[i];
    }
    
    if(next_odom.time_stamp <= prev_odom.time_stamp) return false;

    double d_t1 = scan_time - prev_odom.time_stamp;
    double d_t2 = next_odom.time_stamp - scan_time;
    Pose2D diff = prev_odom.pose.TransformFrom(next_odom.pose);

    double ratio = d_t1 / (d_t1 + d_t2);
    nearest_odom.time_stamp = scan_time;
    nearest_odom.pose = prev_odom.pose.TransformAdd(Pose2D(diff.x() * ratio, diff.y() * ratio, diff.theta() * ratio));
    return true;
}
