#include "pose_manager.h"


PoseManager::PoseManager(uint max_size) : m_max_size(max_size) {}

void PoseManager::AddNewOdom(const TimedPose2D& odom)
{
    if(!m_odom_que.empty() && m_odom_que.back().time_stamp >= odom.time_stamp)
        return ;
    
    m_odom_que.push(odom);
    if(m_odom_que.size() > m_max_size)
        m_odom_que.pop();
}

void PoseManager::AddSlamPose(const TimedPose2D& slam_pose)
{
    m_slam_pose = slam_pose;
}

TimedPose2D PoseManager::GetCurrPose()
{
    if(TimedPose2D* delta_odom = GetDeltaOdom())
        return m_slam_pose.TransformAdd(*delta_odom);
    else
        return m_slam_pose;
}

void PoseManager::Reset()
{
    m_mark_odom = TimedPose2D();
}

TimedPose2D* PoseManager::GetDeltaOdom()
{
    if(m_odom_que.empty()) return nullptr;
        
    static TimedPose2D delta_odom;

    // no new slam pose input
    if(m_mark_odom.time_stamp == m_slam_pose.time_stamp){
        delta_odom = m_mark_odom.TransformFrom(m_odom_que.back());
        return &delta_odom;
    }

    static TimedPose2D prev_odom;
    TimedPose2D next_odom;
    while (m_odom_que.front().time_stamp < m_slam_pose.time_stamp) {
        prev_odom = m_odom_que.front();
        m_odom_que.pop();
        if(m_odom_que.empty()) return nullptr;
    }

    // FIXME: 近似线性差分, 可能不太准, 但方便呀
    next_odom = m_odom_que.front();
    double d_t1 = m_slam_pose.time_stamp - prev_odom.time_stamp;
    double d_t2 = next_odom.time_stamp - m_slam_pose.time_stamp;
    m_mark_odom = d_t1 < d_t2 ? prev_odom : next_odom;
    m_mark_odom.time_stamp = m_slam_pose.time_stamp;
    delta_odom = m_mark_odom.TransformFrom(m_odom_que.back());

    return &delta_odom;
}
