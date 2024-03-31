#ifndef __POSE_MANAGER_H__
#define __POSE_MANAGER_H__


#include "types/pose2d.h"
#include <queue>

class PoseManager
{
public:
    PoseManager(uint max_size = 50);

    void AddNewOdom(const TimedPose2D& odom);

    void AddSlamPose(const TimedPose2D& slam_pose);

    TimedPose2D GetCurrPose();

    void Reset();
private:
    TimedPose2D* GetDeltaOdom();
private:
    uint m_max_size;
    TimedPose2D m_slam_pose;
    TimedPose2D m_mark_odom;
    std::queue<TimedPose2D> m_odom_que;
};

#endif // __POSE_MANAGER_H__