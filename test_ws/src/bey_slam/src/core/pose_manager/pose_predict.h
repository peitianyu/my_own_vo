#ifndef __POSE_PREDICT_H__
#define __POSE_PREDICT_H__

#include "types/pose2d.h"
#include <queue>

class PosePredict
{
public:
    PosePredict(uint max_size = 20);

    void AddNewOdom(const TimedPose2D& odom);

    bool AddSlamPose(const TimedPose2D& slam_pose);

    bool GetPredictPose(const double& scan_time, TimedPose2D& prior_pose);

    void Reset();
private:
    bool FindNearestOdom(TimedPose2D& nearest_odom, double scan_time);
private:
    uint m_max_size;
    TimedPose2D m_mark_odom;
    TimedPose2D m_slam_pose;
    std::vector<TimedPose2D> m_odom_que;
};

#endif // __POSE_PREDICT_H__