#ifndef __SLAM_NORMAL_H__
#define __SLAM_NORMAL_H__

#include "grid_map/grid_map.h"
#include "scan_match/csm.h"
#include "pose_manager/pose_predict.h"
#include <memory>
#include <unordered_map>

class SlamNormal
{
public:
    SlamNormal();

    void AddNewScan(const TimedLaserScan& timed_scan); // 主函数

    void AddNewOdom(const TimedPose2D& new_odom);

    const std::shared_ptr<GridMap> GetGridMap() const;

    const std::shared_ptr<TimedPose2D> GetSlamPose() const;
protected:
    void Init(const TimedLaserScan& timed_scan);

    std::vector<Point2D> VoxelFilter(const std::vector<Point2D>& scan, const float& resolution = 0.2f);
protected:
    uint m_need_inited;
    std::string m_name;
    std::shared_ptr<GridMap> m_grid_map;
    std::shared_ptr<TimedPose2D> m_slam_pose;
    std::unique_ptr<CSM> m_csm;
    std::unique_ptr<PosePredict> m_pose_predict;
};


#endif // __SLAM_NORMAL_H__