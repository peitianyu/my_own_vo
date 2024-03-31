#include "slam_normal.h"

SlamNormal::SlamNormal() : m_need_inited(5), m_name("NORMAL_SLAM")
{
    m_grid_map = std::make_shared<GridMap>(0.05, 20, 20); // 20m * 20m, 分辨率为0.05m
    m_slam_pose = std::make_shared<TimedPose2D>();

    m_csm = std::make_unique<CSM>((CSM::Option){0.06, 0.02, 0.05, 0.01, 0.85});
    m_pose_predict = std::make_unique<PosePredict>();
    m_pose_predict->AddSlamPose(*m_slam_pose);
}

void SlamNormal::AddNewScan(const TimedLaserScan& timed_scan)
{
    Init(timed_scan);

    TimedPose2D prior_pose;
    if(!m_pose_predict->GetPredictPose(timed_scan.GetTime(), prior_pose)) return;

    std::vector<Point2D> filtered_scan = VoxelFilter(timed_scan.GetPoints(), 0.1);

    m_csm->Match(m_grid_map, filtered_scan, prior_pose.pose);
    if(!m_csm->GetResult().is_success) {
        *m_slam_pose = prior_pose;

        std::cout << "fail result: " << m_csm->GetResult() << std::endl;
        return;
    }

    std::cout << "success result: " << m_csm->GetResult() << std::endl;

    m_slam_pose->time_stamp = timed_scan.GetTime();
    m_slam_pose->pose = m_csm->GetResult().robot_pose;

    m_grid_map->UpdateByScan(m_slam_pose->pose, timed_scan.GetPoints());
    m_pose_predict->AddSlamPose(*m_slam_pose);
}

void SlamNormal::AddNewOdom(const TimedPose2D& new_odom)
{
    m_pose_predict->AddNewOdom(new_odom);
}

const std::shared_ptr<GridMap> SlamNormal::GetGridMap() const
{
    return m_grid_map;
}

const std::shared_ptr<TimedPose2D> SlamNormal::GetSlamPose() const
{
    return m_slam_pose;
}

void SlamNormal::Init(const TimedLaserScan& timed_scan)
{
    if(m_need_inited == 0) return;
    m_need_inited--;

    TimedPose2D prior_pose;
    if(!m_pose_predict->GetPredictPose(timed_scan.GetTime(), prior_pose)) return;
    *m_slam_pose = prior_pose;

    m_grid_map->UpdateByScan(m_slam_pose->pose, timed_scan.GetPoints());
    m_grid_map->UpdateByScan(m_slam_pose->pose, timed_scan.GetPoints());
    m_grid_map->UpdateByScan(m_slam_pose->pose, timed_scan.GetPoints());
}

std::vector<Point2D> SlamNormal::VoxelFilter(const std::vector<Point2D>& scan, const float& resolution)
{
    std::vector<Point2D> filtered_scan;

    std::unordered_map<int, std::vector<Point2D>> voxel_map;
    for (const auto& point : scan){
        constexpr int index_max = 1e6;
        int index = std::round(point.x / resolution) * index_max + std::round(point.y / resolution);

        voxel_map[index].push_back(point);
    }

    for (const auto& voxel : voxel_map){
        Point2D mean_point(0.f, 0.f);

        for (const auto& point : voxel.second)
            mean_point = mean_point + point;

        filtered_scan.push_back(mean_point * (1.f / voxel.second.size()));
    }

    return filtered_scan;
}
