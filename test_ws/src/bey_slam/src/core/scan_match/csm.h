#ifndef __CSM_H__
#define __CSM_H__

#include "grid_map/grid_map.h"
#include <Eigen/Core>
#include <memory>

class CSM
{
public:
    struct Option
    {
        double trans_offset;
        double rot_offset;
        double trans_resolution;
        double rot_resolution;
        double score_thr;
    };

    struct Result
    {
        Pose2D robot_pose;
        bool is_success;
        double score;

        Result(Pose2D _robot_pose = Pose2D(), bool _is_success = false, double _score = 0)
            : robot_pose(_robot_pose), is_success(_is_success), score(_score)
        {}

        friend std::ostream& operator<<(std::ostream& os, const Result& result){
            os << "relocate_result: " << result.robot_pose << " " << result.is_success << " " << result.score;
            return os;
        }
    };

    CSM(Option p);

    void SetOption(Option p);

    void Match(std::shared_ptr<GridMap> grid_map, const std::vector<Point2D>& point_cloud, const Pose2D& prior_pose);

    const Result& GetResult() const;
private:
    void GenerateParticles(const Pose2D& predict);

    void MatchFunc(std::shared_ptr<GridMap> grid_map, const std::vector<Point2D>& point_cloud);

    double Score(std::shared_ptr<GridMap> grid_map, const std::vector<Point2D>& point_cloud, Pose2D pose);
private:
    Option m_option;
    Result m_result;
    std::vector<Pose2D> m_poses; 
};

#endif // __CSM_H__