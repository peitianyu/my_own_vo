#include "csm.h"

CSM::CSM(Option p) : m_option(p) {}

void CSM::SetOption(Option p)
{
    m_option = p;
}

void CSM::Match(std::shared_ptr<GridMap> grid_map, const std::vector<Point2D>& point_cloud, const Pose2D& prior_pose)
{
    GenerateParticles(prior_pose);
    MatchFunc(grid_map, point_cloud);
}

const CSM::Result& CSM::GetResult() const { return m_result; }

void CSM::GenerateParticles(const Pose2D& predict)
{
    std::vector<double> nx;
    std::vector<double> ny;
    nx.push_back(predict.x());
    ny.push_back(predict.y());
    for(double len = m_option.trans_resolution; len < m_option.trans_offset; len += m_option.trans_resolution)
    {
        nx.push_back(predict.x() + len);
        nx.push_back(predict.x() - len);
        ny.push_back(predict.y() + len);
        ny.push_back(predict.y() - len);
    }

    std::vector<Radian> angles;
    angles.push_back(predict.theta());
    for(double dt = m_option.rot_resolution; dt < m_option.rot_offset; dt += m_option.rot_resolution)
    {
        angles.push_back(predict.theta() - dt);
        angles.push_back(predict.theta() + dt);
    }

    m_poses.clear();
    for(const double& x: nx){
    for(const double& y: ny){
    for(auto& angle: angles){
        m_poses.push_back(Pose2D(x, y, angle.value()));
    }
    }
    }
}

void CSM::MatchFunc(std::shared_ptr<GridMap> grid_map, const std::vector<Point2D>& point_cloud)
{
    double best_score = 0;
    Pose2D best_pose;
    for(auto& pose: m_poses){
        double score = Score(grid_map, point_cloud, pose);
        if(score > best_score){
            best_score = score;
            best_pose = pose;
        }
    }
    best_score /= point_cloud.size();
    m_result = Result(best_pose, (best_score > m_option.score_thr), best_score);
}

double CSM::Score(std::shared_ptr<GridMap> grid_map, const std::vector<Point2D>& point_cloud, Pose2D pose)
{
    double score = 0;
    for(auto& point: point_cloud){
        Point2D point_in_map = pose.TransformAdd(point);
        score += grid_map->GetCellProb(Index2D(point_in_map, grid_map->GetResolution()));
    }
    return score;
}
