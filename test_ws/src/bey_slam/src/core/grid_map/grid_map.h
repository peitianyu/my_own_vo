#ifndef __GRID_MAP_H__
#define __GRID_MAP_H__


#include"types/index2d.h"
#include"types/pose2d.h"
#include"types/laser_scan.h"
#include<vector>
#include<array>
#include<cmath>
#include <Eigen/Dense>

class GridMap
{
public:
    GridMap(double resolution = 0.05f, double width = 100.f, double height = 100.f); // m

    GridMap(const std::string& log_odds_file);

    GridMap(const GridMap& grid_map, const uint downsample_size = 1);

    GridMap& operator=(const GridMap&) = default;

    double GetCellProb(const Index2D& cell_index) const;

    void UpdateByScan(Pose2D key_pose, const std::vector<Point2D>& point_cloud);

    void SetMapLimit(const Eigen::Matrix2i& map_limit);

    const Eigen::Matrix2i& GetMapLimit() const;

    void AddMapData(const Index2D& index, const double& log_odds);

    void Clear();

    double GetResolution() const;

    const Eigen::MatrixXd& GetData() const;

    const std::array<uint, 2>& GetHalfSize() const;

    void SaveProbMap(const std::string& file_name) const;

    void SaveLogOddsMap(const std::string& file_name) const;

    Index2D ConvertToCellIndex(const Index2D& index) const;
protected:
    void InverseModel(const Index2D &begin_point, const Index2D &end_point);

    void BresenhamCellOccupied(const Index2D &end_point);

    void BresenhamCellFree(const Index2D &begin_point, const Index2D &end_point);

    void BrasenHam(int x0, int y0, int x1, int y1);

    void UpdateMapLimit(const Index2D &index);

    void SetCellOccupied(const Index2D& cell_index);

    void SetCellFree(const Index2D& cell_index);

    double GetCellLogOdds(const Index2D& cell_index) const;

    void ResetMapLimit();

    Index2D ConvertToMapIndex(const Index2D& index) const;

    bool IsValid(const Index2D& cell_index) const;
    bool IsInMapRange(const Index2D& cell_index) const;
protected:
    double m_resolution; // m  
    std::array<uint, 2> m_half_size;  
    Eigen::MatrixXd m_data;
    /*
    | x_min   x_max |
    | y_min   y_max |
    */
    Eigen::Matrix2i m_map_limit;
};

#endif // __GRID_MAP_H__