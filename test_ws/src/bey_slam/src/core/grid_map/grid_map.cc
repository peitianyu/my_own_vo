#include"grid_map.h"
#include<fstream>
#include<sstream>
#include <iomanip>


GridMap::GridMap(double resolution, double width, double height): m_resolution(resolution)
{
    uint row = static_cast<uint>(height / resolution);
    uint col = static_cast<uint>(width / resolution);
    m_half_size = {row/2, col/2};
    m_data = Eigen::MatrixXd::Zero(row, col);
    ResetMapLimit();
}

GridMap::GridMap(const std::string &log_odds_file)
{
    std::ifstream ifs(log_odds_file);
    if (!ifs.is_open()){
        std::cout << "Open log odds file failed!" << std::endl;
        return;
    }

    std::string line;
    std::getline(ifs, line);
    std::stringstream ss(line);
    uint row, col;
    ss >> m_resolution >> row >> col >> m_map_limit(0, 0) >> m_map_limit(0, 1) >> m_map_limit(1, 0) >> m_map_limit(1, 1);
    m_half_size = {row/2, col/2};
    m_data = Eigen::MatrixXd::Zero(row, col);

    uint row_index = m_map_limit(1, 0);
    while(std::getline(ifs, line)){
        std::stringstream ss(line);
        uint col_index = m_map_limit(0, 0);
        double log_odds;
        while(ss >> log_odds){
            m_data(row_index, col_index) = log_odds;
            ++col_index;
        }
        ++row_index;
    }

    ifs.close();
}

GridMap::GridMap(const GridMap& grid_map, const uint downsample_size)
{
    if(downsample_size == 1){
        m_resolution = grid_map.GetResolution();
        m_half_size = grid_map.GetHalfSize();
        m_data = grid_map.GetData();
        m_map_limit = grid_map.GetMapLimit();
    }else{
        m_resolution = grid_map.GetResolution() * downsample_size;
        uint row = grid_map.GetData().rows() / downsample_size;
        uint col = grid_map.GetData().cols() / downsample_size;
        m_half_size = {row/2, col/2};
        m_data = Eigen::MatrixXd::Zero(row, col);
        ResetMapLimit();

        for(int i = grid_map.GetMapLimit()(0, 0); i <= grid_map.GetMapLimit()(0, 1); ++i){
            for(int j = grid_map.GetMapLimit()(1, 0); j <= grid_map.GetMapLimit()(1, 1); ++j){
                Index2D cell_index(i / downsample_size, j / downsample_size, m_resolution);

                if (m_data(cell_index.x, cell_index.y) == 0){
                    UpdateMapLimit(cell_index);
                    m_data(cell_index.x, cell_index.y) = grid_map.GetData()(i, j);
                }else{
                    m_data(cell_index.x, cell_index.y) = std::max(m_data(cell_index.x, cell_index.y), grid_map.GetData()(i, j));
                }
            }
        }
    }
}

bool GridMap::IsValid(const Index2D& cell_index) const
{
    if(cell_index.x < m_map_limit(0, 0) || cell_index.x > m_map_limit(0, 1))
        return false;
    
    if(cell_index.y < m_map_limit(1, 0) || cell_index.y > m_map_limit(1, 1))
        return false;
    
    return true;
}

const Eigen::MatrixXd& GridMap::GetData() const
{
    return m_data;
}

const std::array<uint, 2>& GridMap::GetHalfSize() const
{
    return m_half_size;
}

const Eigen::Matrix2i& GridMap::GetMapLimit() const
{
    return m_map_limit;
}

double GridMap::GetCellProb(const Index2D& cell_index) const
{
    Index2D map_index = ConvertToMapIndex(cell_index);
    if(!IsValid(map_index)) { return 0.5;}

    double log_odds = GetCellLogOdds(map_index);
    double prob = 1.0 / (1.0 + std::exp(-log_odds));
    return std::isnan(prob) ? 0.5 : prob;
}

void GridMap::UpdateByScan(Pose2D key_pose, const std::vector<Point2D>& point_cloud)
{
    Index2D begin_point = ConvertToMapIndex(Index2D(key_pose.m_pos, m_resolution));
    if(!IsInMapRange(begin_point)) return;

    for(const auto& point : point_cloud)
    {
        Index2D end_point = ConvertToMapIndex(Index2D(key_pose.TransformAdd(point), m_resolution));
        if(!IsInMapRange(end_point)) continue;

        UpdateMapLimit(end_point);

        if(!(begin_point == end_point))
            InverseModel(begin_point, end_point);
    }
}

void GridMap::SetMapLimit(const Eigen::Matrix2i& map_limit){
    m_map_limit = map_limit;
}

void GridMap::AddMapData(const Index2D& index, const double& log_odds){
    m_data(index.x, index.y) = log_odds;
}

void GridMap::Clear(){
    m_data.setZero();
}

double GridMap::GetResolution() const{
    return m_resolution;
}

void GridMap::InverseModel(const Index2D &begin_point, const Index2D &end_point){
    BresenhamCellOccupied(end_point);

    BresenhamCellFree(begin_point, end_point);
}

void GridMap::BresenhamCellOccupied(const Index2D &end_point){
    SetCellOccupied(end_point);
}

void GridMap::BresenhamCellFree(const Index2D &begin_point, const Index2D &end_point){
    BrasenHam(begin_point.x, begin_point.y, end_point.x, end_point.y);
}

void GridMap::BrasenHam(int x0, int y0, int x1, int y1)
{
    int dx = std::abs( x1 - x0 );
    int dy = std::abs( y1 - y0 );
    bool inter_change = false;
    int e = -dx;// error
    int signX = x1 > x0 ? 1 : ( ( x1 < x0 ) ? -1 : 0 );
    int signY = y1 > y0 ? 1 : ( ( y1 < y0 ) ? -1 : 0 );
    if (dy > dx) {
        int temp = dx; dx = dy; dy = temp; inter_change = true;
    }

    int x = x0, y = y0;
    for (int i = 1; i <= dx; i++) { 
        SetCellFree(Index2D(x, y));

        if (!inter_change) {x += signX;}
        else {y += signY;}
        e += 2 * dy;
        if (e >= 0) {
            if (!inter_change) {y += signY;}
            else {x += signX;}
            e -= 2 * dx;
        }
    }
}

void GridMap::UpdateMapLimit(const Index2D &index){
    if(index.x < m_map_limit(0, 0)) m_map_limit(0, 0) = index.x;
    if(index.x > m_map_limit(0, 1)) m_map_limit(0, 1) = index.x;
    if(index.y < m_map_limit(1, 0)) m_map_limit(1, 0) = index.y;
    if(index.y > m_map_limit(1, 1)) m_map_limit(1, 1) = index.y;
}

void GridMap::SetCellOccupied(const Index2D& cell_index){
    if(!IsInMapRange(cell_index))
        return;

    constexpr double log_odds_p_occ = 0.6;
    m_data(cell_index.x, cell_index.y) += log_odds_p_occ;
}

void GridMap::SetCellFree(const Index2D& cell_index){
    if(!IsInMapRange(cell_index))
        return;

    constexpr double log_odds_p_free = 0.6;
    m_data(cell_index.x, cell_index.y) -= log_odds_p_free;;
}

double GridMap::GetCellLogOdds(const Index2D& cell_index) const{
    if(!IsValid(cell_index))
        return 0.0;

    return m_data(cell_index.x, cell_index.y);
}

void GridMap::ResetMapLimit(){
    m_map_limit = Eigen::MatrixXi::Zero(2, 2);
    m_map_limit << m_half_size[0], m_half_size[0],
                   m_half_size[1], m_half_size[1];
}

Index2D GridMap::ConvertToCellIndex(const Index2D& index) const
{
    return Index2D(index.x - m_half_size[0], index.y - m_half_size[1]);
}

Index2D GridMap::ConvertToMapIndex(const Index2D& index) const
{
    return Index2D(index.x + m_half_size[0], index.y + m_half_size[1]);
}

bool GridMap::IsInMapRange(const Index2D& cell_index) const
{
    if(cell_index.x < 0 || cell_index.x >= m_data.rows())
        return false;

    if(cell_index.y < 0 || cell_index.y >= m_data.cols())
        return false;
    
    return true;
}

void GridMap::SaveProbMap(const std::string& file_name) const
{
    std::ofstream file(file_name);
    if(!file.is_open()){
        std::cout << "Open file failed!" << std::endl;
        return;
    }

    file << m_resolution << " " << m_data.rows() << " " << m_data.cols() << " " 
                        << m_map_limit(0, 0) << " " << m_map_limit(0, 1) << " " 
                        << m_map_limit(1, 0) << " " << m_map_limit(1, 1) << std::endl;

    for(int i = m_map_limit(0, 0); i <= m_map_limit(0, 1); ++i){
        for(int j = m_map_limit(1, 0); j <= m_map_limit(1, 1); ++j){
            file << std::setw(3) << int(100 / (1.0 + std::exp(-m_data(i, j)))) << " ";
        }
        file << std::endl;
    }
    
    file.close();
}

void GridMap::SaveLogOddsMap(const std::string& file_name) const
{
    std::ofstream file(file_name);
    if(!file.is_open()){
        std::cout << "Open file failed!" << std::endl;
        return;
    }

    file << m_resolution << " " << m_data.rows() << " " << m_data.cols() << " " 
                        << m_map_limit(0, 0) << " " << m_map_limit(0, 1) << " " 
                        << m_map_limit(1, 0) << " " << m_map_limit(1, 1) << std::endl;

    for(int i = m_map_limit(0, 0); i <= m_map_limit(0, 1); ++i){
        for(int j = m_map_limit(1, 0); j <= m_map_limit(1, 1); ++j){
            file << m_data(i, j) << " ";
        }
        file << std::endl;
    }
    file.close();
}





