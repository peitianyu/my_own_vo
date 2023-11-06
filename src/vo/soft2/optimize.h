#ifndef __OPTIMIZE_H__
#define __OPTIMIZE_H__

#include "pt2polar_factor.h"
#include "factor_graph/graph_optimize.h"
#include "ba_factor.h" 
#include <opencv2/opencv.hpp> 
#include "common/data_que.h"

class Optimize
{
public:
    Optimize(const Eigen::Matrix3d& camera_matrix, const double& baseline)
    {

    }

    void set_frame(const cv::Mat& left_img, const cv::Mat& right_img)
    {
        left_frame_que_.push(left_img.clone());
        right_frame_que_.push(right_img.clone());
    }

    void set_pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
    {
        R_que_.push(R);
        t_que_.push(t);
    }

    void set_feats(const std::vector<cv::Point2f>& left_feats, const std::vector<cv::Point2f>& right_feats)
    {
        left_feats_que_.push(left_feats);
        right_feats_que_.push(right_feats);
    }

    Eigen::VectorXd ext_calib(Eigen::Matrix3d& dR, Eigen::Vector3d& dt, cv::Mat& cur_left_frame, cv::Mat& cur_right_frame)
    {
        FactorGraph graph;
        Pt2PolarVariable *v_a = new Pt2PolarVariable(Eigen::Vector4d(ext_r_(0), ext_r_(1), ext_r_(2), 1)); // alpha, beta, gamma, scale
        graph.AddVariable(v_a);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = dR;
        T.block<3, 1>(0, 3) = dt;

        Eigen::Matrix3d K = camera_matrix_;

        std::cout << "K: \n" << K << std::endl;
        std::cout << "base_line: " << baseline_ << std::endl;
    
        // curr_right -> prev_left
        std::vector<cv::Point2f> curr_right_feats;
        calcOpticalFlowPyrLK(left_frame_que_.back(), cur_right_frame, left_feats_que_.back(), curr_right_feats);
        for(uint i = 0; i < left_feats_que_.back().size(); i++) {
            Eigen::Vector3d p1 = Eigen::Vector3d(left_feats_que_.back()[i].x, left_feats_que_.back()[i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(curr_right_feats[i].x, curr_right_feats[i].y, 1);
            Pt2PolarFactor *f = new Pt2PolarFactor(v_a, T, baseline_, K, p0, p1); // p1 for line, p0 for point, // 3种模式, 代表三种坐标变换方式
            graph.AddFactor(f);
        }
        
        // curr_left -> prev_right
        std::vector<cv::Point2f> curr_left_feats;
        std::vector<cv::Point2f> right_feats = right_feats_que_.back();
        calcOpticalFlowPyrLK(right_frame_que_.back(), cur_left_frame, right_feats, curr_left_feats);
        for(uint i = 0; i < right_feats.size(); i++)
        {
            Eigen::Vector3d p1 = Eigen::Vector3d(right_feats[i].x, right_feats[i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(curr_left_feats[i].x, curr_left_feats[i].y, 0);
            Pt2PolarFactor *f = new Pt2PolarFactor(v_a, T, baseline_, K, p0, p1); // p1 for line, p0 for point
            graph.AddFactor(f);
        }

        // curr_right -> prev_right
        std::vector<cv::Point2f> curr_right_feats2;
        calcOpticalFlowPyrLK(right_frame_que_.back(), cur_right_frame, right_feats, curr_right_feats2);
        for(uint i = 0; i < right_feats.size(); i++)
        {
            Eigen::Vector3d p1 = Eigen::Vector3d(right_feats[i].x, right_feats[i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(curr_right_feats2[i].x, curr_right_feats2[i].y, 1);
            Pt2PolarFactor *f = new Pt2PolarFactor(v_a, T, baseline_, K, p0, p1); // p1 for line, p0 for point
            graph.AddFactor(f);
        }

        // 优化
        GraphOptimize::Option option = GraphOptimize::Option();
        GraphOptimize graph_optimize = GraphOptimize(option);

        graph_optimize.OptimizeGN(&graph);

        return v_a->x();
    }

    void ba_optimize()
    {

    }
private:
    void calcOpticalFlowPyrLK(cv::Mat& prev_frame, cv::Mat& cur_frame, std::vector<cv::Point2f>& prev_feats, std::vector<cv::Point2f>& curr_feats)
    {
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, prev_feats, curr_feats, status, err);
        
        remove_outliers(prev_feats, status);
        remove_outliers(curr_feats, status);
    }

    void remove_outliers(std::vector<cv::Point2f>& feats, const std::vector<uchar>& status)
    {
        uint j = 0;
        for(uint i = 0; i < status.size(); i++) {
            if(!status[i]) continue;
            feats[j] = feats[i];
            j++;
        }

        feats.resize(j);
    }
private:
    Eigen::Matrix3d camera_matrix_;
    double baseline_;

    double scale_;
    Eigen::Vector3d ext_r_;

    DataQue<std::vector<cv::Point2f>> left_feats_que_;
    DataQue<std::vector<cv::Point2f>> right_feats_que_;
    DataQue<cv::Mat> left_frame_que_;
    DataQue<cv::Mat> right_frame_que_;

    DataQue<Eigen::Matrix3d> R_que_;
    DataQue<Eigen::Vector3d> t_que_;
};

#endif // __OPTIMIZE_H__