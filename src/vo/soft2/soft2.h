#ifndef __SODFT2_H__
#define __SODFT2_H__


#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <algorithm>
#include <memory>

#include "core/tt_tictoc.h"
#include "core/tt_log.h"
#include "pt2polar_factor.h"
#include "factor_graph/graph_optimize.h"
#include "ba_factor.h" 
#include "optimize.h"  
#include "common/data_que.h"

/*
1. 通过左相机前后两帧对极约束求解相对R,t
2. 通过右相机标定scale与外参R, 并更新R, t
3. 加入滑窗优化
*/

class Soft2
{
public:
    struct Option
    {
        uint max_feat_cnt;
        uint min_feat_cnt;
        uint min_track_cnt;
        uint min_feat_dist;
        uint min_disparity;
        uint max_epipolar;

        Option(uint max_feat_cnt = 500, uint min_feat_cnt = 50, uint min_track_cnt = 50, 
                uint min_feat_dist = 20, uint min_disparity = 2, uint max_epipolar = 5)
            : max_feat_cnt(max_feat_cnt), min_feat_cnt(min_feat_cnt), min_track_cnt(min_track_cnt), 
                min_feat_dist(min_feat_dist), min_disparity(min_disparity), max_epipolar(max_epipolar) {}
    };

    Soft2(const Eigen::Matrix<double, 3, 4>& camera_matrix, const Option& option = Option())
        : option_(option), R_(Eigen::Matrix3d::Identity()), t_(Eigen::Vector3d::Zero()), scale_(1.0), ext_r_(Eigen::Vector3d::Zero())
    {
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = camera_matrix(0, 0);
        camera_matrix_.at<double>(0, 2) = camera_matrix(0, 2);
        camera_matrix_.at<double>(1, 1) = camera_matrix(1, 1);
        camera_matrix_.at<double>(1, 2) = camera_matrix(1, 2);
        baseline_ = fabs(camera_matrix(0, 3));
        // optimize_ = std::make_unique<Optimize>(camera_matrix.block<3, 3>(0, 0), baseline_/1000.0);
    }

    void update(cv::Mat& left_img, cv::Mat& right_img, bool visulization = false)
    {
        if (left_prev_feats_.empty() || right_prev_feats_.empty()) {
            stereo_detect(left_img, right_img);
        } else  {
            stereo_track(left_img, right_img, visulization);
            stereo_detect(left_img, right_img);
        }
    }
private:
    void stereo_detect(cv::Mat& left_img, cv::Mat& right_img)
    {
        int thresh = 10;
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(thresh);
        std::vector<cv::KeyPoint> left_keypoints;
        detector->detect(left_img, left_keypoints);
        
        cv::Mat mask = cv::Mat(left_img.size(), CV_8UC1, cv::Scalar(255));
        static auto cmp = [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool { return a.response > b.response; };
        std::sort(left_keypoints.begin(), left_keypoints.end(), cmp);
        std::vector<cv::Point2f> left_feats, right_feats;
        for(uint i = 0; i < left_keypoints.size(); i++) {
            if (left_feats.size() < option_.max_feat_cnt && mask.at<unsigned char>(left_keypoints[i].pt.y, left_keypoints[i].pt.x))
            {
                left_feats.push_back(left_keypoints[i].pt);
                circle(mask, left_keypoints[i].pt, option_.min_feat_dist, cv::Scalar(0), cv::FILLED);
            }
        }


        uint max_point_size = 500;
        std::vector<cv::Point2f> left_feats2;
        for(uint i = 0; i < left_keypoints.size(); i++) {
            if(left_feats2.size() < max_point_size && mask.at<unsigned char>(left_keypoints[i].pt.y, left_keypoints[i].pt.x))
            {
                // 在col:[300, 1000], row:[200, 375]
                if(left_keypoints[i].pt.x < 400 || left_keypoints[i].pt.x > 700) continue;
                if(left_keypoints[i].pt.y < 240 || left_keypoints[i].pt.y > 375) continue;
                left_feats2.push_back(left_keypoints[i].pt);
                circle(mask, left_keypoints[i].pt, option_.min_feat_dist, cv::Scalar(0), cv::FILLED);
            }
        }
        
        calcOpticalFlowPyrLK(left_img, right_img, left_feats, right_feats);

        left_prev_feats_ = left_feats;
        right_prev_feats_ = right_feats;

        prev_left_frame_ = left_img.clone();
        prev_right_frame_ = right_img.clone();
    }

    uint stereo_track(cv::Mat& cur_left_frame, cv::Mat& cur_right_frame, bool visulization = false)
    {
        std::vector<cv::Point2f> left_curr_feats;
        calcOpticalFlowPyrLK(prev_left_frame_, cur_left_frame, left_prev_feats_, left_curr_feats);

        Eigen::Matrix3d dR = Eigen::Matrix3d::Zero();
        Eigen::Vector3d dt = Eigen::Vector3d::Zero();
        update_relative_pose(left_prev_feats_, left_curr_feats, dR, dt);

        optimize(dR, dt, cur_left_frame, cur_right_frame);

        update_odom(dR, dt);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R_;
        T.block<3, 1>(0, 3) = t_;
        ba_optimize(cur_left_frame, left_curr_feats, T);
        // LOG_TEST("R_: \n", R_);
        LOG_TEST("t_: ", t_.transpose());

        return 0;
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

    void remove_ouliners(std::vector<cv::Point2f>& prev_feats, std::vector<cv::Point2f>& curr_feats)
    {
        std::vector<uchar> status;
        cv::findFundamentalMat(curr_feats, prev_feats, cv::RANSAC, 0.999, 1.0, status);

        remove_outliers(prev_feats, status);
        remove_outliers(curr_feats, status); 
    }

    void update_relative_pose(std::vector<cv::Point2f>& prev_feats, std::vector<cv::Point2f>& curr_feats, Eigen::Matrix3d& R, Eigen::Vector3d& t)
    {
        std::vector<uchar> status;
        cv::Mat E = cv::findEssentialMat(curr_feats, prev_feats, camera_matrix_, cv::RANSAC, 0.999, 1.0, status);

        cv::Mat rev_R, rev_t;
        cv::recoverPose(E, curr_feats, prev_feats, camera_matrix_, rev_R, rev_t);

        R << rev_R.at<double>(0, 0), rev_R.at<double>(0, 1), rev_R.at<double>(0, 2),
             rev_R.at<double>(1, 0), rev_R.at<double>(1, 1), rev_R.at<double>(1, 2),
             rev_R.at<double>(2, 0), rev_R.at<double>(2, 1), rev_R.at<double>(2, 2);
        
        t << rev_t.at<double>(0), rev_t.at<double>(1), rev_t.at<double>(2);

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

    void ba_optimize(cv::Mat& cur_left_frame, const std::vector<cv::Point2f>& left_feats, const Eigen::Matrix4d& T)
    {
        left_frame_que_.push(cur_left_frame);
        left_feats_que_.push(left_feats);
        T_que_.push(T);
        scale_que_.push(scale_);

        if(left_frame_que_.size() < 4) return;

        Eigen::Matrix4d T12 = T_que_[0].inverse() * T_que_[1];
        Eigen::Matrix4d T23 = T_que_[1].inverse() * T_que_[2];
        Eigen::Matrix4d T34 = T_que_[2].inverse() * T_que_[3];
        double scale23 = scale_que_[2] / scale_que_[0];
        double scale34 = scale_que_[3] / scale_que_[0];

        Eigen::VectorXd x(20);
        x << Matrix2Vector(T12), Matrix2Vector(T23), Matrix2Vector(T34), scale23, scale34;
        LinearRegressionVariable *v_a = new LinearRegressionVariable(x);

        FactorGraph graph;
        graph.AddVariable(v_a);

        // BaFactor(LinearRegressionVariable *v_a, const Eigen::VectorXd& measurement, const Eigen::Matrix3d& K, const uint& mode = 0) 

        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = camera_matrix_.at<double>(0, 0);
        K(1, 1) = camera_matrix_.at<double>(1, 1);
        K(1, 2) = camera_matrix_.at<double>(1, 2);
        K(0, 2) = camera_matrix_.at<double>(0, 2);

        // 1, 3
        std::vector<cv::Point2f> feats;
        calcOpticalFlowPyrLK(left_frame_que_[0], left_frame_que_[2], left_feats_que_[0], feats);
        for(uint i = 0; i < left_feats_que_[0].size(); i++) {
            Eigen::Vector3d p1 = Eigen::Vector3d(left_feats_que_[0][i].x, left_feats_que_[0][i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(feats[i].x, feats[i].y, 1);

            Eigen::Matrix<double, 6, 1> measurement;
            measurement << p0, p1;

            BaFactor *f = new BaFactor(v_a, measurement, K, 0); 
            graph.AddFactor(f);
        }

        // 2, 4
        calcOpticalFlowPyrLK(left_frame_que_[1], left_frame_que_[3], left_feats_que_[1], feats);
        for(uint i = 0; i < left_feats_que_[1].size(); i++) {
            Eigen::Vector3d p1 = Eigen::Vector3d(left_feats_que_[1][i].x, left_feats_que_[1][i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(feats[i].x, feats[i].y, 1);

            Eigen::Matrix<double, 6, 1> measurement;
            measurement << p0, p1;

            BaFactor *f = new BaFactor(v_a, measurement, K, 1);
            graph.AddFactor(f);
        }

        // // 1, 4
        // calcOpticalFlowPyrLK(left_frame_que_[0], left_frame_que_[3], left_feats_que_[0], feats);
        // for(uint i = 0; i < left_feats_que_[0].size(); i++) {
        //     Eigen::Vector3d p1 = Eigen::Vector3d(left_feats_que_[0][i].x, left_feats_que_[0][i].y, 1);
        //     Eigen::Vector3d p0 = Eigen::Vector3d(feats[i].x, feats[i].y, 1);

        //     Eigen::Matrix<double, 6, 1> measurement;
        //     measurement << p0, p1;

        //     BaFactor *f = new BaFactor(v_a, measurement, K, 2);
        //     graph.AddFactor(f);
        // }

        // 优化
        GraphOptimize::Option option = GraphOptimize::Option();
        GraphOptimize graph_optimize = GraphOptimize(option);

        graph_optimize.OptimizeGN(&graph);

        Eigen::VectorXd x_opt = v_a->x();

        std::cout << "x_opt: \n" << x_opt.transpose() << std::endl;
    }

    Eigen::VectorXd Matrix2Vector(const Eigen::Matrix4d& T)
    {
        Eigen::VectorXd x(6);
        x << T.block<3, 3>(0, 0).eulerAngles(2, 1, 0).reverse(), T.block<3, 1>(0, 3);
        return x;
    }

    void optimize(Eigen::Matrix3d& dR, Eigen::Vector3d& dt, cv::Mat& cur_left_frame, cv::Mat& cur_right_frame)
    {
        TicTocAuto t("optimize");
        FactorGraph graph;
        Pt2PolarVariable *v_a = new Pt2PolarVariable(Eigen::Vector4d(ext_r_(0), ext_r_(1), ext_r_(2), 1)); // alpha, beta, gamma, scale
        graph.AddVariable(v_a);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = dR;
        T.block<3, 1>(0, 3) = dt;

        // LOG_TEST("T: \n", T);

        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = camera_matrix_.at<double>(0, 0);
        K(1, 1) = camera_matrix_.at<double>(1, 1);
        K(1, 2) = camera_matrix_.at<double>(1, 2);
        K(0, 2) = camera_matrix_.at<double>(0, 2);
        double base_line = 0.386145;
        // std::cout << "K: \n" << K << std::endl;
    
        // curr_right -> prev_left
        std::vector<cv::Point2f> curr_right_feats;
        calcOpticalFlowPyrLK(prev_left_frame_, cur_right_frame, left_prev_feats_, curr_right_feats);
        for(uint i = 0; i < left_prev_feats_.size(); i++) {
            Eigen::Vector3d p1 = Eigen::Vector3d(left_prev_feats_[i].x, left_prev_feats_[i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(curr_right_feats[i].x, curr_right_feats[i].y, 1);
            Pt2PolarFactor *f = new Pt2PolarFactor(v_a, T, base_line, K, p0, p1); // p1 for line, p0 for point, // 3种模式, 代表三种坐标变换方式
            graph.AddFactor(f);
        }
        
        // curr_left -> prev_right
        std::vector<cv::Point2f> curr_left_feats;
        calcOpticalFlowPyrLK(prev_right_frame_, cur_left_frame, right_prev_feats_, curr_left_feats);
        for(uint i = 0; i < right_prev_feats_.size(); i++)
        {
            Eigen::Vector3d p1 = Eigen::Vector3d(right_prev_feats_[i].x, right_prev_feats_[i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(curr_left_feats[i].x, curr_left_feats[i].y, 0);
            Pt2PolarFactor *f = new Pt2PolarFactor(v_a, T, base_line, K, p0, p1); // p1 for line, p0 for point
            graph.AddFactor(f);
        }

        // curr_right -> prev_right
        std::vector<cv::Point2f> curr_right_feats2;
        calcOpticalFlowPyrLK(prev_right_frame_, cur_right_frame, right_prev_feats_, curr_right_feats2);
        for(uint i = 0; i < right_prev_feats_.size(); i++)
        {
            Eigen::Vector3d p1 = Eigen::Vector3d(right_prev_feats_[i].x, right_prev_feats_[i].y, 1);
            Eigen::Vector3d p0 = Eigen::Vector3d(curr_right_feats2[i].x, curr_right_feats2[i].y, 1);
            Pt2PolarFactor *f = new Pt2PolarFactor(v_a, T, base_line, K, p0, p1); // p1 for line, p0 for point
            graph.AddFactor(f);
        }

        // 优化
        GraphOptimize::Option option = GraphOptimize::Option();
        GraphOptimize graph_optimize = GraphOptimize(option);

        graph_optimize.OptimizeGN(&graph);

        v_a->Print();

        scale_ = v_a->scale();
        ext_r_ = v_a->x().head<3>();

        // LOG_TEST("scale: ", scale_);
        // LOG_TEST("ext_r_: \n", v_a->ext_R()); 
    }

    void update_odom(const Eigen::Matrix3d& dR, const Eigen::Vector3d& dt)
    {
        t_ = t_ + dR * (dt * scale_);
        R_ = R_ * dR;
    }
private:
    Option option_;
    cv::Mat camera_matrix_;
    double baseline_;

    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;

    double scale_;          // 尺度
    Eigen::Vector3d ext_r_; // 基于有相机原点, 相对于左相机的偏差

    std::vector<cv::Point2f> left_prev_feats_;
    std::vector<cv::Point2f> right_prev_feats_;
    cv::Mat prev_left_frame_;
    cv::Mat prev_right_frame_;

    DataQue<std::vector<cv::Point2f>> left_feats_que_;
    DataQue<cv::Mat> left_frame_que_;
    DataQue<Eigen::Matrix4d> T_que_;
    DataQue<double> scale_que_;

    std::unique_ptr<Optimize> optimize_;
};



#endif // __SODFT2_H__