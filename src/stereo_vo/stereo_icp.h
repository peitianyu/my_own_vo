#ifndef __STEREO_ICP_H__
#define __STEREO_ICP_H__

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <algorithm>

#include "core/tt_tictoc.h"
#include "common/octree.h"

using Point = Eigen::Vector3d;
using OctreeMap = OctTree<Point, double>;

class StereoICP
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

        Option(uint max_feat_cnt = 500, uint min_feat_cnt = 50, uint min_track_cnt = 500, 
                uint min_feat_dist = 30, uint min_disparity = 2, uint max_epipolar = 5)
            : max_feat_cnt(max_feat_cnt), min_feat_cnt(min_feat_cnt), min_track_cnt(min_track_cnt), 
                min_feat_dist(min_feat_dist), min_disparity(min_disparity), max_epipolar(max_epipolar) {}
    };

    StereoICP(const Eigen::Matrix<double, 3, 4>& camera_matrix, const Option& option = Option())
        : option_(option), q_(Eigen::Quaterniond::Identity()), t_(Eigen::Vector3d::Zero())
    {
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = camera_matrix(0, 0);
        camera_matrix_.at<double>(0, 2) = camera_matrix(0, 2);
        camera_matrix_.at<double>(1, 1) = camera_matrix(1, 1);
        camera_matrix_.at<double>(1, 2) = camera_matrix(1, 2);
        baseline_ = fabs(camera_matrix(0, 3)/camera_matrix(0, 0));
    }

    void update(cv::Mat& left_img, cv::Mat& right_img, bool visulization = false)
    {
        if (feats_map_.empty()) {
            stereo_detect(left_img, right_img);
            
            update_map();
        } else  {
            stereo_detect(left_img, right_img);

            stereo_track(visulization);
        }
    }
private:
    void stereo_detect(cv::Mat& left_img, cv::Mat& right_img)
    {
        int thresh = 10;
        cv::Mat mask = cv::Mat(left_img.size(), CV_8UC1, cv::Scalar(255));
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(thresh);
        std::vector<cv::KeyPoint> left_keypoints;
        detector->detect(left_img, left_keypoints);

        static auto cmp = [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool { return a.response > b.response; };
        std::sort(left_keypoints.begin(), left_keypoints.end(), cmp);
        std::vector<cv::Point2f> left_feats, right_feats;
        for(uint i = 0; i < left_keypoints.size(); i++) {
            if(left_feats.size() >= option_.max_feat_cnt) break; 
            
            left_feats.push_back(left_keypoints[i].pt);
        }

        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(left_img, right_img, left_feats, right_feats, status, err);
        
        std::vector<cv::Point2f> feats_tracked(track_cnt);
        std::vector<cv::Point3f> feat3ds(track_cnt);
        to_point_cloud(status, left_feats, right_feats, feats_tracked, feat3ds);

        remove_outliers(curr_feats_, feats_tracked, feat3ds);
    }

    void stereo_track(bool visulization)
    {
        // icp匹配, 估计位姿
        if(1) update_map();
    }

    void update_map()
    {
        for(uint i = 0; i < curr_feats_.size(); i++) {
            feats_map_.push_back(t_ + q_.toRotationMatrix()*curr_feats_[i]);
        }
    }

    void to_point_cloud(const std::vector<uchar>& status, const std::vector<cv::Point2f> &left_feats, 
                        const std::vector<cv::Point2f> &right_feats, std::vector<cv::Point2f>& feats_tracked)
    {
        double cx = camera_matrix_.at<double>(0, 2);
        double cy = camera_matrix_.at<double>(1, 2);
        double fx = camera_matrix_.at<double>(0, 0);
        double fy = camera_matrix_.at<double>(1, 1);

        uint track_cnt = std::count(status.begin(), status.end(), 1);

        curr_feats_.resize(track_cnt);

        uint j = 0;
        for(uint i = 0; i < status.size(); i++) {
            if(!status[i]) continue;

            double dx = left_feats[i].x - right_feats[i].x;
            double dy = left_feats[i].y - right_feats[i].y;

            cv::Point3f p = cv::Point3f(0, 0, 0);
            if(fabs(dx) > option_.min_disparity && fabs(dy) < option_.max_epipolar){
                p.z = baseline_ * fx / dx;
                p.x = (left_feats[i].x - cx) * p.z / fx;
                p.y = (left_feats[i].y - cy) * p.z / fy;
            } 
            
            // std::cout << "p: " << p << std::endl;
            feat3ds[j] = p;
            curr_feats_[j] = left_feats[i];
            feats_tracked[j] = right_feats[i];

            j++;
        }
    }

    void remove_outliers(std::vector<cv::Point2f> &feats_prev, std::vector<cv::Point2f> &feats_curr, std::vector<cv::Point3f>& feat3ds)
    {
        std::vector<uchar> status;
        cv::findFundamentalMat(feats_prev, feats_curr, cv::FM_RANSAC, 1.0, 0.99, status);

        uint j = 0;
        for(uint i = 0; i < status.size(); i++) {
            if(!status[i]) continue;

            feats_prev[j] = feats_prev[i];
            feats_curr[j] = feats_curr[i];
            feat3ds[j] = feat3ds[i];

            j++;
        }

        feats_prev.resize(j);
        feats_curr.resize(j);
        feat3ds.resize(j);
    }
private:
    Option option_;
    //camera matrix
    cv::Mat camera_matrix_;
    double baseline_;

    Eigen::Quaterniond q_;
    Eigen::Vector3d t_;

    std::vector<Eigen::Vector3f> curr_feats_;
    std::vector<Eigen::Vector3f> feats_map_;
};





#endif // __STEREO_ICP_H__