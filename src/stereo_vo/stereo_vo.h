#ifndef __STEREO_VO_H__
#define __STEREO_VO_H__

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <algorithm>


class StereoVO
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

        Option(uint max_feat_cnt = 1000, uint min_feat_cnt = 50, uint min_track_cnt = 50, 
                uint min_feat_dist = 30, uint min_disparity = 2, uint max_epipolar = 5)
            : max_feat_cnt(max_feat_cnt), min_feat_cnt(min_feat_cnt), min_track_cnt(min_track_cnt), 
                min_feat_dist(min_feat_dist), min_disparity(min_disparity), max_epipolar(max_epipolar) {}
    };

    StereoVO(const Eigen::Matrix<double, 3, 4>& camera_matrix, const Option& option = Option())
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
        if (feat3ds_.empty() || feats_.empty()) {
            stereo_detect(left_img, right_img);
        } else  {
            uint feat_cnt = stereo_track(ref_frame_, left_img, visulization);
            if (feat_cnt < option_.min_feat_cnt) stereo_detect(left_img, right_img); 
        }
    }

    Eigen::Quaterniond get_q() const { return q_; }
    Eigen::Vector3d get_t() const { return t_; }
    std::vector<cv::Point3f> get_feat3ds() const { return feat3ds_; }
private:
    uint stereo_track(cv::Mat &ref_frame, cv::Mat &img, bool visulization)
    {
        std::vector<uchar> status;
        std::vector<float> err;
        std::vector<cv::Point2f> feats_curr;
        cv::calcOpticalFlowPyrLK(ref_frame, img, feats_, feats_curr, status, err, cv::Size(21, 21), 3);

        std::vector<cv::Point2f> points, points_curr;
        std::vector<cv::Point3f> point3ds;
        for(uint i = 0; i < status.size(); i++) {
            if(!status[i]) continue;

            points.push_back(feats_[i]);
            points_curr.push_back(feats_curr[i]);
            point3ds.push_back(feat3ds_[i]);
        }
        
        remove_outliers(points, points_curr, point3ds);

        cv::Mat rvec, tvec;
        std::vector<uchar> inliers;
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        if(cv::solvePnPRansac(point3ds, points_curr, camera_matrix_, dist_coeffs, 
            rvec, tvec, false, 30, 6.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE))
        {
            update_odom(rvec, tvec);
        }

        if(visulization)
        {
            cv::Mat feats_img;
            cv::cvtColor(img, feats_img, cv::COLOR_GRAY2BGR);
            for(uint i = 0; i < points.size(); i++) {
                cv::circle(feats_img, points[i], 2, cv::Scalar(0, 0, 255), cv::FILLED);
                cv::line(feats_img, points[i], points_curr[i], cv::Scalar(0, 255, 0));
            }
            cv::imshow("feats", feats_img);
        }

        return std::count(inliers.begin(), inliers.end(), 1);
    }

    void stereo_detect(const cv::Mat& left_img, const cv::Mat& right_img)
    {
        std::vector<cv::KeyPoint> left_keypoints;
        std::vector<cv::Point2f> left_feats, right_feats;

        int thresh = 10;
        cv::Mat mask = cv::Mat(left_img.size(), CV_8UC1, cv::Scalar(255));
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(thresh);
        detector->detect(left_img, left_keypoints);

        static auto cmp = [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool { return a.response > b.response; };
        std::sort(left_keypoints.begin(), left_keypoints.end(), cmp);

        for(uint i = 0; i < left_keypoints.size(); i++) {
            if(left_feats.size() >= option_.max_feat_cnt) continue;
            if(!mask.at<unsigned char>(left_keypoints[i].pt.y, left_keypoints[i].pt.x)) continue;

            left_feats.push_back(left_keypoints[i].pt);
            cv::circle(mask, left_keypoints[i].pt, option_.min_feat_dist, cv::Scalar(0), cv::FILLED);
        }

        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(left_img, right_img, left_feats, right_feats, status, err, cv::Size(21, 21), 3);

        uint track_cnt = std::count(status.begin(), status.end(), 1);
        feat3ds_.resize(track_cnt);
        feats_.resize(track_cnt);

        std::vector<cv::Point2f> feats_tracked(track_cnt);

        double cx = camera_matrix_.at<double>(0, 2);
        double cy = camera_matrix_.at<double>(1, 2);
        double fx = camera_matrix_.at<double>(0, 0);
        double fy = camera_matrix_.at<double>(1, 1);

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
            feat3ds_[j] = p;
            feats_[j] = left_feats[i];
            feats_tracked[j] = right_feats[i];

            j++;
        }

        remove_outliers(feats_, feats_tracked, feat3ds_);

        ///////////////////////////////update//////////////////////////////////
        left_img.copyTo(ref_frame_);

        // std::cout << "pose: " << q_.coeffs().transpose() << " " << t_.transpose() << std::endl;
    }

    void remove_outliers(std::vector<cv::Point2f> &feats_prev, std::vector<cv::Point2f> &feats_curr, std::vector<cv::Point3f>& feat3ds)
    {
        std::vector<uchar> status;
        cv::findFundamentalMat(feats_prev, feats_curr, cv::FM_RANSAC, 1.0, 0.99, status);

        std::vector<cv::Point2f> f_prev, f_curr; 
        std::vector<cv::Point3f> f3ds;
        for(uint i = 0; i < status.size(); i++) {
            if(!status[i]) continue;

            f_prev.push_back(feats_prev[i]);
            f_curr.push_back(feats_curr[i]);
            f3ds.push_back(feat3ds[i]);
        }

        feats_prev = f_prev;
        feats_curr = f_curr;
        feat3ds = f3ds;
    }

    void update_odom(const cv::Mat& rvec, const cv::Mat& tvec)
    {
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d dR;
        Eigen::Vector3d dt;
        cv::cv2eigen(rmat, dR);
        cv::cv2eigen(tvec, dt);

        dt = -dR.transpose() * dt;
        Eigen::Quaterniond dq(dR.transpose());
        t_ = t_ + q_.toRotationMatrix() * dt;
        q_ = q_ * dq;
    }
private:
    Option option_;
    //camera matrix
    cv::Mat camera_matrix_;
    double baseline_;
    
    Eigen::Quaterniond q_;
    Eigen::Vector3d t_;

    cv::Mat ref_frame_;
    std::vector<cv::Point3f> feat3ds_;
    std::vector<cv::Point2f> feats_;
};

#endif // !__STEREO_VO_H__