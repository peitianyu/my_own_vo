#include "core/tt_test.h"
#include "core/tt_log.h"
#include "core/tt_ini_parse.h"
#include "stereo_vo/read_kitti_dataset.h"
#include "stereo_vo/stereo_vo.h"

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>


static Eigen::Matrix<double, 3, 4> camera_matrix(const Eigen::Matrix<double, 5, 1>& camera_info)
{
    Eigen::Matrix<double, 3, 4> camera_matrix = Eigen::Matrix<double, 3, 4>::Ones();
    camera_matrix << camera_info(0), 0, camera_info(2), camera_info(4),
                     0, camera_info(1), camera_info(3),              0,
                     0,              0,              1,              0;
    return camera_matrix;
}

JUST_RUN_TEST(stereo_vo, test)
TEST(stereo_vo, test)
{
    IniParse ini_parser("../config/config.ini");

    std::string dataset_path = ini_parser.get_val<std::string>("SYSTEM", "dataset_path");
    std::string sequence_num = ini_parser.get_val<std::string>("SYSTEM", "sequence_number");
    std::string ground_truth_path = dataset_path+"poses/"+sequence_num+".txt";
    Eigen::Matrix<double, 5, 1> camera_info = ini_parser.get_vec<double>("CAMERA", "camera_info");

    ReadKittiDataset read_kitti_dataset(dataset_path, sequence_num);
    std::vector<ReadKittiDataset::ImageData> image_data = read_kitti_dataset.get_image_data();
    LOG_TEST("image_data size: ", image_data.size());

    StereoVO stereo(camera_matrix(camera_info));
    for(uint i = 0; i < image_data.size(); ++i)
    {
        LOG_TEST("seq: ", i);

        cv::Mat left_img = cv::imread(image_data[i].left_img, cv::IMREAD_GRAYSCALE);
        cv::Mat right_img = cv::imread(image_data[i].right_img, cv::IMREAD_GRAYSCALE);
        stereo.update(left_img, right_img, false);

        Eigen::Matrix4d T = stereo.get_T();
        LOG_TO_FILE("../log/vo_"+sequence_num+".txt", 
                    T(0, 0), " ", T(0, 1), " ", T(0, 2), " ", T(0, 3), " ",
                    T(1, 0), " ", T(1, 1), " ", T(1, 2), " ", T(1, 3), " ",
                    T(2, 0), " ", T(2, 1), " ", T(2, 2), " ", T(2, 3));

        // LOG_TEST("vo: ", stereo.get_q().coeffs().transpose(), " ", stereo.get_t().transpose());
        // LOG_TEST("gt: ", read_kitti_dataset.get_ground_truth_q(i).coeffs().transpose(), 
        //          " ", read_kitti_dataset.get_ground_truth_t(i).transpose());

        // Eigen::Vector3d vo_t = stereo.get_t();
        // Eigen::Vector3d gt_t = read_kitti_dataset.get_ground_truth_t(i);
        // std::vector<cv::Point3f> point_cloud = stereo.get_feat3ds();

        // for(auto& p : point_cloud) {
        //     Eigen::Vector3d p_eigen(p.x, p.y, p.z);
        //     p_eigen = stereo.get_q().toRotationMatrix()*p_eigen + stereo.get_t();
        //     p = cv::Point3f(p_eigen(0), p_eigen(1), p_eigen(2));
        // }

        // static std::vector<cv::Point3f> vo_traj, gt_traj;
        // vo_traj.emplace_back(vo_t(0), vo_t(1), vo_t(2));
        // gt_traj.emplace_back(gt_t(0), gt_t(1), gt_t(2));

        // {
        //     // 可视化轨迹与点云
        //     static cv::viz::Viz3d viz("Visual Odometry");
        //     viz.setBackgroundColor(cv::viz::Color::white());

        //     cv::viz::WCloud cloud_widget(point_cloud, cv::viz::Color::red());
        //     viz.showWidget("cloud", cloud_widget);
        //     cv::viz::WCloud vo_traj_widget(vo_traj, cv::viz::Color::green());
        //     viz.showWidget("traj", vo_traj_widget);
        //     cv::viz::WCloud gt_traj_widget(gt_traj, cv::viz::Color::blue());
        //     viz.showWidget("gt_traj", gt_traj_widget);
            
        //     viz.spinOnce(1, false);
        // }
        // cv::waitKey(0);
    }
}

