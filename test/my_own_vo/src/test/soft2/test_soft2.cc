#include "common/read_kitti_dataset.h"
#include "core/tt_ini_parse.h"
#include "core/tt_log.h"
#include "core/tt_test.h"
#include "soft2/soft2.h"

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <fstream>
#include <iostream>

static Eigen::Matrix<double, 3, 4>
camera_matrix(const Eigen::Matrix<double, 5, 1> &camera_info) {
  Eigen::Matrix<double, 3, 4> camera_matrix =
      Eigen::Matrix<double, 3, 4>::Ones();
  camera_matrix << camera_info(0), 0, camera_info(2), camera_info(4), 0,
      camera_info(1), camera_info(3), 0, 0, 0, 1, 0;
  return camera_matrix;
}

// JUST_RUN_TEST(soft2, test)
TEST(soft2, test) {
  IniParse ini_parser("../config/config.ini");

  std::string dataset_path =
      ini_parser.get_val<std::string>("SYSTEM", "dataset_path");
  std::string sequence_num =
      ini_parser.get_val<std::string>("SYSTEM", "sequence_number");
  std::string ground_truth_path =
      dataset_path + "poses/" + sequence_num + ".txt";
  Eigen::Matrix<double, 5, 1> camera_info =
      ini_parser.get_vec<double>("CAMERA", "camera_info");

  ReadKittiDataset read_kitti_dataset(dataset_path, sequence_num);
  std::vector<ReadKittiDataset::ImageData> image_data =
      read_kitti_dataset.get_image_data();
  LOG_TEST("image_data size: ", image_data.size());

  std::ofstream vo_ofs("../log/vo_" + sequence_num + ".txt", std::ios::out);
  std::ofstream gt_ofs("../log/gt_" + sequence_num + ".txt", std::ios::out);

  Soft2 soft2(camera_matrix(camera_info));
  for (uint i = 0; i < 7; ++i)
  // for(uint i = 0; i < image_data.size(); ++i)
  {
    std::cout << "-----------------------------------------------------------"
              << std::endl;
    cv::Mat left_img = cv::imread(image_data[i].left_img, cv::IMREAD_GRAYSCALE);
    cv::Mat right_img =
        cv::imread(image_data[i].right_img, cv::IMREAD_GRAYSCALE);
    soft2.update(left_img, right_img, false);

    // Eigen::Vector3d euler_angle =
    // soft2.get_q().toRotationMatrix().eulerAngles(2, 1, 0); LOG_TEST("seq: ",
    // i," ", soft2.get_t().transpose());
  }
}
