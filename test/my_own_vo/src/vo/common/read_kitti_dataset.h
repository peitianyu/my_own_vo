#ifndef __READ_KITTI_DATASET_H__
#define __READ_KITTI_DATASET_H__

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>

class ReadKittiDataset
{
public:
    struct ImageData
    {
        double timestamp;
        std::string left_img;
        std::string right_img;

        ImageData(double timestamp, std::string left_img, std::string right_img)
            : timestamp(timestamp), left_img(left_img), right_img(right_img) {}
    };

    ReadKittiDataset(const std::string& sequence_path, const std::string& sequence_num){
        load_calib(sequence_path + "sequences/" + sequence_num + "/calib.txt");
        load_image(sequence_path, sequence_num);
        load_ground_truth(sequence_path+"poses/"+sequence_num+".txt");
    }

    const std::vector<ImageData>& get_image_data() const { return image_data_; }
    const Eigen::Matrix<double, 3, 4>& get_left_cam() const { return left_cam_; }
    const Eigen::Matrix<double, 3, 4>& get_right_cam() const { return right_cam_; }
    Eigen::Quaterniond get_ground_truth_q(uint idx) const { return Eigen::Quaterniond(ground_truth_[idx].block<3, 3>(0, 0)); }
    Eigen::Vector3d get_ground_truth_t(uint idx) const { return ground_truth_[idx].block<3, 1>(0, 3); }
private:
    void load_image(const std::string& sequence_path, const std::string& sequence_num)
    {
        std::string img_path = sequence_path + "sequences/" + sequence_num;
        std::string image_timestamp_path = img_path + "/times.txt";

        std::ifstream image_timestamp_file(image_timestamp_path);
        uint line_num = 0;
        std::string line;
        while (std::getline(image_timestamp_file, line))
        {
            std::stringstream line_num_str;
            line_num_str << std::setfill('0') << std::setw(6) << line_num << ".png";

            image_data_.emplace_back(std::stod(line), img_path+"/image_0/"+line_num_str.str(), img_path+"/image_1/"+line_num_str.str());

            line_num++;
        }
    }

    void load_calib(const std::string& calib_path)
    {
        left_cam_ = Eigen::Matrix<double, 3, 4>::Zero();
        right_cam_ = Eigen::Matrix<double, 3, 4>::Zero();

        std::ifstream calib_file(calib_path);
        std::string line;
        while (std::getline(calib_file, line))
        {
            std::stringstream line_stream(line);
            std::string calib_type;
            line_stream >> calib_type;
            if (calib_type == "P0:")
                for (uint i = 0; i < 12; i++) line_stream >> left_cam_(i/4, i%4); 
            else if (calib_type == "P1:")
                for (uint i = 0; i < 12; i++) line_stream >> right_cam_(i/4, i%4); 
        }
    }

    void load_ground_truth(const std::string& ground_truth_path)
    {
        std::ifstream ground_truth_file(ground_truth_path);
        std::string line;
        while (std::getline(ground_truth_file, line))
        {
            Eigen::Matrix<double, 3, 4> pose = Eigen::Matrix<double, 3, 4>::Zero();
            std::stringstream line_stream(line);
            for (uint i = 0; i < 12; i++) line_stream >> pose(i/4, i%4);
            ground_truth_.push_back(pose);
        }
    }
private:
    std::vector<ImageData> image_data_;
    Eigen::Matrix<double, 3, 4> left_cam_;
    Eigen::Matrix<double, 3, 4> right_cam_;
    std::vector<Eigen::Matrix<double, 3, 4>> ground_truth_;
};






#endif // !__READ_KITTI_DATASET_H__