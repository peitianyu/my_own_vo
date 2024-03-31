#include "core/tt_test.h"
#include "common/log.h"
#include "utils/read_data.h"
#include "pose_manager/pose_predict.h"
#include "utils/visualize.h"

// JUST_RUN_TEST(pose_predict, test)
TEST(pose_predict, test)
{
    ReadData read_data;
    Visualize visualize;
    ros::Rate loop_rate(30); // Hz

    TimedPose2D slam_pose;
    PosePredict pose_predict;

    while (ros::ok())
    {
        if(!read_data.GetScanData().Empty()) {
            pose_predict.AddSlamPose(slam_pose);

            pose_predict.GetPredictPose(read_data.GetScanData().PopFront().GetTime(), slam_pose);

            std::cout << slam_pose << std::endl;

            visualize.PublishTrajectory(slam_pose);
        }

        if(!read_data.GetOdomData().Empty()) {
            pose_predict.AddNewOdom(read_data.GetOdomData().PopFront());
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}
