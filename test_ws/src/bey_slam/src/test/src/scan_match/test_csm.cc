#include "core/tt_test.h"
#include "common/log.h"
#include "utils/read_data.h"
#include "pose_manager/pose_predict.h"
#include "grid_map/grid_map.h"
#include "utils/visualize.h"
#include "scan_match/csm.h"

// JUST_RUN_TEST(csm, test)
TEST(csm, test)
{
    ReadData read_data;
    ros::Rate loop_rate(30); // Hz
    Visualize visualize;

    TimedPose2D slam_pose;
    PosePredict pose_predict;
    GridMap grid_map(0.05, 10, 10);

    while (ros::ok())
    {
        if(!read_data.GetScanData().Empty()) {
            pose_predict.AddSlamPose(slam_pose);

            TimedLaserScan timed_scan = read_data.GetScanData().PopFront();
            pose_predict.GetPredictPose(timed_scan.GetTime(), slam_pose);

            // LOG_TEST(slam_pose);

            grid_map.UpdateByScan(slam_pose.pose, timed_scan.GetPoints());

            visualize.PublishGridMap(grid_map);
            visualize.PublishTrajectory(slam_pose);
        }

        if(!read_data.GetOdomData().Empty()) {
            pose_predict.AddNewOdom(read_data.GetOdomData().PopFront());
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    grid_map.SaveProbMap("/root/ws/test_ws/src/bey_slam/src/test/log/prob_map.txt");
}
