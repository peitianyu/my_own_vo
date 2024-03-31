#include "core/tt_test.h"
#include "common/log.h"
#include "utils/read_data.h"
#include "slam/slam_normal.h"
#include "utils/visualize.h"

JUST_RUN_TEST(slam_normal, test)
TEST(slam_normal, test)
{
    ReadData read_data;
    ros::Rate loop_rate(30); // Hz
    Visualize visualize;

    SlamNormal slam_normal;
    while (ros::ok()) {
        if(!read_data.GetScanData().Empty()) {
            TimedLaserScan timed_scan = read_data.GetScanData().PopFront();
            slam_normal.AddNewScan(timed_scan);

            visualize.PublishGridMap(*slam_normal.GetGridMap());
            visualize.PublishTrajectory(*slam_normal.GetSlamPose());
            visualize.PublishScan(*slam_normal.GetSlamPose(), timed_scan.GetPoints());
        }

        if(!read_data.GetOdomData().Empty()) {
            slam_normal.AddNewOdom(read_data.GetOdomData().PopFront());
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    slam_normal.GetGridMap()->SaveProbMap("/root/ws/test_ws/src/bey_slam/src/test/log/prob_map.txt");
}
