#include "core/tt_test.h"
#include "common/log.h"
#include "utils/read_data.h"

// JUST_RUN_TEST(read_data, test)
TEST(read_data, test)
{
    ReadData read_data;
    ros::Rate loop_rate(30); // Hz

    while (ros::ok())
    {
        std::cout << "Scan.size = " << read_data.GetScanData().Back().GetPoints().size() << std::endl;
        std::cout << "Imu.size = " << read_data.GetImuData().Size() << std::endl;
        std::cout << "Odom().size = " << read_data.GetOdomData().Size() << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
}
