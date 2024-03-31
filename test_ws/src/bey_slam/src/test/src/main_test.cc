#include "core/tt_test.h"
#include "core/tt_backtrace.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node"); 

    REGISTER_SEGFAULT_HANDLER

    return RunAllTests();
}