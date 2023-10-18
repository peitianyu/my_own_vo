#include "core/tt_test.h"
#include "core/tt_log.h"

#include <fstream>
// JUST_RUN_TEST(log, test)
TEST(log, test)
{
    LOG("hello world");

    // std::ofstream ofs("log.txt", std::ios::app);
    // LOG_FILE(ofs, "hello world\n");

    LOG_TEST("hello world");
}