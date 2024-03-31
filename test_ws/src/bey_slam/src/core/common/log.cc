#include "log.h"

std::string get_file_name(const std::string &file_name) {
    size_t pos = file_name.find_last_of('/');
    if (pos == std::string::npos) return file_name; 
    return file_name.substr(pos + 1);
}

#include <chrono>
std::string get_curr_time()
{
    typedef std::chrono::high_resolution_clock Clock;

    static auto start = Clock::now();
    return "["+std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - start).count())+"] ";
}