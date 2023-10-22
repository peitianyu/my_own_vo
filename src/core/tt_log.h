#ifndef __LOG_H__
#define __LOG_H__

#include<iostream>
#include<fstream>

template <typename... Args, typename O>
O & print_base(O &out, Args&&... args) {
    auto a = {(out << std::forward<Args>(args), 0)...};
    (void)a;
    return out;
} 

std::string get_file_name(const std::string &file_name);
std::string get_curr_time();

#define TT_PRINT_RED       "\033[31m"
#define TT_PRINT_YELLOW    "\033[33m"
#define TT_PRINT_GREEN     "\033[32m"
#define TT_PRINT_BLUE      "\033[34m"
#define TT_PRINT_WHITE     "\033[37m"
#define TT_PRINT_GRAY      "\033[90m"
#define TT_PRINT_CYAN      "\033[36m"
#define TT_PRINT_PURPLE    "\033[35m"
#define TT_PRINT_NONE      "\033[0m"
#define CODE_INFO " [", get_file_name(__FILE__), ":", __LINE__, " (", __FUNCTION__, ")] "

#define LOG_TO_FILE(file_name, ...) \
    do { \
        std::ofstream out(file_name, std::ios::app); \
        print_base(out, __VA_ARGS__) << std::endl; \
    } while(0)

#define LOG(...) print_base(std::cout, __VA_ARGS__, TT_PRINT_NONE) << std::endl
#define LOG_FILE(out, ...) print_base(out, __VA_ARGS__) << std::endl


#define LOG_TEST(...) print_base(std::cout, TT_PRINT_PURPLE, get_curr_time(), __VA_ARGS__, TT_PRINT_NONE) << std::endl

#define LOG_LOCATE print_base(std::cout, CODE_INFO, TT_PRINT_NONE) << std::endl;

#endif // __LOG_H__