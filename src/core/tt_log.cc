#include "tt_log.h"

std::string get_file_name(const std::string &file_name) {
    size_t pos = file_name.find_last_of('/');
    if (pos == std::string::npos) return file_name; 
    return file_name.substr(pos + 1);
}
