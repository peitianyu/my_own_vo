#ifndef __READ_FILE_H__
#define __READ_FILE_H__

#include <iostream>
#include <sstream>
#include <fstream>


class ReadFile
{
public:
    ReadFile(const std::string& file_name)
    {
        ifs_.open(file_name);
        if (!ifs_.is_open()) {
            std::cout << "open file failed: " << file_name << std::endl;
        }
    }

    ~ReadFile()
    {
        ifs_.close();
    }

    bool eof() const { return ifs_.eof(); }

    std::string get_line()
    {
        std::string line;
        std::getline(ifs_, line);
        return line;
    }

    std::stringstream get_stringstream()
    {
        std::string line;
        std::getline(ifs_, line);
        return std::stringstream(line);
    }
private:
    std::ifstream ifs_;
};








#endif // __READ_FILE_H__