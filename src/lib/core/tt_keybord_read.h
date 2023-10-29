#ifndef __KEYBORD_READ_H__
#define __KEYBORD_READ_H__

#include <iostream>
#include <sys/time.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <iomanip>
#include <vector>
#include <thread>

class KeybordRead
{
public:
    KeybordRead(const uint& ms = 10) 
        : ms_(ms), curr_index_(0) 
    { 
        std::cout << "-------------------keybord_read help-------------------" << std::endl;
        std::cout << "空格键: 暂停" << std::endl;
        std::cout << "回车键: 继续" << std::endl;
        std::cout << "w: 加速" << std::endl;
        std::cout << "s: 减速" << std::endl;
        std::cout << "a: 上一个数据" << std::endl;
        std::cout << "d: 下一个数据" << std::endl;
        std::cout << "x: 倒退运行" << std::endl;
        std::cout << "q: 退出" << std::endl;
        std::cout << "------------------------------------------" << std::endl;
        thread_ = std::thread(&KeybordRead::thread_func, this);
    }

    ~KeybordRead() { thread_.join();}

    template <typename T>
    T get_data(const std::vector<T>& datas){
        return (curr_index_ >= datas.size()) ? datas[datas.size() - 1] : datas[curr_index_];
    }

    uint index() const { return curr_index_; }
private:
    void thread_func()
    {
        while (true)
        {
            static char ch = ' ';  
            if (keybord_hit()){
                ch = getchar();
                if(ch == 'w') { ms_ += 5; ch ='\n'; std::cout << "------------------ms: " << ms_ << std::endl;}
                if(ch == 's') { if (ms_ > 5) ms_ -= 5; ch ='\n'; std::cout << "------------------ms: " << ms_ << std::endl;}
                if(ch == 'a') { if(curr_index_ > 0) curr_index_--; ch =' ';}
                if(ch == 'd') { curr_index_++; ch =' ';}
            }
            
            if (ch == 'q') break;
            else if (ch == '\n') curr_index_++;
            if(ch == 'x') { if(curr_index_ > 0) curr_index_--;}

            std::this_thread::sleep_for(std::chrono::milliseconds(ms_));
        }
    }

    bool keybord_hit()
    {
        struct termios oldt, newt;
        int ch;
        int oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if (ch != EOF)
        {
            ungetc(ch, stdin);
            return 1;
        }

        return 0;
    }
private:
    uint ms_;
    uint curr_index_;
    std::thread thread_;
};


#endif // __KEYBORD_READ_H__