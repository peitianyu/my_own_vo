#ifndef __TT_TICTOC_H__
#define __TT_TICTOC_H__

#include <chrono>
#include "tt_log.h"

class TicToc
{
public:
    using Clock = std::chrono::high_resolution_clock;

    TicToc() : start_(Clock::now()) {}
    void Tic() { start_ = Clock::now(); }
    double Toc() const { return std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - start_).count(); }
private:
    std::chrono::time_point<Clock> start_;
};

class TicTocAuto
{
public:
    TicTocAuto(const std::string& msg = "") : msg_(msg) { tictoc_.Tic(); }
    ~TicTocAuto() { 
        std::cout << msg_ << " cost: " << tictoc_.Toc() * 1000 << " ms" << std::endl;
        #ifdef OPEN_TO_FILE
        LOG_TO_FILE("../out/"+msg_+"_cost.txt", tictoc_.Toc() * 1000);
        #endif
    }
private:
    std::string msg_;
    TicToc tictoc_;
};

#endif // __TT_TICTOC_H__