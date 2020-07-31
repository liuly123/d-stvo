// 计时类
#pragma once

//STL
#include <chrono>

namespace StVO {

class Timer {
public:

    static constexpr double SECONDS = 1e-9;///秒
    static constexpr double MILLISECONDS = 1e-6;///毫秒
    static constexpr double NANOSECONDS = 1.0;///纳秒

    Timer(double scale = MILLISECONDS);//默认的时间单位是毫秒
    virtual ~Timer();

    void start();

    double stop();

private:

    std::chrono::high_resolution_clock::time_point start_t;///开始计时的时间
    bool started;
    double scale;
};

} // namespace StVO
