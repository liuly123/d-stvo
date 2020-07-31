#include "lineIterator.h"

//STL
#include <cmath>
#include <utility>

///Implementation of Bresenham's line drawing Algorithm
///Adapted from: https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B

namespace StVO {

    //构造函数：调整线段的起点和终点，使朝向一致
LineIterator::LineIterator(const double x1_, const double y1_, const double x2_, const double y2_)
    : x1(x1_), y1(y1_), x2(x2_), y2(y2_), steep(std::abs(y2_ - y1_) > std::abs(x2_ - x1_)) {

    ///保证存储的值中：|x2-x1|>|y2-y1|，且x2 > x1，x1与y1、x2与y2要配对
    /// 就是为了调整线段的start和end，使朝向更一致
    if (steep) {///保证|x2-x1|>|y2-y1|
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2) {///保证x2 > x1
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    dx = x2 - x1;
    dy = std::abs(y2 - y1);

    error = dx / 2.0;
    ystep = (y1 < y2) ? 1 : -1;

    x = static_cast<int>(x1);///强制类型转换
    y = static_cast<int>(y1);

    maxX = static_cast<int>(x2);
}

// 迭代输出线段覆盖过的grid
bool LineIterator::getNext(std::pair<int, int> &pixel) {

    if (x > maxX)
        return false;
    /// 保证前提：|x2-x1|>|y2-y1|，且x2 > x1，x1与y1、x2与y2要配对
    if (steep)
        pixel = std::make_pair(y, x);///y绝对值大，输出max(x1,x2)在前
    else
        pixel = std::make_pair(x, y);///x绝对值大，输出max(x1,x2)在前

    error -= dy;
    if (error < 0) {
        y += ystep;
        error += dx;
    }

    x++;
    return true;
}

} // namespace StVO
