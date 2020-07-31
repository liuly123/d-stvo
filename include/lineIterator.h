// 线段迭代器
/// 通过迭代getNext()获取线段的所有像素坐标
#pragma once

//STL
#include <utility>

namespace StVO {

class LineIterator {
public:
    //构造函数：调整线段的起点和终点，使朝向一致
    LineIterator(const double x1_, const double y1_, const double x2_, const double y2_);
    // 迭代输出线段覆盖过的grid
    bool getNext(std::pair<int, int> &pixel);

private:

    const bool steep;/// 是否|y2-y1|>|x2-x1|（斜坡的陡度，>45°为正）
    double x1,y1, x2, y2;

    double dx,///max(|x2-x1|,|y2-y1|)
           dy,///min(|x2-x1|,|y2-y1|)
           error;/// dx / 2

    int maxX, ///max(x1,x2.y1.y2)
        ystep;/// x2 > x1的情况下，y1-y2的正负
    int y,///y1
        x;/// y2
};

} // namespace StVO
