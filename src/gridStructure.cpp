#include "gridStructure.h"

//STL
#include <algorithm>
#include <stdexcept>

#include "lineIterator.h"

namespace StVO {

void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords) {
    line_coords.clear();

    /// 自定义的线段迭代器（按差的绝对值大小重新组合一下）
    LineIterator it(x1, y1, x2, y2);

    std::pair<int, int> p;
    while (it.getNext(p))
        line_coords.push_back(p);
}

///构造函数resize容器grid的行和列
GridStructure::GridStructure(int rows, int cols)
    : rows(rows), cols(cols) {

    if (rows <= 0 || cols <= 0)
        throw std::runtime_error("[GridStructure] invalid dimension");

    grid.resize(cols, std::vector<std::list<int>>(rows));
}

GridStructure::~GridStructure() {

}

std::list<int>& GridStructure::at(int x, int y) {

    if (x >= 0 && x < cols &&
            y >= 0 && y < rows)
        return grid[x][y];
    else
        return out_of_bounds;
}

void GridStructure::get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const {///set是有和没有的问题

    int min_x = std::max(0, x - w.width.first);///像素方块的x起点
    int max_x = std::min(cols, x + w.width.second + 1);///像素方块的x终点

    int min_y = std::max(0, y - w.height.first);///像素方块的y起点
    int max_y = std::min(rows, y + w.height.second + 1);///像素方块的y终点

    for (int x_ = min_x; x_ < max_x; ++x_)
        for (int y_ = min_y; y_ < max_y; ++y_)
            indices.insert(grid[x_][y_].begin(), grid[x_][y_].end());///把x_,y_位置处grid的list都提取出来
}

void GridStructure::clear() {

    for (int x = 0; x < cols; ++x)
        for (int y = 0; y < rows; ++y)
            grid[x][y].clear();
}

} //namesapce StVO
