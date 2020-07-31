// 像素特征
// 把整张图像分成许多GridStructure
// 然后GridWindow对GridStructure再次细分
#pragma once

//STL
#include <list>
#include <unordered_set>
#include <utility>
#include <vector>

namespace StVO {

    ///将线段的所有像素坐标位置放入line_coords中
void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords);

///像素窗口
struct GridWindow {
    std::pair<int, int> width, height;
};

class GridStructure {
public:

    int rows, cols;
    ///实例化时resize容器大小
    GridStructure(int rows, int cols);

    ~GridStructure();
    ///获取grid容器中的std::list<int>
    std::list<int>& at(int x, int y);

    /// 获取x,y为中心，w为长宽的像素块，存入indices中，其中的元素为grid
    void get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const;

    void clear();

private:

    std::vector<std::vector<std::list<int>>> grid;///二维vector里面存储着list
    std::list<int> out_of_bounds;
};

} // namespace StVO
