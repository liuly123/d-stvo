#pragma once

//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core.hpp>

#include "gridStructure.h"

namespace StVO {

typedef std::pair<int, int> point_2d;
typedef std::pair<point_2d, point_2d> line_2d;

inline double dot(const std::pair<double, double> &a, const std::pair<double, double> &b) {
    return (a.first*b.first + a.second*b.second);
}

// 归一化
inline void normalize(std::pair<double, double> &v) {
    double magnitude = std::sqrt(dot(v, v));///取模

    v.first /= magnitude;
    v.second /= magnitude;
}

// 匹配结果存放到matches_12（nnr是个阈值，左图匹配右图，不重复）
int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

// 和上面一样，支持左右互相匹配（调用matchNNR）（帧间匹配）
/// 支持点和线段的match
int match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

//计算两个矩阵的差别
int distance(const cv::Mat &a, const cv::Mat &b);

// 在grid中匹配特征点（双目匹配）
int matchGrid(const std::vector<point_2d> &points1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const GridWindow &w, std::vector<int> &matches_12);

// 在grid中匹配线段（双目匹配）
int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

} // namesapce StVO
