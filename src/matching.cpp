#include "matching.h"

//STL
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "config.h"
#include "gridStructure.h"

namespace StVO {

// 匹配结果存放到matches_12（nnr是个阈值，左图匹配右图，不重复）
int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    std::vector<std::vector<cv::DMatch>> matches_;///存放匹配结果
    cv::Ptr<cv::BFMatcher> bfm = cv::BFMatcher::create(cv::NORM_HAMMING, false); ///暴风(Brute Force)算法，是普通的模式匹配算法
    bfm->knnMatch(desc1, desc2, matches_, 2);///进行匹配

    if (desc1.rows != matches_.size())
        throw std::runtime_error("[matchNNR] Different size for matches and descriptors!");

    for (int idx = 0; idx < desc1.rows; ++idx) {
        if (matches_[idx][0].distance < matches_[idx][1].distance * nnr) {
            matches_12[idx] = matches_[idx][0].trainIdx;
            matches++;
        }
    }

    return matches;
}
// 和上面一样，支持左右互相匹配
int match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    if (Config::bestLRMatches()) {///是否互相检查两个图像之间的匹配
        int matches;
        std::vector<int> matches_21;
        if (Config::lrInParallel()) {///是否并行匹配特征
            auto match_12 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc1), std::cref(desc2), nnr, std::ref(matches_12));
            auto match_21 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc2), std::cref(desc1), nnr, std::ref(matches_21));
            matches = match_12.get();
            match_21.wait();
        } else {
            matches = matchNNR(desc1, desc2, nnr, matches_12);
            matchNNR(desc2, desc1, nnr, matches_21);
        }

        for (int i1 = 0; i1 < matches_12.size(); ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {///双向匹配的点对不上号
                i2 = -1;///就置为-1
                matches--;
            }
        }

        return matches;
    } else
        return matchNNR(desc1, desc2, nnr, matches_12);
}

//计算两个矩阵的差别
int distance(const cv::Mat &a, const cv::Mat &b) {

    // adapted from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0;
    for(int i = 0; i < 8; i++, pa++, pb++) {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

// 在grid中匹配特征点
int matchGrid(const std::vector<point_2d> &points1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const GridWindow &w, std::vector<int> &matches_12) {
/// 返回match成功的数量
    if (points1.size() != desc1.rows)
        throw std::runtime_error("[matchGrid] Each point needs a corresponding descriptor!");

    int matches = 0;
    matches_12.resize(desc1.rows, -1);///-1：长度不定

    int best_d, best_d2, best_idx;
    std::vector<int> matches_21, distances;

    if (Config::bestLRMatches()) {///交叉检查匹配
        matches_21.resize(desc2.rows, -1);
        distances.resize(desc2.rows, std::numeric_limits<int>::max());
    }

    for (int i1 = 0; i1 < points1.size(); ++i1) {

        best_d = std::numeric_limits<int>::max();
        best_d2 = std::numeric_limits<int>::max();
        best_idx = -1;

        const std::pair<int, int> &coords = points1[i1];///点位置
        cv::Mat desc = desc1.row(i1);///点的描述子

        std::unordered_set<int> candidates;
        grid.get(coords.first, coords.second, w, candidates);///获取GridWindow大小的图像像素的位置

        if (candidates.empty()) continue;
        for (const int &i2 : candidates) {
            if (i2 < 0 || i2 >= desc2.rows) continue;

            const int d = distance(desc, desc2.row(i2));///desc1和desc2进行匹配（计算Mat的差别，Mat是一维的）

            if (Config::bestLRMatches()) {///互相检查匹配
                if (d < distances[i2]) {
                    distances[i2] = d;///取distances的最小值
                    matches_21[i2] = i1;
                } else continue;
            }
            /// 找到最好的匹配
            if (d < best_d) {
                best_d2 = best_d;///第二小的distance
                best_d = d;///最小的distance
                best_idx = i2;///对应点的索引
            } else if (d < best_d2)
                best_d2 = d;
        }

        if (best_d < best_d2 * Config::minRatio12P()) {///第一和第二最佳匹配之间的最小比率
            matches_12[i1] = best_idx;
            matches++;
        }
    }

    if (Config::bestLRMatches()) {///互相检查匹配
        for (int i1 = 0; i1 < matches_12.size(); ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }

    return matches;
}

// 在grid中匹配线段
int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1,
              const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2,
              const GridWindow &w,
              std::vector<int> &matches_12) {

    if (lines1.size() != desc1.rows)
        throw std::runtime_error("[matchGrid] Each line needs a corresponding descriptor!");

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    int best_d, best_d2, best_idx;
    std::vector<int> matches_21, distances;

    if (Config::bestLRMatches()) {
        matches_21.resize(desc2.rows, -1);
        distances.resize(desc2.rows, std::numeric_limits<int>::max());
    }

    for (int i1 = 0; i1 < lines1.size(); ++i1) {

        best_d = std::numeric_limits<int>::max();
        best_d2 = std::numeric_limits<int>::max();
        best_idx = -1;

        const line_2d &coords = lines1[i1];
        cv::Mat desc = desc1.row(i1);

        const point_2d sp = coords.first;
        const point_2d ep = coords.second;

        std::pair<double, double> v = std::make_pair(ep.first - sp.first, ep.second - sp.second);
        normalize(v);

        std::unordered_set<int> candidates;
        grid.get(sp.first, sp.second, w, candidates);
        grid.get(ep.first, ep.second, w, candidates);

        if (candidates.empty()) continue;
        for (const int &i2 : candidates) {
            if (i2 < 0 || i2 >= desc2.rows) continue;

            if (std::abs(dot(v, directions2[i2])) < Config::lineSimTh())
                continue;

            const int d = distance(desc, desc2.row(i2));

            if (Config::bestLRMatches()) {
                if (d < distances[i2]) {
                    distances[i2] = d;
                    matches_21[i2] = i1;
                } else continue;
            }

            if (d < best_d) {
                best_d2 = best_d;
                best_d = d;
                best_idx = i2;
            } else if (d < best_d2)
                best_d2 = d;
        }

        if (best_d < best_d2 * Config::minRatio12P()) {
            matches_12[i1] = best_idx;
            matches++;
        }
    }

    if (Config::bestLRMatches()) {
        for (int i1 = 0; i1 < matches_12.size(); ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }

    return matches;
}

} //namesapce StVO
