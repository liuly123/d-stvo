// 加载数据集的类
#pragma once

//STL
#include <list>
#include <string>

//OpenCV
#include <opencv2/core.hpp>

#include "pinholeStereoCamera.h"

namespace StVO {

class Dataset {
public:

    // 构造函数
    Dataset(const std::string &dataset_path,///输如：数据集路径。用于生成图像路径的list
            const PinholeStereoCamera &cam,///输出：双目针孔相机参数
            int offset = 0, int nmax = 0, int step = 1);///数据集偏移（起点）、截取的长度、步长

    // 析构函数
    virtual ~Dataset() = default;

    // 读取数据集序列中的下一帧（左右图像），如果成功加载图像，则返回true
    bool nextFrame(cv::Mat &img_l, cv::Mat &img_r, string &fileName);

    // 如果序列中仍有可用的图像，则返回
    inline bool hasNext();

private:

    std::list<std::string> images_l, images_r;
    const PinholeStereoCamera &cam;
};

} // namespace StVO

