#include "dataset.h"

//STL
#include <algorithm>
#include <functional>
#include <limits>
#include <list>
#include <stdexcept>
#include <string>

//Boost
#include <boost/regex.hpp> //Note: using boost regex instead of C++11 regex as it isn't supported by the compiler until gcc 4.9
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//YAML
#include <yaml-cpp/yaml.h>

#include "pinholeStereoCamera.h"

namespace StVO {

    // 外部函数（获取img目录中的文件排序列表）
void getSortedImages(const boost::filesystem::path &img_dir,///数据集的路径
                     std::function<bool(const std::string &)> filter,///排除一些图片：输入图片名称，输出是否排除（排除为true）
                     std::function<bool(const std::string &, const std::string &)> comparator,///用于排序的程序
                     std::vector<std::string> &img_paths){///文件路径输出到vector

    /// img_dir存在且是目录
    if (!boost::filesystem::exists(img_dir) ||
            !boost::filesystem::is_directory(img_dir))
        throw std::runtime_error("[Dataset] Invalid images subfolder");

    /// 获取img目录中的所有文件
    std::list<std::string> all_imgs;
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(img_dir), {})) {
        boost::filesystem::path filename_path = entry.path().filename();///是文件名，不是完整路径
        if (boost::filesystem::is_regular_file(entry.status()) &&
                (filename_path.extension() == ".png"  ||
                 filename_path.extension() == ".jpg"  ||
                 filename_path.extension() == ".jpeg" ||
                 filename_path.extension() == ".pnm"  ||
                 filename_path.extension() == ".tiff")) {
            all_imgs.push_back(filename_path.string());
        }
    }

    /// 排序
    img_paths.clear();
    img_paths.reserve(all_imgs.size());///增加vector的预留空间
    for (const std::string &filename : all_imgs)
        if (!filter(filename)) img_paths.push_back(filename);///用filter排除图片

    if (img_paths.empty())
        throw std::runtime_error("[Dataset] Invalid image names?");

    std::sort(img_paths.begin(), img_paths.end(), comparator);///用comparator函数排序

    for (std::string &filename : img_paths)
        filename = (img_dir / filename).string();///img_paths中的图片名变成绝对路径
}

// 实例化函数
Dataset::Dataset(const std::string &dataset_path, const PinholeStereoCamera &cam, int offset, int nmax, int step)
    : cam(cam) {

    ///判断数据集路径正常
    boost::filesystem::path dataset_base(dataset_path);
    if (!boost::filesystem::exists(dataset_base) ||
            !boost::filesystem::is_directory(dataset_base))
        throw std::runtime_error("[Dataset] Invalid directory");

    ///加载dataset_params.yaml文件
    boost::filesystem::path dataset_params = dataset_base / "dataset_params.yaml";
    if (!boost::filesystem::exists(dataset_params))
        throw std::runtime_error("[Dataset] Dataset parameters not found");
    YAML::Node dataset_config = YAML::LoadFile(dataset_params.string());

    ///设置图像路径
    boost::filesystem::path img_l_dir = dataset_base / dataset_config["images_subfolder_l"].as<std::string>();/// dataset_path/图像子目录
    boost::filesystem::path img_r_dir = dataset_base / dataset_config["images_subfolder_r"].as<std::string>();

    boost::regex expression("^[^0-9]*([0-9]+\\.?+[0-9]*)[^0-9]*\\.[a-z]{3,4}$");///正则表达式（“boost/regex.hpp”）
    boost::cmatch what;///存储匹配返回值

    /// std::function<bool(const std::string &)> filter; 排除函数
    auto filename_filter = [&expression, &what](const std::string &s) {
        return !boost::regex_match(s.c_str(), what, expression);///regex_match匹配算法，和正则表达式匹配就返回true
    };

    /// std::function<bool(const std::string &, const std::string &)> comparator; 排序函数
    auto sort_by_number = [&expression, &what](const std::string &a, const std::string &b) {
        double n1, n2;

        if (boost::regex_match(a.c_str(), what, expression))
            n1 = std::stod(what[1]);/// 获得图片名的纯数字名字
        else
            throw std::runtime_error("[Dataset] Unexpected behaviour while sorting filenames");

        if (boost::regex_match(b.c_str(), what, expression))
            n2 = std::stod(what[1]);
        else
            throw std::runtime_error("[Dataset] Unexpected behaviour while sorting filenames");

        return (n1 < n2);// 通过图片名的数字，从小到大排序
    };

    /// 加载路径并排序，路径存入vector
    std::vector<std::string> img_l_paths, img_r_paths;
    getSortedImages(img_l_dir, filename_filter, sort_by_number, img_l_paths);
    getSortedImages(img_r_dir, filename_filter, sort_by_number, img_r_paths);

    if (img_l_paths.size() != img_r_paths.size())/// 数量不相等就退出
        throw std::runtime_error("[Dataset] Left and right images");

    /// 截取数据集
    offset = std::max(0, offset);
    nmax = (nmax <= 0) ? std::numeric_limits<int>::max() : nmax;///std::numeric_limits<int>::max(); int型的最大值
    step = std::max(1, step);
    for (int i = 0, ctr = 0; (i + offset) < img_l_paths.size() && ctr < nmax; i += step, ctr++) {
        images_l.push_back(img_l_paths[i + offset]);///存入成员变量的vector
        images_r.push_back(img_r_paths[i + offset]);
    }
}

bool Dataset::nextFrame(cv::Mat &img_l, cv::Mat &img_r, string &fileName) {
    if (!hasNext()) return false;

    int pos = images_l.front().find_last_of('/');
    fileName = images_l.front().substr(pos + 1);//文件名（含扩展名）
    fileName = fileName.substr(0, fileName.rfind("."));//去掉扩展名

    img_l = cv::imread(images_l.front(), CV_LOAD_IMAGE_UNCHANGED);
    img_r = cv::imread(images_r.front(), CV_LOAD_IMAGE_UNCHANGED);
    cam.rectifyImagesLR(img_l, img_l, img_r, img_r);
    images_l.pop_front();
    images_r.pop_front();

    return (!img_l.empty() && !img_r.empty());
}

bool Dataset::hasNext() {
    return !(images_l.empty() || images_r.empty());
}

} // namespace StVO

