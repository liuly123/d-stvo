// Frane的一些操作
#pragma once

#include <future>
#include <thread>
#include <time.h>
#include <set>
#include <utility>
using namespace std;

#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
using namespace cv;
using namespace line_descriptor;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include <config.h>
#include <stereoFeatures.h>
#include <pinholeStereoCamera.h>
#include <auxiliar.h>

//grid的大小
/// 图像被划分为grid大小的区块，在grid内匹配以加快匹配速度
#define GRID_ROWS 48///行
#define GRID_COLS 64///列

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

namespace StVO{

class StereoFrame
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //实例化
    StereoFrame();
    StereoFrame(const Mat img_l_, const Mat img_r_, const int idx_, PinholeStereoCamera *cam_ );///idx_：帧索引
    ~StereoFrame();

    // 提取特征
    void extractStereoFeatures( double llength_th, int fast_th = 20 );

    // 特征点
    void detectStereoPoints(int fast_th = 20);
    void detectPointFeatures( Mat img, vector<KeyPoint> &points, Mat &pdesc, int fast_th = 20 );
    void matchStereoPoints(vector<KeyPoint> points_l, vector<KeyPoint> points_r, Mat &pdesc_l_, Mat pdesc_r, bool initial = false );

    // 线段
    void detectStereoLineSegments(double llength_th);
    void detectLineFeatures( Mat img, vector<KeyLine> &lines, Mat &ldesc, double min_line_length, bool &isGood );
    void matchStereoLines(vector<KeyLine> lines_l, vector<KeyLine> lines_r, Mat &ldesc_l_, Mat ldesc_r, bool initial = false );
    void filterLineSegmentDisparity(Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr , double &disp_s, double &disp_e);

    // 计算线段重叠的质量
    double lineSegmentOverlapStereo( double spl_obs, double epl_obs, double spl_proj, double epl_proj  );
    double lineSegmentOverlap( Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj  );

    Mat  plotStereoFrame();

    int frame_idx;
    Mat img_l, img_r;
    ///位姿：由外部程序计算
    Matrix4d Tfw;///世界坐标系下的位姿
    Matrix4d DT;

    Matrix6d Tfw_cov;
    Vector6d Tfw_cov_eig;
    double   entropy_first;

    Matrix6d DT_cov;
    Vector6d DT_cov_eig;
    double   err_norm;

    vector< PointFeature* > stereo_pt;      //特征点
    vector< LineFeature*  > stereo_ls;      //线段

    vector<KeyPoint> points_l, points_r;    //2d特征点(opencv内置类型)
    vector<KeyLine>  lines_l,  lines_r;     //2d线段(opencv内置类型)
    Mat pdesc_l, pdesc_r, ldesc_l, ldesc_r; //描述子

    PinholeStereoCamera *cam;

    double inv_width, inv_height; // grid cell

    // TODO: l-lk部分
    int tracked_frame_count = 0;
    bool isGoodKeyFrame = false;//关键帧是否适合跟踪
    bool trackingGood = false;//跟踪效果好
    double left_length_all;//线段总长
    double right_length_all;
    list<Line> direct_lines_l, direct_lines_r;
    void directTracking( StereoFrame* prev_frame, int fast_th = 20 );//直接法跟踪线段+提取特征点
    void trackingStereoLineSegments( StereoFrame* prev_frame );      //直接跟踪双目线段
    void trackingLineFeatures( StereoFrame* prev_frame, Mat img, list<Line> &lines, bool &trackGood ,bool isLeft);//跟踪线段的实际操作
    void matchDirectStereoLines( vector<LineFeature*> &prev_stereo_ls, list<Line> &direct_lines_left, list<Line> &direct_lines_right );//跟踪之后删除跟踪失败的线段，并计算3D位姿
};

}
