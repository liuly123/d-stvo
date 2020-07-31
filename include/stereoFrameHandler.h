// StVO-PL的主程序
#pragma once
#include <stereoFrame.h>
#include <stereoFeatures.h>

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

namespace StVO{

class StereoFrameHandler
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    StereoFrameHandler( PinholeStereoCamera* cam_ );
    ~StereoFrameHandler();

    void initialize( const Mat img_l_, const Mat img_r_, const int idx_);// 初始化程序
    void insertStereoPair(const Mat img_l_, const Mat img_r_, const int idx_);// 以后每帧的step1
    void updateFrame();// 以后每帧的step3

    void f2fTracking();
    void matchF2FPoints();
    void matchF2FLines();

    bool isGoodSolution( Matrix4d DT, Matrix6d DTcov, double err );
    void optimizePose();// 以后每帧的step2

    // TODO: l-lk部分
    bool isGoodFrame();
    void insertStereoPairDirect(const Mat img_l_, const Mat img_r_, const int idx_);
    void f2fDirectTracking();
    void matchDirectF2FLines();


    // 自适应fast
    int orb_fast_th;//默认orb阈值
    double llength_th;

    //帧间匹配成功的点和线段
    list< PointFeature* > matched_pt;//这里用的是list，frame里是vector
    list< LineFeature*  > matched_ls;

    StereoFrame* prev_frame;///上一帧
    StereoFrame* curr_frame;///当前帧
    PinholeStereoCamera *cam;

    int  n_inliers, n_inliers_pt, n_inliers_ls;///内点和内线

private:


    void removeOutliers( Matrix4d DT );
    void gaussNewtonOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);
    void gaussNewtonOptimizationRobust(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);

    void levenbergMarquardtOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);///没有用到
    void optimizeFunctions(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e);
    void optimizeFunctionsRobust(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e);

};

}
