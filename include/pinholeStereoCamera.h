// 存储相机内参、实现2D-3D投影和重投影
#pragma once

using namespace std;

#include <opencv/cv.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <line_descriptor_custom.hpp>
using namespace cv;
using namespace line_descriptor;

#include <eigen3/Eigen/Core>
using namespace Eigen;

// 理想（水平）配置下双目相机的针孔模型
class PinholeStereoCamera
{

private:
    int                 width, height;
    double              fx, fy;///内参数
    double              cx, cy;
    double              b;///基线长度
    Matrix3d            K;///内参矩阵
    bool                dist;///是否校正畸变
    Matrix<double,5,1>  d;///畸变向量
    Mat                 Kl, Kr, Dl, Dr, Rl, Rr, Pl, Pr, R, t, Q;///下面的矩阵会用到
    Mat                 undistmap1l, undistmap2l, undistmap1r, undistmap2r;///dist==true时用于图像畸变矫正

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //实例化，从yaml读参数
    PinholeStereoCamera(const std::string &params_file);

    ~PinholeStereoCamera();

    // 图像畸变校正
    /// dist==true才校正
    void rectifyImage( const Mat& img_src, Mat& img_rec) const;///单张图
    void rectifyImagesLR( const Mat& img_src_l, Mat& img_rec_l, const Mat& img_src_r, Mat& img_rec_r ) const;///两张图

    // 投影和反向投影（根据K）
    Vector3d backProjection(const double &u, const double &v, const double &disp);///2D -> 3D
    Vector2d projection(const Vector3d &P);///3D -> 2D

    // Getters
    inline const int getWidth()             const { return width; };
    inline const int getHeight()            const { return height; };
    inline const Matrix3d&    getK()        const { return K; };
    inline const double       getB()        const { return b; };
    inline const Matrix<double,5,1> getD()  const { return d; };
    inline const double getFx()             const { return fx; };
    inline const double getFy()             const { return fy; };
    inline const double getCx()             const { return cx; };
    inline const double getCy()             const { return cy; };

};

