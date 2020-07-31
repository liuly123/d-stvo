// 一些辅助函数
#pragma once

#include <iostream>

#include <cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace line_descriptor;

#include <vector>
#include <math.h>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;

// 运动学函数
Matrix3d skew(Vector3d v);

Vector3d skewcoords(Matrix3d M);


Matrix4d inverse_se3(Matrix4d T);
Matrix4d expmap_se3(Vector6d x);
Vector6d logmap_se3(Matrix4d T);
Matrix6d adjoint_se3(Matrix4d T);
Matrix6d uncTinv_se3(Matrix4d T, Matrix6d covT );
Matrix6d unccomp_se3(Matrix4d T1, Matrix6d covT1, Matrix6d covTinc );


bool is_finite(const MatrixXd x);

// vector的辅助函数和结构
double vector_stdv_mad( vector<double> residues);
double vector_mean_mad(vector<double> v, double stdv, double K);
void vector_mean_stdv_mad( vector<double> residues, double &mean, double &stdv );

double robustWeightCauchy(double norm_res);

struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
    }
};

struct sort_flines_by_length
{
    inline bool operator()(const Vec4f& a, const Vec4f& b){
        return ( sqrt(pow(a(0)-a(2),2.0)+pow(a(1)-a(3),2.0)) > sqrt(pow(b(0)-b(2),2.0)+pow(b(1)-b(3),2.0)) );
    }
};

// TODO: l-lk部分
Vector2d FootDrop(Vector2d px, double A, double B, double C);
double GetGrad(Mat& src, Point2i point,Vector2d dir);
