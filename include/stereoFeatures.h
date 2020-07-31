// 表示特征的类
#pragma once
#include <config.h>

#include "auxiliar.h"
#include <math.h>

namespace StVO{

// 点特征
class PointFeature
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointFeature( Vector3d P_, Vector2d pl_obs_);
    PointFeature( Vector2d pl_, double disp_, Vector3d P_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_, int level );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_, int level, Matrix3d covP_an_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, Vector2d pl_obs_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, Vector2d pl_obs_,
                  int idx_, int level_, double sigma2_, Matrix3d covP_an_, bool inlier_ );
    ~PointFeature(){};

    PointFeature* safeCopy();///复制一个和它自己一样的PointFeature，然后返回指针

    int idx;///索引
    Vector2d pl, pl_obs;///像素坐标（本frame内的像素坐标，后一frame对应匹配到的像素坐标）
    double   disp;///双目视差
    Vector3d P;///3D位姿
    bool inlier;///是内点
    int level;///金字塔层数
    double sigma2 = 1.0;///高斯模糊使用的方差
    Matrix3d covP_an;

};

// 线段特征
class LineFeature
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LineFeature( Vector3d sP_, Vector3d eP_, Vector3d le_obs_);

    LineFeature( Vector3d sP_, Vector3d eP_, Vector3d le_obs_, Vector2d spl_obs_, Vector2d epl_obs_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  int idx_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  double angle_, int idx_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  double angle_, int idx_, int level);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_, Vector3d le_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_, Vector3d le_obs_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_, Vector2d spl_obs_, double sdisp_obs_,
                 Vector2d epl_, double edisp_, Vector3d eP_, Vector2d epl_obs_, double edisp_obs_,
                 Vector3d le_, Vector3d le_obs_, double angle_, int idx_, int level_, bool inlier_, double sigma2_,
                 Matrix3d covE_an_, Matrix3d covS_an_);

    LineFeature( Vector2d spl_, Vector2d spr_, double sdisp_, Vector3d sP_,
                Vector2d epl_, Vector2d epr_, double edisp_, Vector3d eP_,
                Vector3d le_,  double angle_, int idx_, int level_);

    ~LineFeature(){};

    LineFeature* safeCopy();///复制一个和它自己一样的LineFeature，然后返回指针

    int idx;///索引
    Vector2d spl, spr ,epl, epr, spl_obs, epl_obs;///像素位姿 TODO:添加了右边线段的像素坐标
    double   sdisp, edisp, angle, sdisp_obs, edisp_obs;///双目视差
    Vector3d sP,eP;///3D位姿
    Vector3d le, le_obs;///le：在本帧（当前帧的上一帧）线段的方向向量，le_obs：再当前帧内再次观测到线段的方向向量
    bool inlier;///是否是内线

    int level;///金字塔层数
    double sigma2 = 1.0;//////高斯模糊使用的方差

    Matrix3d covE_an, covS_an;

};

// TODO: l-lk部分
//线段类型
class Line
{
public:
    Vector2d spx;
    Vector2d epx;
    double length;
    int LastPointNum = 0;//上一帧的采样点数
    bool trackedGood;
    list<Point2f> SampPoints;

    Line(Vector2d spx_, Vector2d epx_);

    bool Sampling();///采样

    bool FitLine(int frame_count);///拟合线段

    bool FilterLine();///剔除线段

    bool RefineLine(Mat &image);

private:
    double A, B, C;///直线方程
    double getY(double x);
    double getX(double y);
};


}
