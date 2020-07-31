#include <stereoFeatures.h>

namespace StVO{

// 点特征

PointFeature::PointFeature( Vector3d P_, Vector2d pl_obs_) :
    P(P_), pl_obs(pl_obs_), level(0)
{}

PointFeature::PointFeature( Vector2d pl_, double disp_, Vector3d P_ ) :
    pl(pl_), disp(disp_), P(P_), inlier(true), level(0)
{}

PointFeature::PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_ ) :
    pl(pl_), disp(disp_), P(P_), inlier(true), idx(idx_), level(0)
{}

PointFeature::PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_, int level_ ) :
    pl(pl_), disp(disp_), P(P_), inlier(true), idx(idx_), level(level_)
{
    for( int i = 0; i < level; i++ )
        sigma2 *= Config::orbScaleFactor();///FAST检测器的金字塔抽取比
    sigma2 = 1.f / (sigma2*sigma2);
}

PointFeature::PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_, int level_, Matrix3d covP_an_ ) :
    pl(pl_), disp(disp_), P(P_), inlier(true), idx(idx_), level(level_), covP_an(covP_an_)
{
    for( int i = 0; i < level; i++ )
        sigma2 *= Config::orbScaleFactor();
    sigma2 = 1.f / (sigma2*sigma2);
}

PointFeature::PointFeature( Vector2d pl_, double disp_, Vector3d P_, Vector2d pl_obs_ ) :
    pl(pl_), disp(disp_), P(P_), pl_obs(pl_obs_), inlier(true), level(0)
{}

PointFeature::PointFeature( Vector2d pl_, double disp_, Vector3d P_, Vector2d pl_obs_,
              int idx_, int level_, double sigma2_, Matrix3d covP_an_, bool inlier_ ) :
    pl(pl_), disp(disp_), P(P_), pl_obs(pl_obs_), inlier(inlier_), level(level_), sigma2(sigma2_), covP_an(covP_an_)
{}

PointFeature* PointFeature::safeCopy(){
    return new PointFeature( pl, disp, P, pl_obs, idx, level, sigma2, covP_an, inlier );
}



// 线段特征

LineFeature::LineFeature( Vector3d sP_, Vector3d eP_, Vector3d le_obs_) :
    sP(sP_), eP(eP_), le_obs(le_obs_), level(0)
{}


LineFeature::LineFeature( Vector3d sP_, Vector3d eP_, Vector3d le_obs_, Vector2d spl_obs_, Vector2d epl_obs_) :
    sP(sP_), eP(eP_), le_obs(le_obs_), spl_obs(spl_obs_), epl_obs(epl_obs_), level(0)
{}


LineFeature::LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                          Vector2d epl_, double edisp_, Vector3d eP_, Vector3d le_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), level(0)
{}

LineFeature::LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                          Vector2d epl_, double edisp_, Vector3d eP_,
                          Vector3d le_, Vector3d le_obs_ ) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), le_obs(le_obs_), inlier(true), level(0)
{}

LineFeature::LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                          Vector2d epl_, double edisp_, Vector3d eP_,
                          Vector3d le_,  int    idx_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), idx(idx_), level(0)
{}

LineFeature::LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                          Vector2d epl_, double edisp_, Vector3d eP_,
                          Vector3d le_,  double angle_, int    idx_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), idx(idx_), angle(angle_), level(0)
{}

LineFeature::LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                          Vector2d epl_, double edisp_, Vector3d eP_,
                          Vector3d le_,  double angle_, int idx_, int level_) :
    spl(spl_), sdisp(sdisp_), sP(sP_), epl(epl_), edisp(edisp_), eP(eP_), le(le_), inlier(true), idx(idx_), angle(angle_), level(level_)
{
    for( int i = 0; i < level; i++ )
        sigma2 *= Config::lsdScale();
    sigma2 = 1.f / (sigma2*sigma2);
}

LineFeature::LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_, Vector2d spl_obs_, double sdisp_obs_,
                          Vector2d epl_, double edisp_, Vector3d eP_, Vector2d epl_obs_, double edisp_obs_,
                          Vector3d le_, Vector3d le_obs_, double angle_, int idx_, int level_, bool inlier_, double sigma2_,
                          Matrix3d covE_an_, Matrix3d covS_an_) :

    spl(spl_), sdisp(sdisp_), sP(sP_), spl_obs(spl_obs_), sdisp_obs(sdisp_obs_),
    epl(epl_), edisp(edisp_), eP(eP_), epl_obs(epl_obs_), edisp_obs(edisp_obs_),
    le(le_), le_obs(le_obs_), angle(angle_), idx(idx_), level(level_), inlier(inlier_), sigma2(sigma2_), covE_an(covE_an_), covS_an(covS_an_)
{
    for( int i = 0; i < level; i++ )
        sigma2 *= Config::lsdScale();
    sigma2 = 1.f / (sigma2*sigma2);
}

LineFeature::LineFeature( Vector2d spl_, Vector2d spr_, double sdisp_, Vector3d sP_,
                          Vector2d epl_, Vector2d epr_, double edisp_, Vector3d eP_,
                          Vector3d le_,  double angle_, int idx_, int level_) :
    spl(spl_), spr(spr_), sdisp(sdisp_), sP(sP_), epl(epl_), epr(epr_), edisp(edisp_), eP(eP_), le(le_), inlier(true), idx(idx_), angle(angle_), level(level_)
{
    for( int i = 0; i < level; i++ )
        sigma2 *= Config::lsdScale();
    sigma2 = 1.f / (sigma2*sigma2);
}

LineFeature* LineFeature::safeCopy(){
    return new LineFeature( spl, sdisp, sP, spl_obs, sdisp_obs,
                            epl, edisp, eP, epl_obs, edisp_obs,
                            le, le_obs, angle, idx, level, inlier, sigma2, covE_an, covS_an );
}

// TODO: l-lk部分
Line::Line(Vector2d spx_, Vector2d epx_) : spx(spx_), epx(epx_), length((epx - spx).norm())
{
    Sampling();
}

bool Line::Sampling() {
    SampPoints.clear();
    Vector2d dif = epx - spx; /// 从起点到终点的差矢量
    length = dif.norm();
    double tan_dir = min(fabs(dif[0]), fabs(dif[1])) / max(fabs(dif[0]), fabs(dif[1]));///角度正切（取正值）
    double sin_dir = tan_dir / sqrt(1.0 + tan_dir * tan_dir);///角度正弦
    double correction = 2.0 * sqrt(1.0 + sin_dir * sin_dir);///校正
    size_t sampling_num = max(1.0, length / (2.0 * Config::patchSize() * correction));///采样点的数量
    if ( sampling_num <= Config::minSamlePoints() ) return false;
    else {
        // 采样
        double x_inc = dif[0] / sampling_num;
        double y_inc = dif[1] / sampling_num;
        for (size_t i = 0; i <= sampling_num; i++) {
            ///i=0时，SampPoints = spx
            ///i=sampling_num时，ampPoints = epx
            double samp_ix = spx[0] + i * x_inc;
            double samp_iy = spx[1] + i * y_inc;
            SampPoints.emplace_back(Point2f(samp_ix, samp_iy));
        }
        return true;
    }
}

bool Line::FitLine(int frame_count) {
    // 将点拟合成线段
    vector<Point2f> point_set(SampPoints.begin(), SampPoints.end());

    bool c1 = point_set.size() < Config::minSamlePoints();///线的跟踪点数太少
    bool c2 = (double)(LastPointNum - point_set.size()) / length > Config::maxPointNumDec() && (frame_count > 3);///线跟踪点数减少的太快

    if (c1 || c2)
        return false;///采样点数过少或下降过快
    // 拟合直线
    cv::Vec4f line_para;
    cv::fitLine(point_set,
                line_para,
                CV_DIST_HUBER,
                0,
                0.01,
                0.01);
    // 画出线段
    if (line_para[0] < 0.00000000001) {
        return false;
    } else {
        ///直线一般式
        B = 1;
        A = -line_para[1] / line_para[0];
        C = -A * line_para[2] - B * line_para[3];
        ///线段端点更新为垂足
        spx = Vector2d(SampPoints.front().x, SampPoints.front().y);
        epx = Vector2d(SampPoints.back().x, SampPoints.back().y);
        spx = FootDrop(spx, A, B, C);///更新线段端点
        epx = FootDrop(epx, A, B, C);
        length = (epx - spx).norm();///线段长
        ///if(point_set.size() / (epx-spx).norm() < 0.001) return false;
        return true;
    }
}

bool Line::FilterLine() {
    //根据线段拟合误差剔除不好的线段
    double drop_length_avg = 0;///垂线平均长度
    double drop_dist_diff = 0;///垂足距离的长度变化程度（反映均匀程度，越小越均匀）
    vector<double> drop_dists;///垂足距离

    Vector2d last_drop_point;///上一个垂足
    double last_drop_dist;///上一个垂足距离
    for (auto iter = SampPoints.begin(); iter != SampPoints.end(); iter++) {
        Vector2d point = Vector2d(iter->x, iter->y);///用于拟合直线的点
        Vector2d drop_point = (FootDrop(point, A, B, C));///垂足
        drop_length_avg += (point - drop_point).norm();///累加垂线长度
        if (iter == SampPoints.begin()) {
            last_drop_point = drop_point;
            continue;
        }
        drop_dists.emplace_back((drop_point - last_drop_point).norm());
    }
    for (auto drop_dist = drop_dists.begin(); drop_dist != drop_dists.end(); drop_dist++) {
        if (drop_dist == drop_dists.begin()) {
            last_drop_dist = *drop_dist;
        }
        drop_dist_diff += abs(*drop_dist - last_drop_dist);
    }

    drop_length_avg = drop_length_avg / SampPoints.size();///平均垂线长度
    drop_dist_diff = drop_dist_diff / (SampPoints.size() - 2.0);///垂足距离变化

    bool c1 = drop_length_avg > Config::maxDropLengthAvg();///平均垂线长度过大
    bool c2 = drop_dist_diff > Config::maxDropDistDiff();///垂足距离变化过大
    bool c3 = length / SampPoints.size() > Config::maxDistPerPoint();///平均垂足间距过大
    bool c4 = length < Config::minLineLength();///线段长度过小
    return !(c1 || c2 || c3 || c4);
}

bool Line::RefineLine(Mat &image) {
    //精确化端点
    vector<Point2i> spxs, epxs;///端点范围
    vector<double> spx_grad, epx_grad;///梯度
    vector<double> spx_grad_d, epx_grad_d;///梯度变化

    //端点范围
    bool x_longer = abs(spx[0] - epx[0]) > abs(spx[1] - epx[1]);
    if (x_longer) {
        if (spx[0] > epx[0]) {///保证spx.x < epx.x
            Vector2d tmp = spx;
            spx = epx;
            epx = tmp;
        }
        for (int i = -Config::growpx(); i <= Config::growpx(); i++) {
            spxs.emplace_back(spx[0] + i, getY(spx[0] + i));
            epxs.emplace_back(epx[0] + i, getY(epx[0] + i));
        }
    } else {
        if (spx[1] > epx[1]) {///保证spx.y < epx.y
            Vector2d tmp = spx;
            spx = epx;
            epx = tmp;
        }
        for (int i = -Config::growpx(); i <= Config::growpx(); i++) {
            spxs.emplace_back(getX(spx[1] + i), spx[1] + i);
            epxs.emplace_back(getX(epx[1] + i), epx[1] + i);
        }
    }

    //计算梯度
    for (auto iter = spxs.begin(); iter != spxs.end(); iter++) {
        spx_grad.emplace_back(GetGrad(image, *iter, Vector2d(A, B)));
    }
    for (auto iter = epxs.begin(); iter != epxs.end(); iter++) {
        epx_grad.emplace_back(GetGrad(image, *iter, Vector2d(A, B)));
    }
    //向量梯度变化
    ///去除了末尾
    for (auto iter = spx_grad.begin(); iter != spx_grad.end(); iter++) {
        if (iter == spx_grad.end()) continue;
        spx_grad_d.emplace_back(abs(*iter - *(iter + 1)));
    }
    for (auto iter = epx_grad.begin(); iter != epx_grad.end(); iter++) {
        if (iter == epx_grad.end()) continue;
        epx_grad_d.emplace_back(abs(*iter - *(iter + 1)));
    }
    //梯度变化最大的值
    vector<double>::iterator spx_iter = max_element(spx_grad_d.begin(), spx_grad_d.end());
    unsigned spx_grad_d_max_index = distance(begin(spx_grad_d), spx_iter);
    Point2i spx_grad_d_max = spxs[spx_grad_d_max_index];

    vector<double>::iterator epx_iter = max_element(epx_grad_d.begin(), epx_grad_d.end());
    unsigned epx_grad_d_max_index = distance(begin(epx_grad_d), epx_iter);
    Point2i epx_grad_d_max = epxs[epx_grad_d_max_index];
    //更新线段端点
    spx = FootDrop(Vector2d(spx_grad_d_max.x, spx_grad_d_max.y), A, B, C);
    epx = FootDrop(Vector2d(epx_grad_d_max.x, epx_grad_d_max.y), A, B, C);
}

// 代入直线方程
double Line::getY(double x) { return -(A * x + C) / B; }

double Line::getX(double y) { return -(B * y + C) / A; }

}
