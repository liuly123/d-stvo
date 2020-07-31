#include <stereoFrameHandler.h>

#include "matching.h"

namespace StVO{

///构造函数和析构函数很简单
StereoFrameHandler::StereoFrameHandler( PinholeStereoCamera *cam_ ) : cam(cam_) {}
StereoFrameHandler::~StereoFrameHandler(){}

// 初始化
void StereoFrameHandler::initialize(const Mat img_l_, const Mat img_r_ , const int idx_)///idx_=0;
{
    /// 阈值
    orb_fast_th = Config::orbFastTh();///FAST阈值
    llength_th  = Config::minLineLength() * std::min( cam->getWidth(), cam->getHeight() ) ;///最小线段长度
    /// 定义双目Frame
    prev_frame = new StereoFrame( img_l_, img_r_, idx_, cam );///new出来的是class的指针
    /// 提取描述子
    prev_frame->extractStereoFeatures( llength_th, orb_fast_th );///因为prev_frame是一个class的指针，所以用->调用成员
    prev_frame->Tfw     = Matrix4d::Identity();
    prev_frame->Tfw_cov = Matrix6d::Identity();
    prev_frame->DT      = Matrix4d::Identity();
    curr_frame = prev_frame;//似乎没什么用，删掉也行
}

// 普通的frame的处理step1
void StereoFrameHandler::insertStereoPair(const Mat img_l_, const Mat img_r_ , const int idx_)
{
    curr_frame =  new StereoFrame( img_l_, img_r_, idx_, cam );
    curr_frame->extractStereoFeatures( llength_th, orb_fast_th );
    f2fTracking();
}

// 普通的frame的处理step3
void StereoFrameHandler::updateFrame()
{

    /// 更新关键点提取的FAST阈值
    if( Config::adaptativeFAST() )/// true
    {
        int min_fast  = Config::fastMinTh();
        int max_fast  = Config::fastMaxTh();
        int fast_inc  = Config::fastIncTh();
        int feat_th   = Config::fastFeatTh();
        float err_th  = Config::fastErrTh();

        /// 如果优化不佳, -= 2*fast_inc
        if( curr_frame->DT == Matrix4d::Identity() || curr_frame->err_norm > err_th )
            orb_fast_th = std::max( min_fast, orb_fast_th - 2*fast_inc );
        /// elif 内点数量 ...
        else if( n_inliers_pt < feat_th )///数量少于阈值
            orb_fast_th = std::max( min_fast, orb_fast_th - 2*fast_inc );
        else if( n_inliers_pt < feat_th * 2 )///阈值 < 数量 <阈值*2
            orb_fast_th = std::max( min_fast, orb_fast_th - fast_inc );
        else if( n_inliers_pt > feat_th * 3 )/// 阈值*3 <数量
            orb_fast_th = std::min( max_fast, orb_fast_th + fast_inc );
        else if( n_inliers_pt > feat_th * 4 )/// 阈值*4 < 数量
            orb_fast_th = std::min( max_fast, orb_fast_th + 2*fast_inc );
    }

    /// 清理和更新变量
    for( auto pt: matched_pt )
        delete pt;
    for( auto ls: matched_ls )
        delete ls;
    matched_pt.clear();
    matched_ls.clear();

    ///prev_frame.reset();
    ///prev_frame.reset( curr_frame );
    delete prev_frame;
    prev_frame = curr_frame;
    curr_frame = NULL;

}

//轨迹跟踪方法
/// insertStereoPair调用
void StereoFrameHandler::f2fTracking()
{

    /// 特征匹配
    matched_pt.clear();
    matched_ls.clear();

    if( Config::plInParallel() && Config::hasPoints() && Config::hasLines() )/// 并行match
    {
        auto detect_p = async(launch::async, &StereoFrameHandler::matchF2FPoints, this );
        auto detect_l = async(launch::async, &StereoFrameHandler::matchF2FLines,  this );
        detect_p.wait();
        detect_l.wait();
    }
    else///不并行match
    {
        if (Config::hasPoints()) matchF2FPoints();
        if (Config::hasLines()) matchF2FLines();
    }

    n_inliers_pt = matched_pt.size();
    n_inliers_ls = matched_ls.size();
    n_inliers    = n_inliers_pt + n_inliers_ls;///inlier=内点数+线数
}
// 匹配特征点
/// f2fTracking调用
void StereoFrameHandler::matchF2FPoints()
{

    /// points f2f tracking
    /// --------------------------------------------------------------------------------------------------------------------
    matched_pt.clear();
    if ( !Config::hasPoints() || curr_frame->stereo_pt.empty() || prev_frame->stereo_pt.empty() )
        return;

    std::vector<int> matches_12;
    match(prev_frame->pdesc_l, curr_frame->pdesc_l, Config::minRatio12P(), matches_12);///matching.h进行f2f matching（更新matches_12）

    /// bucle around pmatches
    for (int i1 = 0; i1 < matches_12.size(); ++i1) {
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        prev_frame->stereo_pt[i1]->pl_obs = curr_frame->stereo_pt[i2]->pl;///prev_frame的特征点在curr_frame再次观测到，把curr_frame的对应点像素坐标存入prev_frame的pl_obs
        prev_frame->stereo_pt[i1]->inlier = true;///这个点也就是内点了
        matched_pt.push_back( prev_frame->stereo_pt[i1]->safeCopy() );
        curr_frame->stereo_pt[i2]->idx = prev_frame->stereo_pt[i1]->idx; /// 当前frame点的索引=上一frame点的索引
    }
}

// 匹配线段
/// f2fTracking调用
void StereoFrameHandler::matchF2FLines()
{

    /// line segments f2f tracking
    matched_ls.clear();
    if( !Config::hasLines() || curr_frame->stereo_ls.empty() || prev_frame->stereo_ls.empty() )
        return;

    std::vector<int> matches_12;
    match(prev_frame->ldesc_l, curr_frame->ldesc_l, Config::minRatio12L(), matches_12);

    /// bucle around pmatches
    for (int i1 = 0; i1 < matches_12.size(); ++i1) {
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        prev_frame->stereo_ls[i1]->sdisp_obs = curr_frame->stereo_ls[i2]->sdisp;///双目视差
        prev_frame->stereo_ls[i1]->edisp_obs = curr_frame->stereo_ls[i2]->edisp;
        prev_frame->stereo_ls[i1]->spl_obs   = curr_frame->stereo_ls[i2]->spl;///匹配到的点
        prev_frame->stereo_ls[i1]->epl_obs   = curr_frame->stereo_ls[i2]->epl;
        prev_frame->stereo_ls[i1]->le_obs    = curr_frame->stereo_ls[i2]->le;
        prev_frame->stereo_ls[i1]->inlier    = true;
        matched_ls.push_back( prev_frame->stereo_ls[i1]->safeCopy() );
        curr_frame->stereo_ls[i2]->idx = prev_frame->stereo_ls[i1]->idx; /// 当前frame线段的索引=上一frame线段的索引
    }
}

// 优化程序
// --------------------------------------------------------------------------------------------------------------------------------
/// 优化的时候才用到李代数表示位姿

// 判断解的好坏
bool StereoFrameHandler::isGoodSolution( Matrix4d DT, Matrix6d DTcov, double err )
{
    SelfAdjointEigenSolver<Matrix6d> eigensolver(DTcov);
    Vector6d DT_cov_eig = eigensolver.eigenvalues();

    if( DT_cov_eig(0)<0.0 || DT_cov_eig(5)>1.0 || err < 0.0 || err > 1.0 || !is_finite(DT) )
    {
        cout << endl << DT_cov_eig(0) << "\t" << DT_cov_eig(5) << "\t" << err << endl;
        return false;
    }

    return true;
}

// 普通frame的处理step2（优化位姿）
void StereoFrameHandler::optimizePose()
{

    Matrix4d DT, DT_;///4x4的SE3矩阵表示位姿
    Matrix6d DT_cov;
    double   err = numeric_limits<double>::max(), e_prev;
    err = -1.0;

    /// 设置求解的初始位姿（取决于是否使用先前的信息，以及先前解的优劣）
    if( Config::useMotionModel() )///默认为false
    {///用上一帧的位姿作为迭代起点
        DT     = prev_frame->DT;
        DT_cov = prev_frame->DT_cov;
        e_prev = prev_frame->err_norm;
        if( !isGoodSolution(DT,DT_cov,e_prev) )
            DT = Matrix4d::Identity();
    }
    else
        DT = Matrix4d::Identity();

    /// 优化模型
    int mode = 0;   /// GN - GNR - LM

    /// 求解器
    if( n_inliers >= Config::minFeatures() )
    {
        /// 优化
        DT_ = DT;
        if( mode == 0 )      gaussNewtonOptimization(DT_,DT_cov,err,Config::maxIters());/// 高斯牛顿法
        else if( mode == 1 ) gaussNewtonOptimizationRobust(DT_,DT_cov,err,Config::maxIters());
        else if( mode == 2 ) levenbergMarquardtOptimization(DT_,DT_cov,err,Config::maxIters());

        /// 去除异常值（基于协方差特征值和优化误差实施一些逻辑）
        if( isGoodSolution(DT_,DT_cov,err) )
        {
            removeOutliers(DT_);///移除异常值
            /// 基于内点再次优化
            if( n_inliers >= Config::minFeatures() )
            {
                if( mode == 0 )      gaussNewtonOptimization(DT,DT_cov,err,Config::maxItersRef());
                else if( mode == 1 ) gaussNewtonOptimizationRobust(DT,DT_cov,err,Config::maxItersRef());
                else if( mode == 2 ) levenbergMarquardtOptimization(DT,DT_cov,err,Config::maxItersRef());
            }
            else///内点不够
            {
                DT     = Matrix4d::Identity();
                cout << "[StVO] not enough inliers (after removal)" << endl;
            }
        }
        else///第一次优化效果就不好（优化结果不收敛），采用GNR优化
        {
            gaussNewtonOptimizationRobust(DT,DT_cov,err,Config::maxItersRef());
            ///DT     = Matrix4d::Identity();
            ///cout << "[StVO] optimization didn't converge" << endl;
        }
    }
    else///第一次优化就没有足够的内点
    {
        DT     = Matrix4d::Identity();
        cout << "[StVO] not enough inliers (before optimization)" << endl;
    }


    /// 更新估计后的位姿
    if( isGoodSolution(DT,DT_cov,err) && DT != Matrix4d::Identity() )
    {
        curr_frame->DT       = expmap_se3(logmap_se3( inverse_se3( DT ) ));
        curr_frame->DT_cov   = DT_cov;
        curr_frame->err_norm = err;
        curr_frame->Tfw      = expmap_se3(logmap_se3( prev_frame->Tfw * curr_frame->DT ));
        curr_frame->Tfw_cov  = unccomp_se3( prev_frame->Tfw, prev_frame->Tfw_cov, DT_cov );
        SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
        curr_frame->DT_cov_eig = eigensolver.eigenvalues();
    }
    else
    {
        ///setAsOutliers();
        curr_frame->DT       = Matrix4d::Identity();
        curr_frame->DT_cov   = Matrix6d::Zero();
        curr_frame->err_norm = -1.0;
        curr_frame->Tfw      = prev_frame->Tfw;
        curr_frame->Tfw_cov  = prev_frame->Tfw_cov;
        curr_frame->DT_cov_eig = Vector6d::Zero();
    }
}

// TODO: l-lk部分
bool StereoFrameHandler::isGoodFrame()
{//用于判断是否适合进行直接跟踪
    return (prev_frame->isGoodKeyFrame || prev_frame->trackingGood);
}

void StereoFrameHandler::insertStereoPairDirect(const Mat img_l_, const Mat img_r_, const int idx_)
{
    curr_frame =  new StereoFrame( img_l_, img_r_, idx_, cam );
    curr_frame->tracked_frame_count = prev_frame->tracked_frame_count + 1;//跟踪的帧数
    curr_frame->directTracking(prev_frame, orb_fast_th);

    // for (auto line = curr_frame->direct_lines_l.begin(); line != curr_frame->direct_lines_l.end(); line++)///遍历线段
    // {
    //     cout << "spr=\n" << line->spx << endl;
    //     cout << "epr=\n" << line->epx << endl;
    // }

    // Mat image_show;
    // cvtColor(img_r_,image_show,CV_GRAY2BGR);
    // for (auto line = curr_frame->direct_lines_r.begin(); line != curr_frame->direct_lines_r.end(); line++)///遍历线段
    // {
    //     cv::line(image_show,
    //                 Point(line->spx[0],line->spx[1]),
    //                 Point(line->epx[0],line->epx[1]),
    //                 Scalar(0,0,200),1,CV_AA);///RGB颜色、粗细、抗锯齿
    // }
    // imshow("DirectTracking", image_show);
    // cv::waitKey(1000);
    // cv::destroyWindow("DirectTracking");

    f2fDirectTracking();//跟踪完了得到本帧特征的3D位姿，然后执行帧间匹配
}

void StereoFrameHandler::f2fDirectTracking()
{
    /// 特征匹配
    matched_pt.clear();
    matched_ls.clear();

    if( Config::plInParallel() && Config::hasPoints() && Config::hasLines() )/// 并行match
    {
        auto detect_p = async(launch::async, &StereoFrameHandler::matchF2FPoints, this );
        auto detect_l = async(launch::async, &StereoFrameHandler::matchDirectF2FLines,  this );
        detect_p.wait();
        detect_l.wait();
    }
    else///不并行match
    {
        if (Config::hasPoints()) matchF2FPoints();
        if (Config::hasLines()) matchDirectF2FLines();
    }

    n_inliers_pt = matched_pt.size();
    n_inliers_ls = matched_ls.size();
    n_inliers    = n_inliers_pt + n_inliers_ls;///inlier=内点数+线数
    // cout << "n_inliers_pt=" << n_inliers_pt << endl
    //      << "n_inliers_ls=" << n_inliers_ls << endl;
    
}

void StereoFrameHandler::matchDirectF2FLines()
{//directTracking之后的matchDirectStereoLines已经删除了跟踪失败的线段对应的上一帧线段
    matched_ls.clear();
    if( !Config::hasLines() || curr_frame->stereo_ls.empty() || prev_frame->stereo_ls.empty() )
        return;

    std::vector<StVO::LineFeature *> &prev_lines = prev_frame->stereo_ls;
    std::vector<StVO::LineFeature *> &curr_lines = curr_frame->stereo_ls;
    auto prev_line = prev_lines.begin();
    auto curr_line = curr_lines.begin();
    //cout << "curr_lines.size()=" << curr_lines.size() << endl;
    for(; prev_line != prev_lines.end(); ++prev_line,++curr_line)
    {
        (*prev_line)->sdisp_obs = (*curr_line)->sdisp;///双目视差
        (*prev_line)->edisp_obs = (*curr_line)->edisp;
        (*prev_line)->spl_obs   = (*curr_line)->spl;///匹配到的点
        (*prev_line)->epl_obs   = (*curr_line)->epl;
        (*prev_line)->le_obs    = (*curr_line)->le;
        (*prev_line)->inlier    = true;
        matched_ls.push_back( (*prev_line)->safeCopy() );
        (*curr_line)->idx = (*prev_line)->idx; /// 当前frame线段的索引=上一frame线段的索引
    }
    //cout << "matched_ls.size()=" << matched_ls.size() << endl;

}

// G-N优化器
void StereoFrameHandler::gaussNewtonOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters)
{
    Matrix6d H;///hessian的se3表示
    Vector6d g, DT_inc;///，解(位姿)增量
    double err, err_prev = 999999999.9;///每次迭代的误差和上一误差

    int iters;
    // 迭代优化
    for( iters = 0; iters < max_iters; iters++)
    {
        /// 估计hessian和梯度（可选）
        optimizeFunctions( DT, H, g, err );///计算Optimization需要的参数：误差梯度、误差雅可比、hessian矩阵
        if (err > err_prev) {///优化发散，退出
            if (iters > 0)
                break;
            err_ = -1.0;
            return;
        }
        /// 如果误差很小（或误差变化很小）就停止
        if( ( ( err < Config::minError()) || abs(err-err_prev) < Config::minErrorChange() ) ) {
            cout << "[StVO] Small optimization improvement" << endl;
            break;
        }
        /// 更新阶段
        ColPivHouseholderQR<Matrix6d> solver(H);///QR分解
        DT_inc = solver.solve(g);///求解H·Δx=g中的Δx（这里Δx是用李代数(向量)表示的迭代位姿增量）
        DT  << DT * inverse_se3( expmap_se3(DT_inc) );///DT_inc转换成se3(4x4矩阵)表示，然后更新求得的位姿
        /// 如果解的变化很小，就停止
        if( DT_inc.head(3).norm() < Config::minErrorChange() && DT_inc.tail(3).norm() < Config::minErrorChange()) {
            cout << "[StVO] Small optimization solution variance" << endl;
            break;
        }
        /// 更新先前的值
        err_prev = err;
    }

    DT_cov = H.inverse();  ///DT_cov = Matrix6d::Identity();
    err_   = err;
}

// G-N-R优化器（类似）
void StereoFrameHandler::gaussNewtonOptimizationRobust(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters)
{

    Matrix4d DT_;
    Matrix6d H;
    Vector6d g, DT_inc;
    double err, err_prev = 999999999.9;
    bool solution_is_good = true;
    DT_ = DT;

    /// 初始解
    int iters;
    for( iters = 0; iters < max_iters; iters++)///迭代优化
    {
        /// 估计hessian和梯度（可选）
        optimizeFunctionsRobust( DT, H, g, err );
        /// 如果误差很小就停止
        if( ( fabs(err-err_prev) < Config::minErrorChange() ) || ( err < Config::minError()) )/// || err > err_prev )
            break;
        /// 更新阶段
        ColPivHouseholderQR<Matrix6d> solver(H);
        DT_inc = solver.solve(g);
        if( solver.logAbsDeterminant() < 0.0 || solver.info() != Success )
        {
            solution_is_good = false;
            break;
        }
        DT  << DT * inverse_se3( expmap_se3(DT_inc) );
        /// 如果参数更改很小，则停止 (change with two parameters, one for R and another one for t)
        if( DT_inc.norm() < Config::minErrorChange() )
            break;
        /// 更新以前的值
        err_prev = err;
    }

    if( solution_is_good )
    {
        DT_cov = H.inverse();  ///DT_cov = Matrix6d::Identity();
        err_   = err;
    }
    else
    {
        DT = DT_;
        err_ = -1.0;
        DT_cov = Matrix6d::Identity();
    }

}

// L-M优化器（类似）
void StereoFrameHandler::levenbergMarquardtOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters)
{
    Matrix6d H;
    Vector6d g, DT_inc;
    double err, err_prev = 999999999.9;

    double lambda   = 0.000000001;
    double lambda_k = 4.0;

    /// form the first iteration
///    optimizeFunctionsRobust( DT, H, g, err );
    optimizeFunctions( DT, H, g, err );

    /// initial guess of lambda
    double Hmax = 0.0;
    for( int i = 0; i < 6; i++)
    {
        if( H(i,i) > Hmax || H(i,i) < -Hmax )
            Hmax = fabs( H(i,i) );
    }
    lambda *= Hmax;

    /// solve the first iteration
    for(int i = 0; i < 6; i++)
        H(i,i) += lambda;/// * H(i,i) ;
    ColPivHouseholderQR<Matrix6d> solver(H);
    DT_inc = solver.solve(g);
    DT  << DT * inverse_se3( expmap_se3(DT_inc) );
    err_prev = err;

    /// start Levenberg-Marquardt minimization
    ///plotStereoFrameProjerr(DT,0);
    for( int iters = 1; iters < max_iters; iters++)
    {
        /// estimate hessian and gradient (select)
///        optimizeFunctionsRobust( DT, H, g, err );
        optimizeFunctions( DT, H, g, err );
        /// if the difference is very small stop
        if( ( fabs(err-err_prev) < Config::minErrorChange() ) || ( err < Config::minError()) )
            break;
        /// add lambda to hessian
        for(int i = 0; i < 6; i++)
            H(i,i) += lambda;/// * H(i,i) ;
        /// update step
        ColPivHouseholderQR<Matrix6d> solver(H);
        DT_inc = solver.solve(g);
        /// update lambda
        if( err > err_prev )
            lambda /= lambda_k;
        else
        {
            lambda *= lambda_k;
            DT  << DT * inverse_se3( expmap_se3(DT_inc) );
        }
        /// plot each iteration
        ///plotStereoFrameProjerr(DT,iters+1);
        /// if the parameter change is small stop
        if( DT_inc.head(3).norm() < Config::minErrorChange() && DT_inc.tail(3).norm() < Config::minErrorChange())
            break;
        /// update previous values
        err_prev = err;
    }

    DT_cov = H.inverse();  ///DT_cov = Matrix6d::Identity();
    err_   = err;
}

// 计算优化器的参数
/// DT：当前的解x
/// H：hessian矩阵，H(x) = J(x)^T · J(x)
/// g：梯度方向，g = -J(x)^T · f(x)
/// e：残差
void StereoFrameHandler::optimizeFunctions(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e )
{

    /// 定义hessian矩阵H、梯度g、残差e
    Matrix6d H_l, H_p;
    Vector6d g_l, g_p;
    double   e_l = 0.0, e_p = 0.0, S_l, S_p;
    H   = Matrix6d::Zero(); H_l = H; H_p = H;
    g   = Vector6d::Zero(); g_l = g; g_p = g;
    e   = 0.0;

    /// 点特征
    int N_p = 0;

    for( auto it = matched_pt.begin(); it!=matched_pt.end(); it++)///遍历点特征
    {
        if( (*it)->inlier )///是内点
        {
            /// 投影函数： 3D（上一帧） -DT-> 3D（当前帧） -> 2D（当前帧）
            Vector3d P_ = DT.block(0,0,3,3) * (*it)->P + DT.col(3).head(3);///相机位姿为DT时，landmark的位置（路标在新位姿DT上的投影）。a'=Ra+t。
            Vector2d pl_proj = cam->projection( P_ );///再投影到像素坐标系，3D -> 2D。（P是上一帧点的3D位姿，P_是它投影到当前帧的3D位姿）
            /// 计算投影误差
            Vector2d err_i    = pl_proj - (*it)->pl_obs;///投影误差 = （当前位姿DT下）投影点的像素坐标 - 实际观测到的特征点像素坐标
            double err_i_norm = err_i.norm();///误差的范数 =  √ [ (Δx)^2 + (Δy)^2 ]
            /// 估计变量J，H和g
            double gx   = P_(0);///投影后的X
            double gy   = P_(1);///投影后的Y
            double gz   = P_(2);///投影后的Z
            double gz2  = gz*gz;/// Z^2
            double fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);/// fx / Z^2    homogTh：（很小的数值）用于避开无穷远处的点
            double dx   = err_i(0);/// 投影误差Δx
            double dy   = err_i(1);/// 投影误差Δy
            /// jacobian
            Vector6d J_aux;
            J_aux << + fgz2 * dx * gz,
                     + fgz2 * dy * gz,
                     - fgz2 * ( gx*dx + gy*dy ),
                     - fgz2 * ( gx*gy*dx + gy*gy*dy + gz*gz*dy ),
                     + fgz2 * ( gx*gx*dx + gz*gz*dx + gx*gy*dy ),
                     + fgz2 * ( gx*gz*dy - gy*gz*dx );
            J_aux = J_aux / std::max(Config::homogTh(),err_i_norm);
            /// 残差
            double s2 = (*it)->sigma2;
            double r = err_i_norm * sqrt(s2);///误差f(x) * 高斯模糊的方差（sigma越大，表明图像分辨率越小，总的像素数越少，单个像素误差表示的位置偏差越大）
            /// 如果采用鲁棒成本函数
            double w  = 1.0;
            w = robustWeightCauchy(r) ;///通过鲁棒核函数滤波误差f(x)

            /// 如果降低远处样本的权重
            ///double zdist   = P_(2) * 1.0 / ( cam->getB()*cam->getFx());
            ///if( false ) w *= 1.0 / ( s2 + zdist * zdist );

            /// 更新H、g、e
            H_p += J_aux * J_aux.transpose() * w;/// H += J * J^T * w ;/// 要叠加每一个特征点的梯度方向所以用了+=，乘上了权重w（w越大，误差越大，权重越大）

            g_p += J_aux * r * w;///g += J * r * w; r是误差
            e_p += r * r * w; ///总的误差
            N_p++;

            /// G-N法：HΔx=g
            /// 其中 H = J^T * J; g = -J^T * f(x)
            /// f(x)为误差（误差函数带入x）
        }
    }

    // 线段特征
    int N_l = 0;
    for( auto it = matched_ls.begin(); it!=matched_ls.end(); it++)
    {
        if( (*it)->inlier )
        {
            /// 投影函数： 3D（上一帧） -DT-> 3D（当前帧） -> 2D（当前帧）
            Vector3d sP_ = DT.block(0,0,3,3) * (*it)->sP + DT.col(3).head(3);
            Vector2d spl_proj = cam->projection( sP_ );///投影到当前帧的2d起点
            Vector3d eP_ = DT.block(0,0,3,3) * (*it)->eP + DT.col(3).head(3);
            Vector2d epl_proj = cam->projection( eP_ );///投影到当前帧的2d终点
            Vector3d l_obs = (*it)->le_obs;///线段的方向向量（上一帧检测的线段在当前帧观测到的方向向量，3D）
                        ///上一帧的le_obs之前已通过insertStereoPair->f2fTracking->matchF2FLine更新为当前帧的le（当前帧的线段方向向量）
            /// 计算投影误差
            Vector2d err_i;///根据(投影后的）两个端点位置和
            err_i(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_i(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            double err_i_norm = err_i.norm();
            /// 估计变量J，H和g
            /// -- 起点
            double gx   = sP_(0); ///投影后的X
            double gy   = sP_(1); ///投影后的Y
            double gz   = sP_(2); ///投影后的Z
            double gz2  = gz*gz;/// Z^2
            double fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);/// fx / Z^2
            double ds   = err_i(0);///起点的投影误差
            double de   = err_i(1);///终点的投影误差
            double lx   = l_obs(0);///方向向量
            double ly   = l_obs(1);
            Vector6d Js_aux;///雅可比
            Js_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );
            /// -- 终点
            gx   = eP_(0); ///投影后的X
            gy   = eP_(1); ///投影后的Y
            gz   = eP_(2); ///投影后的Z
            gz2  = gz*gz;
            fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
            Vector6d Je_aux, J_aux;
            Je_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );
            J_aux = ( Js_aux * ds + Je_aux * de ) / std::max(Config::homogTh(),err_i_norm);///起点和终点的误差函数雅可比分别乘上误差值（作为权重）

            /// 残差
            double s2 = (*it)->sigma2;
            double r = err_i_norm * sqrt(s2);

            /// 应用鲁棒损失函数
            double w  = 1.0;
            w = robustWeightCauchy(r) ;

            /// 估计线段之间的重叠程度
            bool has_overlap = true;
            double overlap = prev_frame->lineSegmentOverlap( (*it)->spl, (*it)->epl, spl_proj, epl_proj );
            if( has_overlap )
                w *= overlap;///损失函数*重叠质量

            /// 如果降低远处的权重
            /*double zdist = max( sP_(2), eP_(2) ) / ( cam->getB()*cam->getFx());
            if( false )
                w *= 1.0 / ( s2 + zdist * zdist );*/

            /// 更新H、g、e
            H_l += J_aux * J_aux.transpose() * w;
            g_l += J_aux * r * w;
            e_l += r * r * w;
            N_l++;
        }

    }

    /// 分别累加点和线段的H、g、e
    H = H_p + H_l;
    g = g_p + g_l;
    e = e_p + e_l;

    /// 归一化误差
    e /= (N_l+N_p);

}

void StereoFrameHandler::optimizeFunctionsRobust(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e )
{

    /// 定义hessian矩阵H、梯度g、残差e
    Matrix6d H_l, H_p;
    Vector6d g_l, g_p;
    double   e_l = 0.0, e_p = 0.0, S_l, S_p;
    H   = Matrix6d::Zero(); H_l = H; H_p = H;
    g   = Vector6d::Zero(); g_l = g; g_p = g;
    e   = 0.0;

    vector<double> res_p, res_l;///存储点特征和线特征的重投影误差

    /// 点特征重投影误差
    for( auto it = matched_pt.begin(); it!=matched_pt.end(); it++)
    {
        if( (*it)->inlier )
        {
            /// 3D -DT-> 3D -> 2D
            Vector3d P_ = DT.block(0,0,3,3) * (*it)->P + DT.col(3).head(3);
            Vector2d pl_proj = cam->projection( P_ );
            /// 投影误差
            Vector2d err_i    = pl_proj - (*it)->pl_obs;
            res_p.push_back( err_i.norm());
        }
    }

    /// 线特征重投影误差
    for( auto it = matched_ls.begin(); it!=matched_ls.end(); it++)
    {
        if( (*it)->inlier )
        {
            /// 3D -DT-> 3D -> 2D
            Vector3d sP_ = DT.block(0,0,3,3) * (*it)->sP + DT.col(3).head(3);
            Vector2d spl_proj = cam->projection( sP_ );
            Vector3d eP_ = DT.block(0,0,3,3) * (*it)->eP + DT.col(3).head(3);
            Vector2d epl_proj = cam->projection( eP_ );
            Vector3d l_obs = (*it)->le_obs;
            /// 投影误差（根据）
            Vector2d err_i;
            err_i(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_i(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            res_l.push_back( err_i.norm() );
        }

    }

    /// 估计残差的尺度
    double s_p = 1.0, s_l = 1.0;
    double th_min = 0.0001;
    double th_max = sqrt(7.815);

    if( false )///不会执行
    {

        res_p.insert( res_p.end(), res_l.begin(), res_l.end() );
        s_p = vector_stdv_mad( res_p );

        ///cout << s_p << "\t";
        ///if( res_p.size() < 4*Config::minFeatures() )
            ///s_p = 1.0;
        ///cout << s_p << endl;

        if( s_p < th_min )
            s_p = th_min;
        if( s_p > th_max )
            s_p = th_max;

        s_l = s_p;

    }
    else///一定会执行
    {
        ///计算中位数
        s_p = vector_stdv_mad( res_p );
        s_l = vector_stdv_mad( res_l );
        ///限幅
        if( s_p < th_min )
            s_p = th_min;
        if( s_p > th_max )
            s_p = th_max;

        if( s_l < th_min )
            s_l = th_min;
        if( s_l > th_max )
            s_l = th_max;

    }

    // 点特征
    int N_p = 0;
    for( auto it = matched_pt.begin(); it!=matched_pt.end(); it++)
    {
        if( (*it)->inlier )
        {
            /// 投影函数和投影误差（又计算了一遍）
            Vector3d P_ = DT.block(0,0,3,3) * (*it)->P + DT.col(3).head(3);
            Vector2d pl_proj = cam->projection( P_ );
            Vector2d err_i    = pl_proj - (*it)->pl_obs;
            double err_i_norm = err_i.norm();
            /// 估计J, H, g
            double gx   = P_(0);
            double gy   = P_(1);
            double gz   = P_(2);
            double gz2  = gz*gz;
            double fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
            double dx   = err_i(0);
            double dy   = err_i(1);

            Vector6d J_aux;///雅可比
            J_aux << + fgz2 * dx * gz,
                     + fgz2 * dy * gz,
                     - fgz2 * ( gx*dx + gy*dy ),
                     - fgz2 * ( gx*gy*dx + gy*gy*dy + gz*gz*dy ),
                     + fgz2 * ( gx*gx*dx + gz*gz*dx + gx*gy*dy ),
                     + fgz2 * ( gx*gz*dy - gy*gz*dx );
            J_aux = J_aux / std::max(Config::homogTh(),err_i_norm);

            double s2 = (*it)->sigma2;
            double r = err_i_norm ;
            /// 应用鲁棒损失函数
            double w  = 1.0;
            double x = r / s_p;
            w = robustWeightCauchy(x) ;

            /// 如果使用不确定性权重（没有使用）
            ///----------------------------------------------------
            if( false )///不会执行
            {
                Matrix2d covp;
                Matrix3d covP_ = (*it)->covP_an;
                MatrixXd J_Pp(2,3), J_pr(1,2);
                /// uncertainty of the projection
                J_Pp  << gz, 0.f, -gx, 0.f, gz, -gy;
                J_Pp  << J_Pp * DT.block(0,0,3,3);
                covp  << J_Pp * covP_ * J_Pp.transpose();
                covp  << covp / std::max(Config::homogTh(),gz2*gz2);               /// Covariance of the 3D projection \hat{p} up to f2*b2*sigma2
                covp  = sqrt(s2) * cam->getB()* cam->getFx() * covp;
                covp(0,0) += s2;
                covp(1,1) += s2;
                /// Point Uncertainty constants
                /*bsigmaP   = f * baseline * sigmaP;
                bsigmaP   = bsigmaP * bsigmaP;
                bsigmaP_inv   = 1.f / bsigmaP;
                sigmaP2       = sigmaP * sigmaP;
                sigmaP2_inv   = 1.f / sigmaP2;
                // Line Uncertainty constants
                bsigmaL   = baseline * sigmaL;
                bsigmaL   = bsigmaL * bsigmaL;
                bsigmaL_inv   = 1.f / bsigmaL;*/
                /// uncertainty of the residual
                J_pr << dx / r, dy / r;
                double cov_res = (J_pr * covp * J_pr.transpose())(0);
                cov_res = 1.0 / std::max(Config::homogTh(),cov_res);
                double zdist   = P_(2) * 1.0 / ( cam->getB()*cam->getFx());

                ///zdist   = 1.0 / std::max(Config::homogTh(),zdist);
                /*cout.setf(ios::fixed,ios::floatfield); cout.precision(8);
                cout << endl << cov_res << " " << 1.0 / cov_res << " " << zdist << " " << 1.0 / zdist << " " << 1.0 / (zdist*40.0) << "\t"
                     << 1.0 / ( 1.0 +  cov_res * cov_res + zdist * zdist ) << " \t"
                     << 1.0 / ( cov_res * cov_res + zdist * zdist )
                     << endl;
                cout.setf(ios::fixed,ios::floatfield); cout.precision(3);*/
                ///w *= 1.0 / ( cov_res * cov_res + zdist * zdist );
                w *= 1.0 / ( s2 + zdist * zdist );
            }
            ///----------------------------------------------------

            /// 更新H、g、e
            H_p += J_aux * J_aux.transpose() * w;
            g_p += J_aux * r * w;
            e_p += r * r * w;
            N_p++;
        }
    }

    // 线特征
    int N_l = 0;
    for( auto it = matched_ls.begin(); it!=matched_ls.end(); it++)
    {
        /// 投影函数和投影误差（又计算了一遍）
        if( (*it)->inlier )
        {
            Vector3d sP_ = DT.block(0,0,3,3) * (*it)->sP + DT.col(3).head(3);
            Vector2d spl_proj = cam->projection( sP_ );
            Vector3d eP_ = DT.block(0,0,3,3) * (*it)->eP + DT.col(3).head(3);
            Vector2d epl_proj = cam->projection( eP_ );
            Vector3d l_obs = (*it)->le_obs;
            Vector2d err_i;
            err_i(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_i(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            double err_i_norm = err_i.norm();
            /// 估计J, H, g
            /// -- 起点
            double gx   = sP_(0);
            double gy   = sP_(1);
            double gz   = sP_(2);
            double gz2  = gz*gz;
            double fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
            double ds   = err_i(0);
            double de   = err_i(1);
            double lx   = l_obs(0);
            double ly   = l_obs(1);
            Vector6d Js_aux;
            Js_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );
            /// -- 终点
            gx   = eP_(0);
            gy   = eP_(1);
            gz   = eP_(2);
            gz2  = gz*gz;
            fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
            Vector6d Je_aux, J_aux;
            Je_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );

            J_aux = ( Js_aux * ds + Je_aux * de ) / std::max(Config::homogTh(),err_i_norm);///雅可比
            /// 定义残差
            double s2 = (*it)->sigma2;
            double r = err_i_norm;
            /// 采用鲁棒损失函数
            double w  = 1.0;
            double x = r / s_l;
            w = robustWeightCauchy(x) ;
            /// 估计线段的重合程度
            bool has_overlap = true;
            double overlap = prev_frame->lineSegmentOverlap( (*it)->spl, (*it)->epl, spl_proj, epl_proj );
            if( has_overlap )
                w *= overlap;

            ///----------------- DEBUG: 27/11/2017 ----------------------
            if( false )
            {
                ///double cov3d = (*it)->cov3d;
                ///cov3d = 1.0;
                ///w *= 1.0 / ( 1.0 +  cov3d * cov3d + zdist * zdist );
                double zdist = max( sP_(2), eP_(2) ) / ( cam->getB()*cam->getFx());
                w *= 1.0 / ( s2 + zdist * zdist );
            }
            ///----------------------------------------------------------

            /// 更新H、g、e
            H_l += J_aux * J_aux.transpose() * w;
            g_l += J_aux * r * w;
            e_l += r * r * w;
            N_l++;
        }

    }

    /// 汇总H、g、e
    H = H_p + H_l;
    g = g_p + g_l;
    e = e_p + e_l;

    /// 归一化误差
    e /= (N_l+N_p);

}

// 移除异常值
/// optimizePose()调用
void StereoFrameHandler::removeOutliers(Matrix4d DT)
{

    ///if not usig mad stdv, use just a fixed threshold (sqrt(7.815)) to filter outliers (with a single for loop...)

    if (Config::hasPoints()) {
        /// 点特征
        vector<double> res_p;///存储投影误差
        res_p.reserve(matched_pt.size());
        int iter = 0;
        /// 计算投影误差
        for( auto it = matched_pt.begin(); it!=matched_pt.end(); it++, iter++)
        {
            Vector3d P_ = DT.block(0,0,3,3) * (*it)->P + DT.col(3).head(3);
            Vector2d pl_proj = cam->projection( P_ );
            res_p.push_back( ( pl_proj - (*it)->pl_obs ).norm() * sqrt((*it)->sigma2) );
            ///res_p.push_back( ( pl_proj - (*it)->pl_obs ).norm() );
        }
        /// 估计鲁棒参数
        double p_stdv, p_mean, inlier_th_p;
        vector_mean_stdv_mad( res_p, p_mean, p_stdv );
        inlier_th_p = Config::inlierK() * p_stdv;
        ///inlier_th_p = sqrt(7.815);
        ///cout << endl << p_mean << " " << p_stdv << "\t" << inlier_th_p << endl;

        /// 滤除外点
        iter = 0;
        for( auto it = matched_pt.begin(); it!=matched_pt.end(); it++, iter++)
        {
            if( (*it)->inlier && fabs(res_p[iter]-p_mean) > inlier_th_p )///误差较大，内点变为外点
            {
                (*it)->inlier = false;
                n_inliers--;
                n_inliers_pt--;
            }
        }
    }

    if (Config::hasLines()) {
        /// 线特征
        vector<double> res_l; /// 存储重投影误差
        res_l.reserve(matched_ls.size());
        int iter = 0;
        for( auto it = matched_ls.begin(); it!=matched_ls.end(); it++, iter++)
        {
            /// 计算投影误差
            Vector3d sP_ = DT.block(0,0,3,3) * (*it)->sP + DT.col(3).head(3);
            Vector3d eP_ = DT.block(0,0,3,3) * (*it)->eP + DT.col(3).head(3);
            Vector2d spl_proj = cam->projection( sP_ );
            Vector2d epl_proj = cam->projection( eP_ );
            Vector3d l_obs    = (*it)->le_obs;
            Vector2d err_li;
            err_li(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_li(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            res_l.push_back( err_li.norm() * sqrt( (*it)->sigma2 ) );
            ///res_l.push_back( err_li.norm() );
        }

        /// 估计鲁棒参数
        double l_stdv, l_mean, inlier_th_l;
        vector_mean_stdv_mad( res_l, l_mean, l_stdv );
        inlier_th_l = Config::inlierK() * l_stdv;
        ///inlier_th_p = sqrt(7.815);
        ///cout << endl << l_mean << " " << l_stdv << "\t" << inlier_th_l << endl << endl;

        /// 滤除外点
        iter = 0;
        for( auto it = matched_ls.begin(); it!=matched_ls.end(); it++, iter++)
        {
            if( fabs(res_l[iter]-l_mean) > inlier_th_l  && (*it)->inlier )///误差较大，内点变为外点
            {
                (*it)->inlier = false;
                n_inliers--;
                n_inliers_ls--;
            }
        }
    }

    if (n_inliers != (n_inliers_pt + n_inliers_ls))
        throw runtime_error("[StVO; stereoFrameHandler] Assetion failed: n_inliers != (n_inliers_pt + n_inliers_ls)");
}

}
