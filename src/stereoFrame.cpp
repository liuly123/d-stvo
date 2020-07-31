#include <stereoFrame.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

#include "lineIterator.h"
#include "matching.h"

namespace StVO{

StereoFrame::StereoFrame(){}
// 构造函数
StereoFrame::StereoFrame(const Mat img_l_, const Mat img_r_ , const int idx_, PinholeStereoCamera *cam_) :
    img_l(img_l_), img_r(img_r_), frame_idx(idx_), cam(cam_) {

    if (img_l_.size != img_r_.size)
        throw std::runtime_error("[StereoFrame] Left and right images have different sizes");

    inv_width  = GRID_COLS / static_cast<double>(img_l.cols);/// 1 / 每个网格的行宽 （用于求出像素属于第几个网格）
    inv_height = GRID_ROWS / static_cast<double>(img_l.rows);/// 1 / 每个网格的列宽
}

StereoFrame::~StereoFrame()
{
    for( auto pt: stereo_pt )
        delete pt;
    for( auto ls: stereo_ls )
        delete ls;
}

// 提取双目的特征
void StereoFrame::extractStereoFeatures( double llength_th, int fast_th )
{

    if( Config::plInParallel() )///并行提取点和线段
    {
        auto detect_p = async(launch::async, &StereoFrame::detectStereoPoints,        this, fast_th );///创建额外的线程并执行
        auto detect_l = async(launch::async, &StereoFrame::detectStereoLineSegments,  this, llength_th );
        detect_p.wait();
        detect_l.wait();
    }
    else
    {
        detectStereoPoints(fast_th);
        detectStereoLineSegments(llength_th);
    }

}

// 双目点特征提取和匹配-------------------------------------------------------------------------------------------------------------------------------------
void StereoFrame::detectStereoPoints( int fast_th )
{

    if( !Config::hasPoints() )
        return;

    /// 检测并估计左右图像的每个描述符
    if( Config::lrInParallel() )///并行
    {
        auto detect_l = async(launch::async, &StereoFrame::detectPointFeatures, this, img_l, ref(points_l), ref(pdesc_l), fast_th );
        auto detect_r = async(launch::async, &StereoFrame::detectPointFeatures, this, img_r, ref(points_r), ref(pdesc_r), fast_th );
        detect_l.wait();
        detect_r.wait();
    }
    else
    {
        detectPointFeatures( img_l, points_l, pdesc_l, fast_th );
        detectPointFeatures( img_r, points_r, pdesc_r, fast_th );
    }

    /// 执行双目匹配
    matchStereoPoints(points_l, points_r, pdesc_l, pdesc_r, (frame_idx==0) );// (frame_idx==0)判断句
}

// 特征点提取
void StereoFrame::detectPointFeatures( Mat img, vector<KeyPoint> &points, Mat &pdesc, int fast_th )///pdesc是描述子
{
    if( Config::hasPoints() )
    {
        int fast_th_ = Config::orbFastTh();/// FAST阈值
        if( fast_th != 0 )
            fast_th_ = fast_th;
        Ptr<ORB> orb = ORB::create(
                Config::orbNFeatures(),   /// 检测的最大点数
                Config::orbScaleFactor(), /// FAST检测器的金字塔抽取比
                Config::orbNLevels(),     /// 金字塔层数
                Config::orbEdgeTh(),      /// 未检测到特征的边框大小
                0,              /// 金字塔起始层
                Config::orbWtaK(),       /// 生成oriented BRIEF描述子的每个元素的点数
                Config::orbScore(),      /// FAST评分
                Config::orbPatchSize(),  /// oriented BRIEF描述符使用的patch的大小
                fast_th_ );              /// 默认FAST阈值
        orb->detectAndCompute( img, Mat(), points, pdesc, false);
}

}

// 双目特征点匹配
void StereoFrame::matchStereoPoints( vector<KeyPoint> points_l, vector<KeyPoint> points_r, Mat &pdesc_l_, Mat pdesc_r, bool initial )///initial是否是初始帧
{
    ///匹配描述子（通过比较一维Mat的差异）
    ///检查点的位置误差是否在允许范围内
    ///计算特征点的3D位置
    ///push_back特征点到frame
    stereo_pt.clear();///双目匹配成功的点，清空
    if (!Config::hasPoints() || points_l.empty() || points_r.empty())
        return;

    /// 左图特征点所属grid存入coords（只计算了所属grid的编号，并没有加载到grid中）
    std::vector<point_2d> coords;/// 左图中点所在的grid
    coords.reserve(points_l.size());
    for (const KeyPoint &kp : points_l)
        coords.push_back(std::make_pair(kp.pt.x * inv_width, kp.pt.y * inv_height));///std::vector<point_2d>就是std::pair<int, int>

    ///右图特征点填充到grid中（存入特征点数据到对应的grid）
    GridStructure grid(GRID_ROWS, GRID_COLS);
    for (int idx = 0; idx < points_r.size(); ++idx) {
        const KeyPoint &kp = points_r[idx];
        grid.at(kp.pt.x * inv_width, kp.pt.y * inv_height).push_back(idx);
    }

    GridWindow w;//极线搜索的窗口
    w.width = std::make_pair(Config::matchingSWs(), 0);///查找双目匹配的窗口大小（像素），默认10
    w.height = std::make_pair(0, 0);///一个像素宽的像素带

    std::vector<int> matches_12;
    /// 遍历coords中的左grid的特征点，与右边对应grid中的特征点进行描述子匹配，已找到当前grid内的最佳特征匹配（在matching.h中实现）
    matchGrid(coords, pdesc_l, grid, pdesc_r, w, matches_12);
///    match(pdesc_l, pdesc_r, Config::minRatio12P(), matches_12);

    /// 匹配点对齐
    Mat pdesc_l_aux;
    int pt_idx = 0;
    for (int i1 = 0; i1 < matches_12.size(); ++i1) {
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        /// 检测双目对极约束（点都在平面上才有对极约束）
        if (std::abs(points_l[i1].pt.y - points_r[i2].pt.y) <= Config::maxDistEpip()) {///差异<= 极线以像素为单位的最大距离
            /// 检查最小差异
            double disp_ = points_l[i1].pt.x - points_r[i2].pt.x;
            if (disp_ >= Config::minDisp()){/// >= 最小视差（避免无穷远点），disp_：x像素差值，用于计算3D位姿
                pdesc_l_aux.push_back(pdesc_l_.row(i1));
                Vector2d pl_(points_l[i1].pt.x, points_l[i1].pt.y); //以左边摄像头为相机远点
                Vector3d P_ = cam->backProjection(pl_(0), pl_(1), disp_);///通过相机模型反投影得到3D位置
                if (initial)///如果是初始化
                    stereo_pt.push_back(new PointFeature(pl_, disp_, P_, pt_idx++, points_l[i1].octave));///PointFeature(像素坐标，像素x视差，（相机坐标系下）空间坐标，索引，提取到特征点的金字塔层数)
                else
                    stereo_pt.push_back(new PointFeature(pl_, disp_, P_, -1, points_l[i1].octave));
            }
        }
    }

    pdesc_l_ = pdesc_l_aux;
}

// 双目线段特征提取和匹配-------------------------------------------------------------------------------------------------------------------------------------------

void StereoFrame::detectStereoLineSegments(double llength_th)
{

    if( !Config::hasLines() )
        return;

    /// 检提取并估计左右图像的每个线段描述子
    bool leftIsGood, rightIsGood;
    if( Config::lrInParallel() )///并行
    {
        auto detect_l = async(launch::async, &StereoFrame::detectLineFeatures, this, img_l, ref(lines_l), ref(ldesc_l), llength_th, ref(leftIsGood) );
        auto detect_r = async(launch::async, &StereoFrame::detectLineFeatures, this, img_r, ref(lines_r), ref(ldesc_r), llength_th, ref(rightIsGood) );
        detect_l.wait();
        detect_r.wait();
    }
    else
    {
        detectLineFeatures( img_l, lines_l, ldesc_l, llength_th ,leftIsGood );
        detectLineFeatures( img_r, lines_r, ldesc_r, llength_th ,rightIsGood );
    }
    isGoodKeyFrame = (leftIsGood && rightIsGood);
    //cout << (isGoodKeyFrame ? "isGoodKeyFrame" : "isBadKeyFrame") << endl;

    /// 双目匹配
    matchStereoLines(lines_l,  lines_r,  ldesc_l, ldesc_r, (frame_idx==0));// (frame_idx==0)判断句

}
// 双目线段提取程序
void StereoFrame::detectLineFeatures( Mat img, vector<KeyLine> &lines, Mat &ldesc, double min_line_length,bool &isGood )
{

    lines.clear();
    Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
    if( Config::hasLines() )
    {

        if( !Config::useFLDLines() )
        {///使用LSD提取线段
            Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
            line_descriptor::LSDDetectorC::LSDOptions opts;///lsd参数
            opts.refine       = Config::lsdRefine();
            opts.scale        = Config::lsdScale();
            opts.sigma_scale  = Config::lsdSigmaScale();
            opts.quant        = Config::lsdQuant();
            opts.ang_th       = Config::lsdAngTh();
            opts.log_eps      = Config::lsdLogEps();
            opts.density_th   = Config::lsdDensityTh();
            opts.n_bins       = Config::lsdNBins();
            opts.min_length   = min_line_length;
            lsd->detect( img, lines, Config::lsdScale(), 1, opts);
            /// 滤除部分线段
            if( lines.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )//超过了最多允许的线段数量
            {
                /// 根据response对线段进行排序
                ///response： 用线的长度与图像的宽度和高度之间的最大值之比表示
                sort( lines.begin(), lines.end(), sort_lines_by_response() );
                ///sort( lines.begin(), lines.end(), sort_lines_by_length() );
                lines.resize(Config::lsdNFeatures());///resize就把短的线删除掉
                for( int i = 0; i < Config::lsdNFeatures(); ++i)///重新分配索引
                    lines[i].class_id = i;
            }

            lbd->compute( img, lines, ldesc);// 计算线段描述子

            // TODO: l-lk:判断是否是合适的关键帧
            
            //注意此时线段存储在lines_l和lines_r中，用内置的KeyLine格式
            int lines_num = 0;//线段数量
            double lines_length = 0;//线段总长度
            vector<KeyLine>::iterator iter;
            for(iter = lines.begin(); iter != lines.end(); ++iter)
            {
                lines_length += iter->lineLength;
                ++lines_num;
            }
            isGood = (lines_num >= Config::minLinesNum() && lines_length >= Config::minTotalLength());
        }
        // 未用
        else///使用FLD提取线段（也是在line_descriptor中实现的）
        {
            Mat fld_img, img_gray;
            vector<Vec4f> fld_lines;

            if( img.channels() != 1 )
            {
                cv::cvtColor( img, img_gray, CV_RGB2GRAY );
                img_gray.convertTo( fld_img, CV_8UC1 );
            }
            else
                img.convertTo( fld_img, CV_8UC1 );

            Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(min_line_length);
            fld->detect( fld_img, fld_lines );

            /// filter lines
            if( fld_lines.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
            {
                /// sort lines by their response
                sort( fld_lines.begin(), fld_lines.end(), sort_flines_by_length() );
                fld_lines.resize(Config::lsdNFeatures());
            }

            /// loop over lines object transforming into a vector<KeyLine>
            lines.reserve(fld_lines.size());
            for( int i = 0; i < fld_lines.size(); i++ )
            {
                KeyLine kl;
                double octaveScale = 1.f;
                int    octaveIdx   = 0;

                kl.startPointX     = fld_lines[i][0] * octaveScale;
                kl.startPointY     = fld_lines[i][1] * octaveScale;
                kl.endPointX       = fld_lines[i][2] * octaveScale;
                kl.endPointY       = fld_lines[i][3] * octaveScale;

                kl.sPointInOctaveX = fld_lines[i][0];
                kl.sPointInOctaveY = fld_lines[i][1];
                kl.ePointInOctaveX = fld_lines[i][2];
                kl.ePointInOctaveY = fld_lines[i][3];

                kl.lineLength = (float) sqrt( pow( fld_lines[i][0] - fld_lines[i][2], 2 ) + pow( fld_lines[i][1] - fld_lines[i][3], 2 ) );

                kl.angle    = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
                kl.class_id = i;
                kl.octave   = octaveIdx;
                kl.size     = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
                kl.pt       = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

                kl.response = kl.lineLength / max( fld_img.cols, fld_img.rows );
                cv::LineIterator li( fld_img, Point2f( fld_lines[i][0], fld_lines[i][1] ), Point2f( fld_lines[i][2], fld_lines[i][3] ) );
                kl.numOfPixels = li.count;

                lines.push_back( kl );

            }

            /// compute lbd descriptor
            lbd->compute( fld_img, lines, ldesc);
        }

    }
}

// 线段匹配
void StereoFrame::matchStereoLines( vector<KeyLine> lines_l, vector<KeyLine> lines_r, Mat &ldesc_l_, Mat ldesc_r, bool initial )
{
    //输入：lines_l, lines_r, ldesc_l_, ldesc_r
    //输出：stereo_ls, ldesc_l_ (线段vector和描述子Mat)

    stereo_ls.clear();
    if (!Config::hasLines() || lines_l.empty() || lines_r.empty())
        return;

    std::vector<line_2d> coords;/// 左图中线段所在的grid
    coords.reserve(lines_l.size());
    for (const KeyLine &kl : lines_l)
        coords.push_back(std::make_pair(std::make_pair(kl.startPointX * inv_width, kl.startPointY * inv_height),
                                        std::make_pair(kl.endPointX * inv_width, kl.endPointY * inv_height)));

    /// 右图线段填满grid和directions
    list<pair<int, int>> line_coords;
    GridStructure grid(GRID_ROWS, GRID_COLS);
    std::vector<std::pair<double, double>> directions(lines_r.size());
    for (int idx = 0; idx < lines_r.size(); ++idx) {
        const KeyLine &kl = lines_r[idx];

        std::pair<double, double> &v = directions[idx];
        v = std::make_pair((kl.endPointX - kl.startPointX) * inv_width, (kl.endPointY - kl.startPointY) * inv_height);///填充线的方向
        normalize(v);

        getLineCoords(kl.startPointX * inv_width, kl.startPointY * inv_height, kl.endPointX * inv_width, kl.endPointY * inv_height, line_coords);///线段的坐标（调整start和end）
        for (const std::pair<int, int> &p : line_coords)
            grid.at(p.first, p.second).push_back(idx);///线段所属的grid
    }

    GridWindow w;
    w.width = std::make_pair(Config::matchingSWs(), 0);
    w.height = std::make_pair(0, 0);

    std::vector<int> matches_12;
    matchGrid(coords, ldesc_l, grid, ldesc_r, directions, w, matches_12);/// 在grid中匹配线段，匹配结果存入matches_12
///    match(ldesc_l, ldesc_r, Config::minRatio12P(), matches_12);

    /// 端点对齐
    Mat ldesc_l_aux;///最终的描述子
    int ls_idx = 0;
    for (int i1 = 0; i1 < matches_12.size(); ++i1) {
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        /// 估计端点的差异
        // lines_l[i1]:左图的一个KeyLine
        // lines_r[i2]:右图对应的Keyline
        Vector3d sp_l; sp_l << lines_l[i1].startPointX, lines_l[i1].startPointY, 1.0;///左起始点像素位置
        Vector3d ep_l; ep_l << lines_l[i1].endPointX,   lines_l[i1].endPointY,   1.0;///左终止点像素位置
        Vector3d le_l; le_l << sp_l.cross(ep_l);// 起始点和终止点的叉积
        le_l = le_l / std::sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );///对叉积取模

        Vector3d sp_r; sp_r << lines_r[i2].startPointX, lines_r[i2].startPointY, 1.0;///右边起始点像素位置
        Vector3d ep_r; ep_r << lines_r[i2].endPointX,   lines_r[i2].endPointY,   1.0;///右边终止点像素位置
        Vector3d le_r; le_r << sp_r.cross(ep_r);

        double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );/// 双目线段y坐标重叠的质量

        double disp_s, disp_e;
        sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
        ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
        filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );///计算起点x的差和终点x的差（sp_l.head(2)取前2的元素）

        // 检查最小差异
        if( disp_s >= Config::minDisp() && disp_e >= Config::minDisp()///x视差不能太小（避免无穷远）
            && std::abs( sp_l(1)-ep_l(1) ) > Config::lineHorizTh()///避免检测到水平线
            && std::abs( sp_r(1)-ep_r(1) ) > Config::lineHorizTh()
            && overlap > Config::stereoOverlapTh() )///y的重叠阈值
        {// 计算线的3维位姿
            /// 用起点和终点两个点计算的
            Vector3d sP_; sP_ = cam->backProjection( sp_l(0), sp_l(1), disp_s);/// 左图起点2D -> 3D (输入spx,spy，spx的视差)
            Vector3d eP_; eP_ = cam->backProjection( ep_l(0), ep_l(1), disp_e);/// 左图终点2D -> 3D
            double angle_l = lines_l[i1].angle;
            if( initial )///如果是初始化阶段
            {
                ldesc_l_aux.push_back( ldesc_l_.row(i1) );///push_back描述子和线段特征
                //TODO: 添加了右边线段的2D坐标（原来只有左边有）
                stereo_ls.push_back( new LineFeature(Vector2d(sp_l(0), sp_l(1)), Vector2d(lines_r[i2].startPointX, lines_r[i2].startPointY), disp_s, sP_,///起点：spl，spr，px视差，三维位姿
                                                     Vector2d(ep_l(0), ep_l(1)), Vector2d(lines_r[i2].endPointX,   lines_r[i2].endPointY  ), disp_e, eP_,///终点：epl，epr，px视差，三维位姿
                                                     le_l,angle_l,ls_idx,lines_l[i1].octave) );///起点终点的叉积取模、线段方向角、线段索引号、金字塔层数
                ls_idx++;//初始化时每个线段都是唯一的，所以都给它设置id，并且从0开始
            }
            else
            {
                ldesc_l_aux.push_back( ldesc_l_.row(i1) );
                stereo_ls.push_back( new LineFeature(Vector2d(sp_l(0), sp_l(1)), Vector2d(lines_r[i2].startPointX, lines_r[i2].startPointY), disp_s, sP_,
                                                     Vector2d(ep_l(0), ep_l(1)), Vector2d(lines_r[i2].endPointX,   lines_r[i2].endPointY  ), disp_e, eP_,
                                                     le_l,angle_l,-1,lines_l[i1].octave) );//id同一为-1，在后面修改
            }
        }
    }

    ldesc_l_aux.copyTo(ldesc_l_);///最终的描述子存入ldesc_l_
}


// 计算起点x的差和终点x的差
void StereoFrame::filterLineSegmentDisparity( Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e )
{
    disp_s = spl(0) - spr(0);///起点x的差
    disp_e = epl(0) - epr(0);///终点x的差
    /// 如果差别太大就忽略
    if(  min( disp_s, disp_e ) / max( disp_s, disp_e ) < Config::lsMinDispRatio() )/// 差别大的/差别小的 > 阈值 TODO 合理吗？
    {
        disp_s = -1.0;
        disp_e = -1.0;
    }
}

/* Auxiliar methods */

// 计算双目线段y坐标重叠的质量
double StereoFrame::lineSegmentOverlapStereo( double spl_obs, double epl_obs, double spl_proj, double epl_proj  )///spl_proj其实是sp_r，因为以左相机为基准，所以
{

    double overlap = 1.f;///交叠

    if( fabs( epl_obs - spl_obs ) > Config::lineHorizTh() ) /// 法线（包括垂直线），lineHorizTh：避免（像素）水平线的参数
    {
        double sln    = min(spl_obs,  epl_obs);///左起始x
        double eln    = max(spl_obs,  epl_obs);///左结束x
        double spn    = min(spl_proj, epl_proj);///右起始x
        double epn    = max(spl_proj, epl_proj);///右结束x

        double length = eln-spn;

        if ( (epn < sln) || (spn > eln) )///没法重合
            overlap = 0.f;
        else{
            if ( (epn>eln) && (spn<sln) )
                overlap = eln-sln;
            else
                overlap = min(eln,epn) - max(sln,spn);
        }

        if(length>0.01f)
            overlap = overlap / length;
        else
            overlap = 0.f;

        if( overlap > 1.f )
            overlap = 1.f;

    }

    return overlap;

}

// 计算帧间线段（斜率）重叠质量
/// StereoFrameHandler::optimizeFunctions用到
double StereoFrame::lineSegmentOverlap( Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj  )
{

    double overlap = 1.f;

    if( fabs(spl_obs(0)-epl_obs(0)) < 1.0 )/// 直线x坐标的差
    {

        /// 线方程
        Vector2d l = epl_obs - spl_obs;

        /// 相交点
        Vector2d spl_proj_line, epl_proj_line;
        spl_proj_line << spl_obs(0), spl_proj(1);
        epl_proj_line << epl_obs(0), epl_proj(1);

        /// 估计λ函数的重叠
        double lambda_s = (spl_proj_line(1)-spl_obs(1)) / l(1);
        double lambda_e = (epl_proj_line(1)-spl_obs(1)) / l(1);

        double lambda_min = min(lambda_s,lambda_e);
        double lambda_max = max(lambda_s,lambda_e);

        if( lambda_min < 0.f && lambda_max > 1.f )
            overlap = 1.f;
        else if( lambda_max < 0.f || lambda_min > 1.f )
            overlap = 0.f;
        else if( lambda_min < 0.f )
            overlap = lambda_max;
        else if( lambda_max > 1.f )
            overlap = 1.f - lambda_min;
        else
            overlap = lambda_max - lambda_min;

    }
    else if( fabs(spl_obs(1)-epl_obs(1)) < 1.0 )/// 水平线（之前已删除）
    {

        /// 线方程
        Vector2d l = epl_obs - spl_obs;

        /// 相交点
        Vector2d spl_proj_line, epl_proj_line;
        spl_proj_line << spl_proj(0), spl_obs(1);
        epl_proj_line << epl_proj(0), epl_obs(1);

        /// 估计λ函数的重叠
        double lambda_s = (spl_proj_line(0)-spl_obs(0)) / l(0);
        double lambda_e = (epl_proj_line(0)-spl_obs(0)) / l(0);

        double lambda_min = min(lambda_s,lambda_e);
        double lambda_max = max(lambda_s,lambda_e);

        if( lambda_min < 0.f && lambda_max > 1.f )
            overlap = 1.f;
        else if( lambda_max < 0.f || lambda_min > 1.f )
            overlap = 0.f;
        else if( lambda_min < 0.f )
            overlap = lambda_max;
        else if( lambda_max > 1.f )
            overlap = 1.f - lambda_min;
        else
            overlap = lambda_max - lambda_min;

    }
    else /// 非简并情况
    {

        /// 线方程
        Vector2d l = epl_obs - spl_obs;
        double a = spl_obs(1)-epl_obs(1);
        double b = epl_obs(0)-spl_obs(0);
        double c = spl_obs(0)*epl_obs(1) - epl_obs(0)*spl_obs(1);

        /// 相交点
        Vector2d spl_proj_line, epl_proj_line;
        double lxy = 1.f / (a*a+b*b);

        spl_proj_line << ( b*( b*spl_proj(0)-a*spl_proj(1))-a*c ) * lxy,
                         ( a*(-b*spl_proj(0)+a*spl_proj(1))-b*c ) * lxy;

        epl_proj_line << ( b*( b*epl_proj(0)-a*epl_proj(1))-a*c ) * lxy,
                         ( a*(-b*epl_proj(0)+a*epl_proj(1))-b*c ) * lxy;

        /// 估估计λ函数的重叠
        double lambda_s = (spl_proj_line(0)-spl_obs(0)) / l(0);
        double lambda_e = (epl_proj_line(0)-spl_obs(0)) / l(0);

        double lambda_min = min(lambda_s,lambda_e);
        double lambda_max = max(lambda_s,lambda_e);

        if( lambda_min < 0.f && lambda_max > 1.f )
            overlap = 1.f;
        else if( lambda_max < 0.f || lambda_min > 1.f )
            overlap = 0.f;
        else if( lambda_min < 0.f )
            overlap = lambda_max;
        else if( lambda_max > 1.f )
            overlap = 1.f - lambda_min;
        else
            overlap = lambda_max - lambda_min;

    }

    return overlap;

}

// 画出双目frame
/// main函数中使用（场景的左下角）
Mat StereoFrame::plotStereoFrame()
{

    /// 创建新图像以对其进行修改
    Mat img_l_aux;
    img_l.copyTo( img_l_aux );
    if( img_l_aux.channels() == 1 )
        cvtColor(img_l_aux, img_l_aux, CV_GRAY2BGR, 3);
    else if (img_l_aux.channels() == 4)
        cvtColor(img_l_aux, img_l_aux, CV_BGRA2BGR, 3);
    else if (img_l_aux.channels() != 3)
        throw std::runtime_error(std::string("[StereoFrame->plotStereoFrame] unsupported image format: ") +
                                 std::to_string(img_l_aux.channels()));
    img_l_aux.convertTo(img_l_aux, CV_8UC3);///图像格式为RGB3通道

    /// 变量
    unsigned int    r = 0, g, b = 0;///颜色
    Point2f         p,q;///点的位置
    double          thick = 1.5;///粗细
    int             k = 0, radius  = 3;

    for( auto pt_it = stereo_pt.begin(); pt_it != stereo_pt.end(); pt_it++)// 画出点特征
    {
        if( (*pt_it)->inlier )
        {
            g = 200;///显示为绿色
            p = cv::Point( int((*pt_it)->pl(0)), int((*pt_it)->pl(1)) );///位置
            circle( img_l_aux, p, radius, Scalar(b,g,r), thick);///画绿色圆圈
        }
    }

    for( auto ls_it = stereo_ls.begin(); ls_it != stereo_ls.end(); ls_it++)// 画出线特征
    {
        if( (*ls_it)->inlier )
        {
            g = 200;
            p = cv::Point( int((*ls_it)->spl(0)), int((*ls_it)->spl(1)) );
            q = cv::Point( int((*ls_it)->epl(0)), int((*ls_it)->epl(1)) );
            line( img_l_aux, p, q, Scalar(b,g,r), thick);///画出绿色线段
        }
    }

    return img_l_aux;
}

// TODO: l-lk部分：线段跟踪和特征点提取匹配
void StereoFrame::directTracking( StereoFrame* prev_frame, int fast_th )
{
    if( Config::plInParallel() )///并行提取点和线段
    {
        auto detect_p   = async(launch::async, &StereoFrame::detectStereoPoints,          this, fast_th );      //对于特征电，按原来的程序执行
        auto tracking_l = async(launch::async, &StereoFrame::trackingStereoLineSegments,  this, prev_frame );   //对于线段，采用直接跟踪的方法
        
        detect_p.wait();
        tracking_l.wait();
    }
    else
    {
        detectStereoPoints(fast_th);
        trackingStereoLineSegments( prev_frame );
    }
}

void StereoFrame::trackingStereoLineSegments( StereoFrame* prev_frame )
{//并行跟踪左右图像的线段
    if( !Config::hasLines() )
        return;

    bool leftIsGood, rightIsGood;
    if( Config::plInParallel() )//Config::lrInParallel()
    {
        auto detect_l = async(launch::async, &StereoFrame::trackingLineFeatures, this, prev_frame, img_l, ref(direct_lines_l), ref(leftIsGood), true );
        auto detect_r = async(launch::async, &StereoFrame::trackingLineFeatures, this, prev_frame, img_r, ref(direct_lines_r), ref(rightIsGood), false );
        detect_l.wait();
        detect_r.wait();
    }
    else
    {
        trackingLineFeatures( prev_frame, img_l, direct_lines_l, leftIsGood ,true );
        trackingLineFeatures( prev_frame, img_r, direct_lines_r, rightIsGood ,false );
    }
    trackingGood = ( leftIsGood && rightIsGood );
    // cout << (leftIsGood ? "leftIsGood" : "leftIsBad") << endl;
    // cout << (rightIsGood ? "rightIsGood" : "rightIsBad") << endl;
    
    // for (auto line = direct_lines_l.begin(); line != direct_lines_l.end(); line++)///遍历线段
    // {
    //     cout << "spr=\n" << line->spx << endl;
    //     cout << "epr=\n" << line->epx << endl;
    // }

    // Mat image_show;
    // cvtColor(img_l,image_show,CV_GRAY2BGR);
    // for (auto line = direct_lines_l.begin(); line != direct_lines_l.end(); line++)///遍历线段
    // {
    //     cv::line(image_show,
    //                 Point(line->spx[0],line->spx[1]),
    //                 Point(line->epx[0],line->epx[1]),
    //                 Scalar(0,0,200),1,CV_AA);///RGB颜色、粗细、抗锯齿
    // }
    // imshow("DirectTracking", image_show);
    // cv::waitKey(10000);
    //cv::destroyWindow("DirectTracking");

    /// 双目匹配
    matchDirectStereoLines( prev_frame->stereo_ls, direct_lines_l, direct_lines_r );

}

void StereoFrame::trackingLineFeatures( StereoFrame* prev_frame, Mat img, list<Line> & tracked_lines, bool &trackGood, bool isLeft)
{//跟踪一张图像

    //复制一份上一个frame的prev_lines(按顺序)
    vector< LineFeature* > prev_lines_vector = prev_frame->stereo_ls;//指向上一帧线段vector
    vector< LineFeature* >::iterator iter;

    //cout << "start copy " << (isLeft ? "left_" : "right_") << "prev_lines:" << prev_lines_vector.size() << endl;
    if(isLeft)
    {
        for(iter = prev_lines_vector.begin(); iter != prev_lines_vector.end(); iter++)
        {
            //cout << "spl=" << (*iter)->spl << endl;
            //cout << "epl=" << (*iter)->epl << endl;
            Line line( (*iter)->spl, (*iter)->epl );//实例化line时会进行采样
            tracked_lines.push_back(line);//复制一份
        }
        //cout << "tracked_lines.size()=" << tracked_lines.size() << endl;
    }
    else
    {
        int i = 0;
        for(iter = prev_lines_vector.begin(); iter != prev_lines_vector.end(); ++iter,++i)
        {
            //cout << "spr=" << (*iter)->spr << endl;
            //cout << "epr=" << (*iter)->epr << endl;
            Line line( (*iter)->spr, (*iter)->epr );
            tracked_lines.push_back(line);
        }
        //cout << "tracked_lines.size()=" << tracked_lines.size() << endl;
    }

    // if(isLeft)
    // {
    //     Mat image;
    //     cvtColor(img,image,CV_GRAY2BGR);
    //     for (auto line = tracked_lines.begin(); line != tracked_lines.end(); line++)///遍历线段
    //     {
    //         cv::line(image,
    //                     Point(line->spx[0],line->spx[1]),
    //                     Point(line->epx[0],line->epx[1]),
    //                     Scalar(0,0,200),1,CV_AA);///RGB颜色、粗细、抗锯齿
    //     }
    //     string title = "before_left";
    //     imshow(title, image);
    //     cv::waitKey(1000);
    // }

    //（通过跟踪采样点）跟踪这些线段
    vector<Point2f> prev_keypoints;
    vector<Point2f> next_keypoints;

    for (auto line = tracked_lines.begin(); line != tracked_lines.end(); line++)///遍历线段
        for (auto point = line->SampPoints.begin(); point != line->SampPoints.end(); point++)//遍历线段的采样点
            prev_keypoints.push_back(*point);///所有线段中的采样点都加载到一个vector中

    vector<unsigned char> status;
    vector<float> error;

    if(isLeft)
        cv::calcOpticalFlowPyrLK(prev_frame->img_l, img, prev_keypoints, next_keypoints, status, error);
    else
        cv::calcOpticalFlowPyrLK(prev_frame->img_r, img, prev_keypoints, next_keypoints, status, error);

    // 把跟丢的点删掉(线段数量不变)
    int i = 0;
    for (auto line = tracked_lines.begin(); line != tracked_lines.end(); line++)//遍历线段
    {
        line->LastPointNum = line->SampPoints.size();///更新上一帧的采样点数
        list<Point2f> &points = line->SampPoints;
        for (auto point = points.begin(); point != points.end(); i++)
        {
            if (status[i] == 0)
            {
                point = points.erase(point);//跟丢了就删掉(注意写法)
                continue;
            }
            *point = Point2f(next_keypoints[i].x, next_keypoints[i].y);//没跟丢就更新新的位置(注意是直接修改Line里的SampPoints)
            //cout << *point << endl;
            point++;
        }
    }

    //拟合直线。把不好线段作标记（上一帧和当前帧的，按顺序作标记，保证一一对应）
    list<Line>::iterator tracked_line = tracked_lines.begin();
    for (tracked_line = tracked_lines.begin(); tracked_line != tracked_lines.end(); ++tracked_line)//遍历每一个线段
    {
        if (tracked_line->FitLine(tracked_frame_count) && tracked_line->FilterLine())///拟合线段并剔除不好的线段
        {
            //line->RefineLine(image);///线段长度细化
            //line->Sampling();
            tracked_line->trackedGood = true;//线段的跟踪好坏
            //cout << "spx=\n" << tracked_line->spx << endl;
            //cout << "epx=\n" << tracked_line->epx << endl;
        } else
        {
            tracked_line->trackedGood = false;
        }
    }

    // 判断跟踪效果好不好（根据整张图像的线段）
    double prev_length_all;//上一帧线段总长
    prev_length_all = (isLeft ? prev_frame->left_length_all : prev_frame->right_length_all);

    double curr_length_all = 0;//当前线段总长
    for (auto line:tracked_lines)
        curr_length_all += line.length;

    bool c1 = curr_length_all > Config::minLengthAll();///线段总长 > 阈值
    bool c2 = tracked_lines.size() > Config::minLinesNum();///线段总数 > 阈值
    bool c3 = ( ( curr_length_all / prev_length_all ) > Config::minLinesNumTrack() ) || (tracked_frame_count < 3);///帧间跟踪的线段总长下降(跳过检查关键帧)
    bool c4 = tracked_frame_count < Config::maxTrackCount();///连续跟踪的帧数
    // cout << "c1=" << (c1 ? "true" : "false") << endl;
    // cout << "c2=" << (c2 ? "true" : "false") << endl;
    // cout << "c3=" << (c3 ? "true" : "false") << endl;
    // cout << "c4=" << (c4 ? "true" : "false") << endl;
    //cout << "tracked_frame_count=" << tracked_frame_count << endl;

    trackGood =  (c1 && c2 && c3 && c4);//整个帧的跟踪好坏

    if (isLeft)
        left_length_all = curr_length_all;
    else
        right_length_all = curr_length_all;

    // if(isLeft)
    // {
    //     Mat image;
    //     cvtColor(img,image,CV_GRAY2BGR);
    //     for (auto line = tracked_lines.begin(); line != tracked_lines.end(); line++)///遍历线段
    //     {
    //         cv::line(image,
    //                     Point(line->spx[0],line->spx[1]),
    //                     Point(line->epx[0],line->epx[1]),
    //                     Scalar(0,0,200),1,CV_AA);///RGB颜色、粗细、抗锯齿
    //     }
    //     string title = "after_left";
    //     imshow(title, image);
    //     cv::waitKey(1000);
    // }

}

//直接跟踪的线段是一一对应的，这个match函数用来计算线段的3D位姿等信息，来更新stereo_ls
void StereoFrame::matchDirectStereoLines( vector<LineFeature*> &prev_stereo_ls, list<Line> &direct_lines_left, list<Line> &direct_lines_right )
{
    stereo_ls.clear();//清空当前帧的stereo_ls（本来就是空的）
    if (!Config::hasLines())
        return;

    //删除跟踪失败的线段
    auto prev_line = prev_stereo_ls.begin();
    list<Line>::iterator line_left = direct_lines_left.begin();
    list<Line>::iterator line_right = direct_lines_right.begin();
    while(prev_line != prev_stereo_ls.end())
    {
        if( !line_left->trackedGood || !line_right->trackedGood )
        {
            //cout << "false" << endl;
            prev_line = prev_stereo_ls.erase(prev_line);        //删除上一帧3D线段
            line_left = direct_lines_left.erase(line_left);     //删除当前帧左图线段
            line_right = direct_lines_right.erase(line_right);  //删除当前帧有图线段
            //erase之后不用++
            continue;
        }
        //左边2D线段
        Vector3d sp_l; sp_l << line_left->spx[0], line_left->spx[1], 1.0;///左起始点像素位置
        Vector3d ep_l; ep_l << line_left->epx[0], line_left->epx[1], 1.0;///左终止点像素位置
        Vector3d le_l; le_l << sp_l.cross(ep_l);// 起始点和终止点的叉积
        le_l = le_l / std::sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );///对叉积取模
        //右边2D线段
        Vector3d sp_r; sp_r << line_right->spx[0], line_right->spx[1], 1.0;///右边起始点像素位置
        Vector3d ep_r; ep_r << line_right->epx[0], line_right->epx[1], 1.0;///右边终止点像素位置
        Vector3d le_r; le_r << sp_r.cross(ep_r);

        //cout << "spl=" << sp_l(0) << " " << sp_l(1) << endl;
        //cout << "epl=" << ep_l(0) << " " << sp_l(1) << endl;
        // cout << "spr=" << sp_r(0) << " " << sp_r(1) << endl;
        // cout << "epr=" << ep_r(0) << " " << sp_r(1) << endl;

        double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );/// 双目线段y坐标重叠的质量

        double disp_s, disp_e;
        sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
        ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
        filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );///计算起点x的差和终点x的差（sp_l.head(2)取前2的元素）

        // 检查最小差异
        if( disp_s >= Config::minDisp() && disp_e >= Config::minDisp()///x视差不能太小（避免无穷远）
            && std::abs( sp_l(1)-ep_l(1) ) > Config::lineHorizTh()///避免检测到水平线
            && std::abs( sp_r(1)-ep_r(1) ) > Config::lineHorizTh()
            && overlap > Config::stereoOverlapTh() )///y的重叠阈值
        {// 计算线的3维位姿
            /// 用起点和终点两个点计算的
            Vector3d sP_; sP_ = cam->backProjection( sp_l(0), sp_l(1), disp_s);/// 左图起点2D -> 3D (输入spx,spy，spx的视差)
            Vector3d eP_; eP_ = cam->backProjection( ep_l(0), ep_l(1), disp_e);/// 左图终点2D -> 3D
            double angle_l = atan2( ( line_left->epx[1] - line_left->spx[1] ), ( line_left->epx[0] - line_left->spx[0] ) );
            //更新stereo_ls
            stereo_ls.push_back( new LineFeature(Vector2d(line_left->spx[0],line_left->spx[1]),     //spl
                                                 Vector2d(line_right->spx[0],line_right->spx[1]),   //spr
                                                 disp_s,sP_,
                                                 Vector2d(line_left->epx[0],line_left->epx[1]),     //epl
                                                 Vector2d(line_right->epx[0],line_right->epx[1]),   //epr
                                                 disp_e,eP_,
                                                 le_l,angle_l,-1,-2) );//id统一为-1，在后面修改。octave设置为-2，因为跟踪时没有图像金字塔层数
            ++prev_line;
            ++line_left;
            ++line_right;
            //cout << "true" << endl;
        }else
        {
            prev_line = prev_stereo_ls.erase(prev_line);        //删除上一帧3D线段
            line_left = direct_lines_left.erase(line_left);     //删除当前帧左图线段
            line_right = direct_lines_right.erase(line_right);  //删除当前帧有图线段
            //cout << "true->false" << endl;
        }
    }

    // Mat image;
    // cvtColor(img_r,image,CV_GRAY2BGR);
    // for (auto line = stereo_ls.begin(); line != stereo_ls.end(); line++)///遍历线段
    // {
    //     auto aline = **line;
    //     cv::line(image,
    //                 Point(aline.spr[0],aline.spr[1]),
    //                 Point(aline.epr[0],aline.epr[1]),
    //                 Scalar(0,0,200),1,CV_AA);///RGB颜色、粗细、抗锯齿
    // }
    // string title = "matched_left";
    // imshow(title, image);
    // cv::waitKey(2000);
}

}
