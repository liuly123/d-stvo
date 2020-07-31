#include <config.h>

//STL
#include <iostream>

//Boost
#include <boost/filesystem.hpp>

//YAML
#include <yaml-cpp/yaml.h>

using namespace std;

Config::Config()
{

    // 选取关键帧的决策参数（SLAM）
    min_entropy_ratio     = 0.85;
    max_kf_t_dist         = 5.0;
    max_kf_r_dist         = 15.0;

    // StVO-PL选项（bool型的标志）
    // -----------------------------------------------------------------------------------------------------
    has_points         = true;      // true if using points
    has_lines          = true;      // true if using line segments
    use_fld_lines      = false;     // true if using FLD detector
    lr_in_parallel     = true;      // true if detecting and matching features in parallel          并行提取和匹配特征
    pl_in_parallel     = true;      // true if detecting points and line segments in parallel       并行提取点和线段
    best_lr_matches    = true;      // true if double-checking the matches between the two images   重复检查两个图像之间的匹配
    adaptative_fast    = true;      // true if using adaptative fast_threshold                      使用自适应fast阈值
    use_motion_model   = false;     // true if using constant motion model                          恒定运动模型

    // Tracking参数
    // -----------------------------------------------------------------------------------------------------
    // Point特征
    max_dist_epip     = 1.0;        // max. epipolar distance in pixels                     极线以像素为单位的最大距离
    min_disp          = 1.0;        // min. disparity (avoid points in the infinite)        最小视差（避免无穷远点）（没有视差时计算出的距离就是无穷远）
    min_ratio_12_p    = 0.9;        // min. ratio between the first and second best matches 第一和第二最佳匹配之间的最小比率

    // 线段特征
    line_sim_th       = 0.75;       // threshold for cosine similarity                   余弦相似阈值
    stereo_overlap_th = 0.75;
    f2f_overlap_th    = 0.75;
    min_line_length   = 0.025;      // min. line length (relative to img size)           最小线长度（相对于img尺寸）
    line_horiz_th     = 0.1;        // parameter to avoid horizontal lines (pixels)      避免水平线（像素）的参数
    min_ratio_12_l    = 0.9;        // parameter to avoid outliers in line matching      线匹配中避免异常值的参数
    ls_min_disp_ratio = 0.7;        // min ratio between min(disp)/max(disp) for a LS    LS最小（disp）/最大（disp）之间的最小比率（disp：像素距离）

    // 自适应FAST参数
    fast_min_th       = 5;          // min. value for FAST threshold                                       FAST阈值的最小值
    fast_max_th       = 50;         // max. value for FAST threshold                                       FAST阈值的最大值
    fast_inc_th       = 5;          // base increment for the FAST threshold                               FAST阈值的基础增量(如果特征数量少，就增加)
    fast_feat_th      = 50;         // base number of features to increase/decrease FAST threshold         增加/减少FAST阈值的特征基数
    fast_err_th       = 0.5;        // threshold for the optimization error                                优化误差的阈值

    // 优化参数
    // -----------------------------------------------------------------------------------------------------
    homog_th         = 1e-7;        // avoid points in the infinite                                        避开无穷远处的点
    min_features     = 10;          // min. number of features to perform StVO                             执行StVO的最小功能数
    max_iters        = 5;           // max. number of iterations in the first stage of the optimization    优化第一阶段的最大迭代次数
    max_iters_ref    = 10;          // max. number of iterations in the refinement stage                   精细优化阶段的最大迭代次数
    min_error        = 1e-7;        // min. error to stop the optimization                                 停止优化的最小误差
    min_error_change = 1e-7;        // min. error change to stop the optimization                          停止优化的最小错误变化
    inlier_k         = 4.0;         // factor to discard outliers before the refinement stage              在细化阶段之前丢弃异常值的因子

    // 特征提取参数
    // -----------------------------------------------------------------------------------------------------
    matching_strategy = 0;          // 0 - pure descriptor based  |  1 - window based plus descriptor                      0-纯描述符| 1-基于窗口的描述符+
    matching_s_ws     = 10;         // size of the windows (in pixels) to look for stereo matches (if matching_stereo=1)  查找双目匹配的窗口大小（像素）（如果matching_stereo=1）
    matching_f2f_ws   = 3;          // size of the windows (in pixels) to look for f2f matches                            查找f2f（帧间）匹配项的窗口大小（像素）

    // ORB特征提取
    orb_nfeatures     = 1200;       // number of ORB features to detect                                             要检测的ORB特征数
    orb_scale_factor  = 1.2;        // pyramid decimation ratio for the ORB detector                                FAST检测器的金字塔抽取比
    orb_nlevels       = 4;          // number of pyramid levels                                                     金字塔层数
    orb_edge_th       = 19;         // size of the border where the features are not detected                       未检测到特征的边框大小
    orb_wta_k         = 2;          // number of points that produce each element of the oriented BRIEF descriptor  生成oriented BRIEF描述符的每个元素的点数
    orb_score         = 1;          // 0 - HARRIS  |  1 - FAST
    orb_patch_size    = 31;         // size of the patch used by the oriented BRIEF descriptor.                     oriented BRIEF描述符使用的patch的大小
    orb_fast_th       = 20;         // default FAST threshold
    // LSD参数
    lsd_nfeatures     = 300;        // number of LSD lines detected (set to 0 if keeping all lines)                 最多检测的LSD线数（如果保留所有线，则设置为0）
    lsd_refine        = 0;          // the way of refining or not the detected lines                                检测的线是否细化
    lsd_scale         = 1.2;        // scale of the image that will be used to find the lines                       用于检测线条的图像比例
    lsd_sigma_scale   = 0.6;        // sigma for Gaussian filter                                                    高斯滤波器的sigma
    lsd_quant         = 2.0;        // bound to the quantization error on the gradient norm                         关于梯度范数的量化误差
    lsd_ang_th        = 22.5;       // gradient angle tolerance in degrees                                          坡度角公差（度）
    lsd_log_eps       = 1.0;        // detection threshold (only for advanced refinement)                           检测阈值（仅用于高级优化）
    lsd_density_th    = 0.6;        // minimal density of aligned region points in the enclosing rectangle          包围矩形中对齐区域点的最小密度
    lsd_n_bins        = 1024;       // number of bins in pseudo-ordering of gradient modulus                        梯度模量伪序中的位数
}

Config::~Config(){}

Config& Config::getInstance()
{
    static Config instance; /// 首次使用时实例化并保证被销毁（用的static）
    return instance;
}

template<typename T>
inline T loadSafe(const YAML::Node &config, std::string param, T default_value = T()) {

    if (YAML::Node parameter = config[param])
        return parameter.as<T>();
    else
        return default_value;
}

void Config::loadFromFile( const string &config_file )
{

    if (!boost::filesystem::exists(config_file) || !boost::filesystem::is_regular(config_file)) {
        cout << "[Config->loadFromFile] Invalid config file, keeping default params..." << endl;
        return;
    }

    YAML::Node config = YAML::LoadFile(config_file);

    Config::minEntropyRatio() = loadSafe(config, "min_entropy_ratio", Config::minEntropyRatio());
    Config::maxKFTDist() = loadSafe(config, "max_kf_t_dist", Config::maxKFTDist());
    Config::maxKFRDist() = loadSafe(config, "max_kf_r_dist", Config::maxKFRDist());

    Config::hasPoints() = loadSafe(config, "has_points", Config::hasPoints());
    Config::hasLines() = loadSafe(config, "has_lines", Config::hasLines());
    Config::useFLDLines() = loadSafe(config, "use_fld_lines", Config::useFLDLines());
    Config::lrInParallel() = loadSafe(config, "lr_in_parallel", Config::lrInParallel());
    Config::plInParallel() = loadSafe(config, "pl_in_parallel", Config::plInParallel());
    Config::bestLRMatches() = loadSafe(config, "best_lr_matches", Config::bestLRMatches());
    Config::adaptativeFAST() = loadSafe(config, "adaptative_fast", Config::adaptativeFAST());
    Config::useMotionModel() = loadSafe(config, "use_motion_model", Config::useMotionModel());

    Config::maxDistEpip() = loadSafe(config, "max_dist_epip", Config::maxDistEpip());
    Config::minDisp() = loadSafe(config, "min_disp", Config::minDisp());
    Config::minRatio12P() = loadSafe(config, "min_ratio_12_p", Config::minRatio12P());

    Config::lineSimTh() = loadSafe(config, "line_sim_th", Config::lineSimTh());
    Config::stereoOverlapTh() = loadSafe(config, "stereo_overlap_th", Config::stereoOverlapTh());
    Config::f2fOverlapTh() = loadSafe(config, "f2f_overlap_th", Config::f2fOverlapTh());
    Config::minLineLength() = loadSafe(config, "min_line_length", Config::minLineLength());
    Config::lineHorizTh() = loadSafe(config, "line_horiz_th", Config::lineHorizTh());
    Config::minRatio12L() = loadSafe(config, "min_ratio_12_l", Config::minRatio12L());
    Config::lsMinDispRatio() = loadSafe(config, "ls_min_disp_ratio", Config::lsMinDispRatio());

    Config::fastMinTh() = loadSafe(config, "fast_min_th", Config::fastMinTh());
    Config::fastMaxTh() = loadSafe(config, "fast_max_th", Config::fastMaxTh());
    Config::fastIncTh() = loadSafe(config, "fast_inc_th", Config::fastIncTh());
    Config::fastFeatTh() = loadSafe(config, "fast_feat_th", Config::fastFeatTh());
    Config::fastErrTh() = loadSafe(config, "fast_err_th", Config::fastErrTh());

    Config::rgbdMinDepth() = loadSafe(config, "rgbd_min_depth", Config::rgbdMinDepth());
    Config::rgbdMaxDepth() = loadSafe(config, "rgbd_max_depth", Config::rgbdMaxDepth());

    Config::homogTh() = loadSafe(config, "homog_th", Config::homogTh());
    Config::minFeatures() = loadSafe(config, "min_features", Config::minFeatures());
    Config::maxIters() = loadSafe(config, "max_iters", Config::maxIters());
    Config::maxItersRef() = loadSafe(config, "max_iters_ref", Config::maxItersRef());
    Config::minError() = loadSafe(config, "min_error", Config::minError());
    Config::minErrorChange() = loadSafe(config, "min_error_change", Config::minErrorChange());
    Config::inlierK() = loadSafe(config, "inlier_k", Config::inlierK());

    Config::matchingStrategy() = loadSafe(config, "matching_strategy", Config::matchingStrategy());
    Config::matchingSWs() = loadSafe(config, "matching_s_ws", Config::matchingSWs());
    Config::matchingF2FWs() = loadSafe(config, "matching_f2f_ws", Config::matchingF2FWs());

    Config::orbNFeatures() = loadSafe(config, "orb_nfeatures", Config::orbNFeatures());
    Config::orbScaleFactor() = loadSafe(config, "orb_scale_factor", Config::orbScaleFactor());
    Config::orbNLevels() = loadSafe(config, "orb_nlevels", Config::orbNLevels());
    Config::orbEdgeTh() = loadSafe(config, "orb_edge_th", Config::orbEdgeTh());
    Config::orbWtaK() = loadSafe(config, "orb_wta_k", Config::orbWtaK());
    Config::orbScore() = loadSafe(config, "orb_score", Config::orbScore());
    Config::orbPatchSize() = loadSafe(config, "orb_patch_size", Config::orbPatchSize());
    Config::orbFastTh() = loadSafe(config, "orb_fast_th", Config::orbFastTh());

    Config::lsdNFeatures() = loadSafe(config, "lsd_nfeatures", Config::lsdNFeatures());
    Config::lsdRefine() = loadSafe(config, "lsd_refine", Config::lsdRefine());
    Config::lsdScale() = loadSafe(config, "lsd_scale", Config::lsdScale());
    Config::lsdSigmaScale() = loadSafe(config, "lsd_sigma_scale", Config::lsdSigmaScale());
    Config::lsdQuant() = loadSafe(config, "lsd_quant", Config::lsdQuant());
    Config::lsdAngTh() = loadSafe(config, "lsd_ang_th", Config::lsdAngTh());
    Config::lsdLogEps() = loadSafe(config, "lsd_log_eps", Config::lsdLogEps());
    Config::lsdDensityTh() = loadSafe(config, "lsd_density_th", Config::lsdDensityTh());
    Config::lsdNBins() = loadSafe(config, "lsd_n_bins", Config::lsdNBins());
}
