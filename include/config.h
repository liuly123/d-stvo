// 全局参数都存储在Config类中
/// 实例化函数Config();先设置默认的参数
/// 通过loadFromFile()成员函数从yaml文件加载参数（没有的就默认）
#pragma once
#include <string>

class Config
{

public:///全是public

    Config();
    ~Config();
    // loadFromFile通过loadSafe尝试加载yaml中的参数，加载失败就用默认值
    static void loadFromFile( const std::string &config_file );

    static Config& getInstance();

    // 关键帧选择参数（对于SLAM，如果有的话）
    static double&  minEntropyRatio()   { return getInstance().min_entropy_ratio; }
    static double&  maxKFTDist()        { return getInstance().max_kf_t_dist; }
    static double&  maxKFRDist()        { return getInstance().max_kf_r_dist; }

    // 一些标志
    static bool&    hasPoints()         { return getInstance().has_points; }
    static bool&    hasLines()          { return getInstance().has_lines; }
    static bool&    useFLDLines()       { return getInstance().use_fld_lines; }
    static bool&    lrInParallel()      { return getInstance().lr_in_parallel; }
    static bool&    plInParallel()      { return getInstance().pl_in_parallel; }
    static bool&    bestLRMatches()     { return getInstance().best_lr_matches; }
    static bool&    adaptativeFAST()    { return getInstance().adaptative_fast; }
    static bool&    useMotionModel()    { return getInstance().use_motion_model; }

    // 点检测与匹配
    static int&     matchingStrategy()  { return getInstance().matching_strategy; }
    static int&     matchingSWs()       { return getInstance().matching_s_ws; }
    static int&     matchingF2FWs()     { return getInstance().matching_f2f_ws; }


    static int&     orbNFeatures()      { return getInstance().orb_nfeatures; }
    static double&  orbScaleFactor()    { return getInstance().orb_scale_factor; }
    static int&     orbNLevels()        { return getInstance().orb_nlevels; }
    static int&     orbEdgeTh()         { return getInstance().orb_edge_th; }
    static int&     orbWtaK()           { return getInstance().orb_wta_k; }
    static int&     orbScore()          { return getInstance().orb_score; }
    static int&     orbPatchSize()      { return getInstance().orb_patch_size; }
    static int&     orbFastTh()         { return getInstance().orb_fast_th; }
    static int&     fastMinTh()         { return getInstance().fast_min_th; }
    static int&     fastMaxTh()         { return getInstance().fast_max_th; }
    static int&     fastIncTh()         { return getInstance().fast_inc_th; }
    static int&     fastFeatTh()        { return getInstance().fast_feat_th; }
    static double&  fastErrTh()         { return getInstance().fast_err_th; }
    static double&  maxDistEpip()       { return getInstance().max_dist_epip; }
    static double&  minDisp()           { return getInstance().min_disp; }
    static double&  minRatio12P()       { return getInstance().min_ratio_12_p; }

    static double&  rgbdMinDepth()      { return getInstance().rgbd_min_depth; }
    static double&  rgbdMaxDepth()      { return getInstance().rgbd_max_depth; }


    // 线的检测和匹配
    static int&     lsdNFeatures()      { return getInstance().lsd_nfeatures; }
    static int&     lsdRefine()         { return getInstance().lsd_refine; }
    static double&  lsdScale()          { return getInstance().lsd_scale; }
    static double&  lsdSigmaScale()     { return getInstance().lsd_sigma_scale; }
    static double&  lsdQuant()          { return getInstance().lsd_quant; }
    static double&  lsdAngTh()          { return getInstance().lsd_ang_th; }
    static double&  lsdLogEps()         { return getInstance().lsd_log_eps; }
    static double&  lsdDensityTh()      { return getInstance().lsd_density_th; }
    static int&     lsdNBins()          { return getInstance().lsd_n_bins; }
    static double&  lineHorizTh()       { return getInstance().line_horiz_th; }
    static double&  minLineLength()     { return getInstance().min_line_length; }
    static double&  minRatio12L()       { return getInstance().min_ratio_12_l; }
    static double&  stereoOverlapTh()   { return getInstance().stereo_overlap_th; }
    static double&  f2fOverlapTh()      { return getInstance().f2f_overlap_th; }
    static double&  lineSimTh()         { return getInstance().line_sim_th; }
    static double&  lsMinDispRatio()    { return getInstance().ls_min_disp_ratio; }

    // 优化参数
    static double&  homogTh()           { return getInstance().homog_th; }
    static int&     minFeatures()       { return getInstance().min_features; }
    static int&     maxIters()          { return getInstance().max_iters; }
    static int&     maxItersRef()       { return getInstance().max_iters_ref; }
    static double&  minError()          { return getInstance().min_error; }
    static double&  minErrorChange()    { return getInstance().min_error_change; }
    static double&  inlierK()           { return getInstance().inlier_k; }

    /// SLAM参数（关键帧选择）
    double min_entropy_ratio;
    double max_kf_t_dist;
    double max_kf_r_dist;

    /// 一些VO的标志
    bool has_points;
    bool has_lines;
    bool lr_in_parallel;
    bool pl_in_parallel;
    bool best_lr_matches;
    bool adaptative_fast;
    bool use_fld_lines;
    bool use_motion_model;

    /// 点检测和匹配
    int matching_strategy;
    int matching_s_ws;
    int matching_f2f_ws;

    int    orb_nfeatures;
    double orb_scale_factor;
    int    orb_nlevels;
    int    orb_edge_th;
    int    orb_wta_k;
    int    orb_score;
    int    orb_patch_size;
    int    orb_fast_th;

    int    fast_min_th;
    int    fast_max_th;
    int    fast_inc_th;
    int    fast_feat_th;
    double fast_err_th;

    double max_dist_epip;
    double min_disp;
    double min_ratio_12_p;
    double stereo_overlap_th;
    double f2f_overlap_th;
    double line_sim_th;

    double rgbd_min_depth;
    double rgbd_max_depth;

    /// 线检测和匹配
    int    lsd_nfeatures;
    int    lsd_refine;
    double lsd_scale;
    double lsd_sigma_scale;
    double lsd_quant;
    double lsd_ang_th;
    double lsd_log_eps;
    double lsd_density_th;
    int    lsd_n_bins;
    double line_horiz_th;
    double min_line_length;
    double min_ratio_12_l;
    double ls_min_disp_ratio;

    // 优化参数
    double homog_th;
    int    min_features;
    int    max_iters;
    int    max_iters_ref;
    double min_error;
    double min_error_change;
    double inlier_k;

    // TODO: l-lk参数

    //关键帧判断
    int min_lines_num = 20;                  //关键帧中线段数量最小值
    double min_total_length = 10.0;         //关键帧中线段总长最小值
    //Sampling
    int patch_size = 1;                     //采样间隔
    //FitLine
    int min_samle_points = 4;               //最小采样点数
    double max_point_num_dec = 3.0;         //采样点数下降速率阈值（用于判断跟踪质量）
    //FilterLine
    double max_drop_length_avg = 30.0;      //最大垂足平均长度（用于滤除线段）
    double max_drop_dist_diff = 60.0;       //平均垂足间距离变化的最大阈值（用于滤除线段）
    double max_dist_per_point = 5.0;        //采样点间的间距最大值
    //RefineLine
    int Grow_px = 10;                       //线段生长的变化范围
    //NeedKeyframe
    double min_length_all = 500;            //frame中所有line的最短总长
    double min_lines_num_track = 0.9;       //最小跟踪成功率（按照线段总长度）
    int max_track_count = 50;


    //关键帧判断
    static int&  minLinesNum()          { return getInstance().min_lines_num; }
    static double&  minTotalLength()    { return getInstance().min_total_length; }
    //Sampling
    static int&  patchSize()            { return getInstance().patch_size; }
    //FitLine
    static int&  minSamlePoints()       { return getInstance().min_samle_points; }
    static double&  maxPointNumDec()    { return getInstance().max_point_num_dec; }
    //FilterLine
    static double&  maxDropLengthAvg()  { return getInstance().max_drop_length_avg; }
    static double&  maxDropDistDiff()   { return getInstance().max_drop_dist_diff; }
    static double&  maxDistPerPoint()   { return getInstance().max_dist_per_point; }
    //RefineLine
    static int&  growpx()               { return getInstance().Grow_px; }
    //NeedKeyframe
    static double&  minLengthAll()      { return getInstance().min_length_all; }
    static double&  minLinesNumTrack()  { return getInstance().min_lines_num_track; }
    static int&  maxTrackCount()           { return getInstance().max_track_count; }

};

