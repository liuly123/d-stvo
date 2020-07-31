#include <pinholeStereoCamera.h>

#include <cmath>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

// 实例化（从yaml读取参数）
PinholeStereoCamera::PinholeStereoCamera(const string &params_file) {

    if  (!boost::filesystem::exists(params_file) || !boost::filesystem::is_regular(params_file))
        throw std::runtime_error("[PinholeSteroCamera] Invalid parameters file");

    YAML::Node dset_config = YAML::LoadFile(params_file);
    YAML::Node cam_config = dset_config["cam0"];///读取yaml文件中cam0项下面的参数

    width = cam_config["cam_width"].as<double>();
    height = cam_config["cam_height"].as<double>();
    b = cam_config["cam_bl"].as<double>();

    string camera_model = cam_config["cam_model"].as<string>();
    if( camera_model == "Pinhole" )/// 如果是EuRoC或Falcon数据集的yaml文件
    {
    // 如果是Falcon数据集
        if( cam_config["Kl"].IsDefined() )
        {
            dist = true;
            vector<double> Kl_ = cam_config["Kl"].as<vector<double>>();
            vector<double> Kr_ = cam_config["Kr"].as<vector<double>>();
            vector<double> Dl_ = cam_config["Dl"].as<vector<double>>();
            vector<double> Dr_ = cam_config["Dr"].as<vector<double>>();
            Kl = ( Mat_<float>(3,3) << Kl_[0], 0.0, Kl_[2], 0.0, Kl_[1], Kl_[3], 0.0, 0.0, 1.0 );
            Kr = ( Mat_<float>(3,3) << Kr_[0], 0.0, Kr_[2], 0.0, Kr_[1], Kr_[3], 0.0, 0.0, 1.0 );
            /// 加载旋转和平移
            vector<double> R_ = cam_config["R"].as<vector<double>>();
            vector<double> t_ = cam_config["t"].as<vector<double>>();
            R = Mat::eye(3,3,CV_64F);
            t = Mat::eye(3,1,CV_64F);
            int k = 0;
            for( int i = 0; i < 3; i++ )
            {
                t.at<double>(i,0) = t_[i];
                for( int j = 0; j < 3; j++, k++ )
                    R.at<double>(i,j) = R_[k];
            }
            /// 加载畸变参数
            int Nd = Dl_.size();
            Dl = Mat::eye(1,Nd,CV_64F);
            Dr = Mat::eye(1,Nd,CV_64F);
            for( int i = 0; i < Nd; i++ )
            {
                Dl.at<double>(0,i) = Dl_[i];
                Dr.at<double>(0,i) = Dr_[i];
            }
            /// if dtype is equidistant (now it is default)，initialize undistort rectify map OpenCV
            /// 如果dtype是等距的（现在是默认值），就初始化畸变矫正
            if(cam_config["dtype"].IsDefined())
            {
                stereoRectify( Kl, Dl, Kr, Dr, cv::Size(width,height), R, t, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0 );
                cv::fisheye::initUndistortRectifyMap( Kl, Dl, Rl, Pl, cv::Size(width,height), CV_16SC2, undistmap1l, undistmap2l );
                cv::fisheye::initUndistortRectifyMap( Kr, Dr, Rr, Pr, cv::Size(width,height), CV_16SC2, undistmap1r, undistmap2r );
            }
            else
            {
                stereoRectify( Kl, Dl, Kr, Dr, cv::Size(width,height), R, t, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0 );
                initUndistortRectifyMap( Kl, Dl, Rl, Pl, cv::Size(width,height), CV_16SC2, undistmap1l, undistmap2l );
                initUndistortRectifyMap( Kr, Dr, Rr, Pr, cv::Size(width,height), CV_16SC2, undistmap1r, undistmap2r );
            }

            fx = Pl.at<double>(0,0);
            fy = Pl.at<double>(1,1);
            cx = Pl.at<double>(0,2);
            cy = Pl.at<double>(1,2);

            K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
        } else {
            // 如果是EuRoC数据集
            fx = std::abs(cam_config["cam_fx"].as<double>());
            fy = std::abs(cam_config["cam_fy"].as<double>());
            cx = cam_config["cam_cx"].as<double>();
            cy = cam_config["cam_cy"].as<double>();

            double d0 = cam_config["cam_d0"].as<double>(),
                    d1 = cam_config["cam_d1"].as<double>(),
                    d2 = cam_config["cam_d2"].as<double>(),
                    d3 = cam_config["cam_d3"].as<double>();

            dist = (d0 != 0.0 );///d0不为0，dist为true，就矫正畸变
            d   << d0, d1, d2, d3, 0.0;
            Kl = ( Mat_<float>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 );
            Dl = ( Mat_<float>(1,5) << d(0), d(1), d(2), d(3), d(4) );
            Pl = ( Mat_<float>(3,4) << fx, 0.0, cx, 0.0,   0.0, fx, cy, 0.0,   0.0, 0.0, 1.0, 0.0 );
            K    << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

            /// initialize undistort rectify map OpenCV
            /// 畸变矫正的初始化
            initUndistortRectifyMap( Kl, Dl, cv::Mat_<double>::eye(3,3), Pl, cv::Size(width,height), CV_16SC2, undistmap1l, undistmap2l );
            undistmap1r = undistmap1l;
            undistmap2r = undistmap2l;
        }
    }
    else
        throw std::runtime_error("[PinholeStereoCamera] Invalid cam_model");
}

PinholeStereoCamera::~PinholeStereoCamera() {

}

void PinholeStereoCamera::rectifyImage( const Mat& img_src, Mat& img_rec) const
{
    if(dist)
      remap( img_src, img_rec, undistmap1l, undistmap2l, cv::INTER_LINEAR);
    else
      img_src.copyTo(img_rec);
}

void PinholeStereoCamera::rectifyImagesLR( const Mat& img_src_l, Mat& img_rec_l, const Mat& img_src_r, Mat& img_rec_r ) const
{
    if(dist)
    {
        remap( img_src_l, img_rec_l, undistmap1l, undistmap2l, cv::INTER_LINEAR);
        remap( img_src_r, img_rec_r, undistmap1r, undistmap2r, cv::INTER_LINEAR);
    }
    else
    {
        img_src_l.copyTo(img_rec_l);
        img_src_r.copyTo(img_rec_r);
    }
}

// 投影和反投影
///（由于线段的关系，我们应该在内部处理校正后的图像）

Vector3d PinholeStereoCamera::backProjection( const double &u, const double &v, const double &disp )
{
    Vector3d P;
    double bd = b/disp;
    P(0) = bd*(u-cx);
    P(1) = bd*(v-cy);
    P(2) = bd*fx;
    return P;
}

Vector2d PinholeStereoCamera::projection(const Vector3d &P )
{
    Vector2d uv_unit;
    uv_unit(0) = cx + fx * P(0) / P(2);
    uv_unit(1) = cy + fy * P(1) / P(2);
    return uv_unit;
}
