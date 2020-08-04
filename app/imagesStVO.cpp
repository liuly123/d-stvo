#ifdef HAS_MRPT
#include <sceneRepresentation.h>
#endif

#include <stereoFrame.h>
#include <stereoFrameHandler.h>
#include <boost/filesystem.hpp>
#include <sstream>

#include "dataset.h"
#include "timer.h"

#define RESET   "\033[0m"
#define RED     "\033[32m"/* Green */

using namespace StVO;

void showHelp();
bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file);
string DoubleToString(const double value, unsigned int precision);

int main(int argc, char **argv)
{

    // 读取命令行参数
    string dataset_name, config_file;
    int frame_offset = 0, frame_number = 0, frame_step = 1;
    if (!getInputArgs(argc, argv, dataset_name, frame_offset, frame_number, frame_step, config_file)) {
        showHelp();
        return -1;
    }

    if (!config_file.empty()) Config::loadFromFile(config_file);

    // 从环境变量读取数据集路径
    boost::filesystem::path dataset_path(string( getenv("DATASETS_DIR")));
    if (!boost::filesystem::exists(dataset_path) || !boost::filesystem::is_directory(dataset_path)) {
        cout << "Check your DATASETS_DIR environment variable" << endl;
        return -1;
    }

    dataset_path /= dataset_name;
    if (!boost::filesystem::exists(dataset_path) || !boost::filesystem::is_directory(dataset_path)) {
        cout << "Invalid dataset path" << endl;
        return -1;
    }

    // 读取数据集参数文件
    string dataset_dir = dataset_path.string();
    PinholeStereoCamera*  cam_pin = new PinholeStereoCamera((dataset_path / "dataset_params.yaml").string());
    Dataset dataset(dataset_dir, *cam_pin, frame_offset, frame_number, frame_step);

    // 创建场景
    Matrix4d Tcw, T_inc = Matrix4d::Identity();
    Vector6d cov_eig;
    Matrix6d cov;
    Tcw = Matrix4d::Identity();
    Tcw << 1, 0, 0, 0,
           0, 0, 1, 0,
           0,-1, 0, 0,
           0, 0, 0, 1;

    #ifdef HAS_MRPT
    sceneRepresentation scene("../config/scene_config.ini");//读取场景配置文件
    scene.initializeScene(Tcw, false);
    #endif

    // 文本输出
    string fileName;//时间戳
    Matrix4d T_cur = Tcw;//当前位姿
    // 打开文本文件
    ofstream trajectoryFile("traj.txt");
    double fps = 0;
    int count = 0;

    Timer timer;

    // 初始化并运行PL-StVO
    int frame_counter = 0;
    double t1;
    StereoFrameHandler* StVO = new StereoFrameHandler(cam_pin);
    Mat img_l, img_r;
    while (dataset.nextFrame(img_l, img_r,fileName))
    {
        if( frame_counter == 0 ) // 初始化
            StVO->initialize(img_l,img_r,0);
        else // 运行
        {
            timer.start();
            if(StVO->isGoodFrame())
            {
                cout << "DirectFrame" << endl;
                StVO->insertStereoPairDirect( img_l, img_r, frame_counter );//直接跟踪法
            }
            else
            {
                
                #ifdef HAS_MRPT
                scene.updateFrequency = true;
                #endif
                cout << RED << "KeyFrame" << endl << RESET;
                StVO->insertStereoPair( img_l, img_r, frame_counter );//提取和匹配的方法
            }
            
            StVO->optimizePose();//求解位姿
            t1 = timer.stop();

            //
            T_inc   = StVO->curr_frame->DT;
            cov     = StVO->curr_frame->DT_cov;
            cov_eig = StVO->curr_frame->DT_cov_eig;

            // 更新场景
            #ifdef HAS_MRPT                                                                 //inliers少         ,matched多
            scene.setText(frame_counter,t1,StVO->n_inliers_pt,StVO->matched_pt.size(),StVO->n_inliers_ls,StVO->matched_ls.size());
            scene.setCov( cov );
            scene.setPose( T_inc );
            scene.setImage(StVO->curr_frame->plotStereoFrame());
            scene.updateScene(StVO->matched_pt, StVO->matched_ls);
            #endif

            // 命令行输出
            cout.setf(ios::fixed,ios::floatfield); cout.precision(8);
            cout << "Frame: " << frame_counter << "\tRes.: " << StVO->curr_frame->err_norm;
            cout.setf(ios::fixed,ios::floatfield); cout.precision(3);
            cout << " \t Proc. time: " << t1 << " ms\t ";
            if( Config::adaptativeFAST() )  cout << "\t FAST: "   << StVO->orb_fast_th;
            if( Config::hasPoints())        cout << "\t Points: " << StVO->matched_pt.size() << " (" << StVO->n_inliers_pt << ") " ;
            if( Config::hasLines() )        cout << "\t Lines:  " << StVO->matched_ls.size() << " (" << StVO->n_inliers_ls << ") " ;
            cout << endl;

            // 更新Frame
            StVO->updateFrame();
            T_cur = T_cur * T_inc;//更新当前位姿

            fps += 1000.f/t1;
            count ++;

        }

        //更新四元数
        Matrix3d Rot;//旋转矩阵
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                Rot(i,j) = T_cur(i,j);

        Quaterniond Qua(Rot);//旋转四元数

        //文件名除以1e9（文件名数值以ns表示时间，转换成s表示的）
        double fileNameDouble = atof(fileName.c_str());
        fileNameDouble /= 1e+9;

        // 更新文本(TUM格式)
        trajectoryFile << DoubleToString(fileNameDouble,19) << " "
                       << DoubleToString(T_cur(0,3),19) << " " << DoubleToString(T_cur(1,3),19) << " " << DoubleToString(T_cur(2,3),19) << " "
                       << DoubleToString(Qua.x(),19) << " " << DoubleToString(Qua.y(),19) << " " << DoubleToString(Qua.z(),19) << " " << DoubleToString(Qua.w(),19)
                       << endl;

        frame_counter++;
    }

    cout << "average speed:" << ( fps / count) << "fps" << endl;
    trajectoryFile.close();
    // 等待场景关闭
    #ifdef HAS_MRPT
    while( scene.isOpen() );
    #endif

    return 0;
}

void showHelp() {
    cout << endl << "Usage: ./imagesStVO <dataset_name> [options]" << endl
         << "Options:" << endl
         << "\t-c Config file" << endl
         << "\t-o Offset (number of frames to skip in the dataset directory" << endl
         << "\t-n Number of frames to process the sequence" << endl
         << "\t-s Parameter to skip s-1 frames (default 1)" << endl
         << endl;
}

bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file) {

    if( argc < 2 || argc > 10 || (argc % 2) == 1 )
        return false;

    dataset_name = argv[1];
    int nargs = argc/2 - 1;
    for( int i = 0; i < nargs; i++ )
    {
        int j = 2*i + 2;
        if( string(argv[j]) == "-o" )
            frame_offset = stoi(argv[j+1]);
        else if( string(argv[j]) == "-n" )
            frame_number = stoi(argv[j+1]);
        else if( string(argv[j]) == "-s" )
            frame_step = stoi(argv[j+1]);
        else if (string(argv[j]) == "-c")
            config_file = string(argv[j+1]);
        else
            return false;
    }

    return true;
}

std::string DoubleToString(const double value, unsigned int precision)
{
	std::ostringstream out;
    out << std::fixed;
	if(precision > 0)
        out.precision(precision);
	out << value;
    return out.str();
}