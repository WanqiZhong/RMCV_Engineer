#include "calculate.hpp"

Calculator::Calculator(){};
Calculator::~Calculator(){};

void Calculator::Run()
{
    logger.info("Calculator Run");
    Calculator_thread = thread(&Calculator::Calculate_Run,this);
}

void Calculator::Join()
{
    logger.info("Waiting for [Calculator]");
    Calculator_thread.join();
    logger.sinfo("[Calculator] joined");
}

void Calculator::Calculate_Run()
{
    CalculateInit();
    while(1)
    {
        CalculateMinePnp();
        // this_thread::sleep_for(chrono::milliseconds(10));
    }
}




void Calculator::CalculateInit()
{
    #ifndef OFFICIAL
    toml::value constants = toml::parse(param.constants_path);
    toml_to_matrix(constants.at("camera").at("F"), F);
    toml_to_matrix(constants.at("camera").at("C"), C);
    toml_to_matrix(constants.at("cam2pitch").at("R"), cam2pitch_R);
    toml_to_matrix(constants.at("cam2pitch").at("T"), cam2pitch_T);
    toml_to_matrix(constants.at("pitch2yaw").at("T"), pitch2yaw_T);
    cam2pitch.block<3,3>(0,0) = cam2pitch_R;
    cam2pitch.block<3,1>(0,3) = cam2pitch_T;
    cam2pitch.block<1,4>(3,0) << 0., 0., 0., 1.;
    if(is_WM){
        Eigen::Matrix3d trans_to_1024;
        trans_to_1024 << 0.625, 0, 120,
                        0, 0.625, 120,
                        0, 0, 1;
        F = trans_to_1024 * F;
    }
    cv::eigen2cv(F,CameraMatrix);
    cv::eigen2cv(C,DistCoeffs);
    #else
    CameraMatrix = (Mat_<double>(3,3) << 1273.5096931699643, 0.0, 281.6342455704224 , 
                                            0.0, 1274.412923337173, 356.51342207682484,
                                        0.0, 0.0, 1.0);
    DistCoeffs = (Mat_<double>(1,5) << -0.22375498735597868, 0.28173237030830756, 0.0023061024316095753, -0.002034056774360411, -2.3013327557759515);
    #endif
}

void Calculator::CalculateMinePnp()
{
    vector<Point3f> Mine3D;
    vector<Point2f> Mine2D;
    Mine3D.push_back(Point3f(HALF_LENGTH,HALF_LENGTH,0));
    Mine3D.push_back(Point3f(-HALF_LENGTH,HALF_LENGTH,0));
    Mine3D.push_back(Point3f(-HALF_LENGTH,-HALF_LENGTH,0));
    Mine3D.push_back(Point3f(HALF_LENGTH,-HALF_LENGTH,0));
    Mat rvec = Mat::zeros(3,1,CV_64FC1);
    Mat tvec = Mat::zeros(3,1,CV_64FC1);
    Mat_<float> rotMat(3, 3);
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 1> T;

    for(int i = 0; i < MineCorner.size(); i++)
    {
        for(int j = 0; j < 4; j++)
        {
            Mine2D[j] = MineCorner[i][j];
        }
        solvePnP(Mine3D,Mine2D,CameraMatrix,DistCoeffs,rvec,tvec);
        Rodrigues(rvec, rotMat);  
        //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵
        cv::cv2eigen(rotMat, R);
        cv::cv2eigen(tvec, T);
        Eigen::Vector3d eulerAngle = R.eulerAngles(2, 1, 0);
        cout << "rvec: " << rvec << endl;
        cout << "tvec: " << tvec << endl;
        cout << "eulerAngle: " << eulerAngle << endl;
    }
    // for(int i=0;i<5;i++)
    // {
    //     //Pc=R*Po+T
    //     Eigen::Matrix<double,3,1> p0;
    //     p0<<pw_result[i][0],pw_result[i][1],pw_result[i][2];
    //     pb.push_back( pc_to_pb(R*p0+T) );
    // }
    // pb.push_back( pc_to_pb(T) );
    // return pb;
}
#endif