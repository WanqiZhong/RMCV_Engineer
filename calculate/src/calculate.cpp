#include "calculate.hpp"

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
    umt::Subscriber<MINE_POSITION_MSG> mine_sub("anchor_point_data");
    umt::Publisher<ANGLE_DATA_MSG> angle_pub("robot_data");
    while(param.get_run_mode()!=HALT)
    {
        anchor_point.clear();
        try{
            MINE_POSITION_MSG msg = mine_sub.pop();
            if(msg.goal.empty()){
                logger.warn("No anchor_point, can't solve pnp.");
                continue;
            }
            anchor_point = msg.goal;
            logger.info("Reiceve anchor_point:[{},{}],[{},{}],[{},{}],[{},{}]",anchor_point[0][0].x, \
            anchor_point[0][0].y,anchor_point[0][1].x,anchor_point[0][1].y,anchor_point[0][2].x,anchor_point[0][2].y, \
            anchor_point[0][3].x,anchor_point[0][3].y);
        }
        catch(const HaltEvent&){
            cout<<"Catch HaltEvent"<<endl;
            break;
        }
        CalculatePnp();
        if(param.visual_status == 1){
            ANGLE_DATA_MSG angle_msg;
            angle_msg.is_valid = true;
            angle_msg.ratation_right = true;
            // change to float
            angle_msg.roll = float(ypr[2]);
            angle_msg.pitch = float(ypr[1]);
            angle_msg.yaw = float(ypr[0]);
            angle_msg.x = float(position[0]);
            angle_msg.y = float(position[1]);
            angle_msg.z = float(position[2]);
            last_angle_data_msg = angle_msg;
            logger.warn("angle_msg: x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}",angle_msg.x,angle_msg.y,angle_msg.z,\
            angle_msg.roll,angle_msg.pitch,angle_msg.yaw);
            last_angle_data_msg = angle_msg;
            angle_pub.push(angle_msg);
        }
        else if(param.visual_status == 2){
            last_angle_data_msg.ratation_right = false;
            logger.critical("Last one angle_msg: x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}",last_angle_data_msg.x,last_angle_data_msg.y,last_angle_data_msg.z,\
            last_angle_data_msg.roll,last_angle_data_msg.pitch,last_angle_data_msg.yaw);
            angle_pub.push(last_angle_data_msg);
        }

    }
}

void Calculator::CalculateInit()
{
//   Eigen::Matrix<double, 3, 3> F;
//   Eigen::Matrix<double, 1, 5> C;
//   toml::value constants = toml::parse(param.constants_path);
//   toml_to_matrix(constants.at("camera").at("F"), F);
//   toml_to_matrix(constants.at("camera").at("C"), C);
//   cv::eigen2cv(F,CameraMatrix);
//   cv::eigen2cv(C,DistCoeffs);
    // CameraMatrix = (Mat_<double>(3,3) << 1273.5096931699643, 0.0, 281.6342455704224 ,
    //                                     0.0, 1274.412923337173, 356.51342207682484,
    //                                     0.0, 0.0, 1.0);
    // DistCoeffs = (Mat_<double>(1,5) << -0.22375498735597868, 0.28173237030830756, 0.0023061024316095753, -0.002034056774360411, -2.3013327557759515);
    CameraMatrix = (Mat_<double>(3,3) << 590.057431, 0.000000, 628.940684,
                                        0.000000, 590.227016, 358.945741, 
                                        0.000000, 0.000000, 1.000000);
    DistCoeffs = (Mat_<double>(1,5) << 0.048251, -0.049567, -0.000578, -0.000505, 0.016714);
}

void Calculator::CalculatePnp()
{
    vector<Point3f> Mine3D;
    vector<Point2f> Mine2D;
    if(param.get_run_mode() == GoldMode){
        Mine3D.push_back(Point3f(HALF_LENGTH,HALF_LENGTH,0));
        Mine3D.push_back(Point3f(-HALF_LENGTH,HALF_LENGTH,0));
        Mine3D.push_back(Point3f(-HALF_LENGTH,-HALF_LENGTH,0));
        Mine3D.push_back(Point3f(HALF_LENGTH,-HALF_LENGTH,0));
    }
    else if(param.get_run_mode() == ExchangeSiteMode){
        Mine3D.push_back(Point3f(HALF_LENGTH,HALF_LENGTH,50));
        Mine3D.push_back(Point3f(HALF_LENGTH,-HALF_LENGTH,50));
        Mine3D.push_back(Point3f(-HALF_LENGTH,-HALF_LENGTH,50));
        Mine3D.push_back(Point3f(-HALF_LENGTH,HALF_LENGTH,50));
    }
    Mat rvec = Mat::zeros(3,1,CV_64FC1);
    Mat tvec = Mat::zeros(3,1,CV_64FC1);
    Mat rotMat = Mat::zeros(3,1,CV_64FC1);
    Mat cvPosition = Mat::zeros(3,1,CV_64FC1);
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 1> T;
    for(int i = 0; i < 1; ++i){
        for(int j = 0; j < anchor_point[i].size(); ++j){
            Mine2D.push_back(anchor_point[i][j]);
        }
        solvePnP(Mine3D,Mine2D,CameraMatrix,DistCoeffs,rvec,tvec);
        Rodrigues(rvec, rotMat);
        //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵
        cv::cv2eigen(rotMat, R);
        cv::cv2eigen(tvec, T);
        logger.info("revc x:{}, y:{}, z:{}",T(0),T(1),T(2));
        // Eigen::Vector3d eulerAngle = R.eulerAngles(2, 1, 0);
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));

        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        ypr = ypr / M_PI * 180.0;
        logger.info("roll:{}, pitch:{}, yaw:{}",ypr(0),ypr(1),ypr(2));

        // position =  final_R[view_type] * (final_Rvec * tvec + final_Tvec) * final_T[view_type];
        cvPosition = final_Rvec * tvec + final_Tvec;
        cv::cv2eigen(cvPosition, position);
        logger.info("position x:{}, y:{}, z:{}",position(0),position(1),position(2));

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
