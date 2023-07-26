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
            anchor_point = msg.goal;
            ANGLE_DATA_MSG angle_msg;
            for(int i = 0; i < 4; ++i){
                angle_msg.anchor_x[i] = 0;
                angle_msg.anchor_y[i] = 0;
            }
            if(msg.goal.empty() || msg.goal[0].size() != 4){
                angle_msg.is_valid = false;
                if(!msg.goal.empty()){
                    for(int i = 0; i < msg.goal[0].size(); ++i){
                        angle_msg.anchor_x[i] = int(anchor_point[0][i].x);
                        angle_msg.anchor_y[i] = int(anchor_point[0][i].y);
                        logger.info("anchor_point_{}:[{},{}]",i,anchor_point[0][i].x,anchor_point[0][i].y);
                    }
                }
                angle_pub.push(angle_msg);
                logger.warn("No anchor_point, can't solve pnp.");
                continue;
            }else{
                bool unsafe_flag = false;
                for(int i = 0; i < msg.goal[0].size(); ++i){
                    if(anchor_point[0][i].x < 5 || anchor_point[0][i].x > param.frame_width - 5 || anchor_point[0][i].y < 5 || anchor_point[0][i].y > param.frame_height - 5 ){
                        anchor_point[0][i].x = 0;
                        anchor_point[0][i].y = 0;               
                        unsafe_flag = true;
                    }
                    angle_msg.anchor_x[i] = int(anchor_point[0][i].x);
                    angle_msg.anchor_y[i] = int(anchor_point[0][i].y);
                    logger.info("anchor_point_{}:[{},{}]",i,anchor_point[0][i].x,anchor_point[0][i].y);
                }
                if(unsafe_flag){
                    angle_msg.is_valid = false;
                    angle_pub.push(angle_msg);
                    logger.warn("No anchor_point, can't solve pnp.");
                    continue;
                }
            }
            logger.info("Reiceve anchor_point:[{},{}],[{},{}],[{},{}],[{},{}]",anchor_point[0][0].x, \
            anchor_point[0][0].y,anchor_point[0][1].x,anchor_point[0][1].y,anchor_point[0][2].x,anchor_point[0][2].y, \
            anchor_point[0][3].x,anchor_point[0][3].y);
        }
        catch(const HaltEvent&){
            cout<<"Catch HaltEvent"<<endl;
            break;
        }
        CalculatePnp();
        // if(param.visual_status == 1){
        
        ANGLE_DATA_MSG angle_msg;
        angle_msg.is_valid = true;
        // change to float

        for(int i = 0; i < 4; ++i){
            angle_msg.anchor_x[i] = int(anchor_point[0][i].x);
            angle_msg.anchor_y[i] = int(anchor_point[0][i].y);
        }
        angle_msg.roll = float(-eulerAngle2[0]);
        angle_msg.pitch = float(eulerAngle2[1]);
        angle_msg.yaw = float(-eulerAngle2[2]);
        angle_msg.x = float(position[0]);
        angle_msg.y = float(position[1]);
        angle_msg.z = float(position[2]);
        last_angle_data_msg = angle_msg;
        logger.warn("angle_msg: x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}",angle_msg.x,angle_msg.y,angle_msg.z,\
        angle_msg.roll,angle_msg.pitch,angle_msg.yaw);
        last_angle_data_msg = angle_msg;
        angle_pub.push(angle_msg);
        // }
        // else if(param.visual_status == 2){
        //     last_angle_data_msg.ratation_right = false;
        //     logger.critical("Last one angle_msg: x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}",last_angle_data_msg.x,last_angle_data_msg.y,last_angle_data_msg.z,\
        //     last_angle_data_msg.roll,last_angle_data_msg.pitch,last_angle_data_msg.yaw);
        //     angle_pub.push(last_angle_data_msg);
        // }
        // else{
        //     ANGLE_DATA_MSG angle_msg;
        //     angle_msg.is_valid = true;
        //     angle_msg.ratation_right = true;
        //     // change to float
        //     angle_msg.roll = 0;
        //     angle_msg.pitch = 0;
        //     angle_msg.yaw = 0;
        //     angle_msg.x = 0;
        //     angle_msg.y = 0;
        //     angle_msg.z = 0;
        //     logger.warn("Virtual angle_msg: x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}",angle_msg.x,angle_msg.y,angle_msg.z,\
        //     angle_msg.roll,angle_msg.pitch,angle_msg.yaw);
        //     angle_pub.push(angle_msg);
        // }

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
    // CameraMatrix = (Mat_<double>(3,3) << 590.057431, 0.000000, 628.940684,
    //                                     0.000000, 590.227016, 358.945741, 
    //                                     0.000000, 0.000000, 1.000000);
    // DistCoeffs = (Mat_<double>(1,5) << 0.048251, -0.049567, -0.000578, -0.000505, 0.016714);
    //647.4766049532548, 0.0, 772.8141822832324], [0.0, 647.3988049681597, 367.0779491855369], [0.0, 0.0, 1.0
    CameraMatrix = (Mat_<double>(3,3) << 648.4049021201001, 0.0, 772.7708597356725,
                                        0.0, 647.954255791574, 367.27133843225334, 
                                        0.0, 0.0, 1.0);
    DistCoeffs = (Mat_<double>(1,5) << -0.03900241043569671, 0.07901858897325609, 0.0012754337138536477, 0.0005857123595320316, -0.06278245602713038);
}




void Calculator::CalculatePnp()
{
    vector<Point3f> Mine3D;
    vector<Point2f> Mine2D;
    Mine3D.clear();
    // if(param.get_run_mode() == GoldMode){
    //     Mine3D.push_back(Point3f(HALF_LENGTH,HALF_LENGTH,0));
    //     Mine3D.push_back(Point3f(-HALF_LENGTH,HALF_LENGTH,0));
    //     Mine3D.push_back(Point3f(-HALF_LENGTH,-HALF_LENGTH,0));
    //     Mine3D.push_back(Point3f(HALF_LENGTH,-HALF_LENGTH,0));
    // }
    // else if(param.get_run_mode() == ExchangeSiteMode){
        Mine3D.push_back(Point3f(-HALF_LENGTH,-HALF_LENGTH,200));
        Mine3D.push_back(Point3f(-HALF_LENGTH,HALF_LENGTH,200));
        Mine3D.push_back(Point3f(HALF_LENGTH,HALF_LENGTH,200));
        Mine3D.push_back(Point3f(HALF_LENGTH,-HALF_LENGTH,200));
    // }
    Mat rvec = Mat::zeros(3,1,CV_64FC1);
    Mat tvec = Mat::zeros(3,1,CV_64FC1);
    Mat rotMat = Mat::zeros(3,1,CV_64FC1);
    Mat cvPosition = Mat::zeros(3,1,CV_64FC1);
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 1> T;
    for(int i = 0; i < 1; ++i){
        Mine2D.clear();
        for(int j = 0; j < anchor_point[i].size(); ++j){
            Mine2D.push_back(anchor_point[i][j]);
        }
        
        solvePnP(Mine3D,Mine2D,CameraMatrix,DistCoeffs,rvec,tvec);
        
        cv::cv2eigen(tvec, T);
        logger.info("tvec x:{}, y:{}, z:{}",T(0),T(1),T(2));

        rvec = final_Rvec_rpy * rvec;
        Rodrigues(rvec, rotMat);

        cv::cv2eigen(rotMat, R);

        Eigen::Vector3d eulerAngle = R.eulerAngles(2, 1, 0);  // R.eulerAngles(2:z, 1:y, 0:x)
        eulerAngle = eulerAngle / M_PI * 180.0;
        // logger.info("Eigen库 roll:{} pitch:{} yaw:{}",eulerAngle[2],eulerAngle[1],eulerAngle[0]);
    
        
        /* ====================================  */
        eulerAngle2 = rotationMatrixToEulerAngles(R);
        eulerAngle2 = eulerAngle2 / M_PI * 180.0;
        logger.info("roll:{} pitch:{} yaw:{}",eulerAngle2[0],eulerAngle2[1],eulerAngle2[2]);


        /* ====================================  */
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
        // logger.info("动轴ZYX(ypr) roll:{}, pitch:{}, yaw:{}",ypr(2),ypr(1),ypr(0));

        // position =  final_R[view_type] * (final_Rvec * tvec + final_Tvec) * final_T[view_type];
        cvPosition = final_Rvec * tvec + final_Tvec;
        cv::cv2eigen(cvPosition, position);
        // logger.info("position x:{}, y:{}, z:{}",position(0),position(1),position(2));

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



bool Calculator::isRotationMatirx(Eigen::Matrix3d R)
{
    double err=1e-6;
    Eigen::Matrix3d shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
    return (shouldIdenity - I).norm() < err;
}

Eigen::Vector3d Calculator::rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    assert(isRotationMatirx(R));
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}