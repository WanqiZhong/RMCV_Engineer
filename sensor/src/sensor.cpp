#include "sensor.hpp"


void Sensor::Run()
{
    logger.info("Sensor Run");
    Sensor_thread = thread(&Sensor::Sensor_Run,this);
}

void Sensor::Join()
{
    logger.info("Waiting for [Sensor]");
    Sensor_thread.join();
    logger.sinfo("[Sensor] Join");
}

void Sensor::Sensor_Run() {

    int index = 0;

    IMU_DATA_MSG ecu_data;
    pair<bool, IMU_DATA_MSG> ecu_data_try;
    umt::Publisher<cv::Mat> pub("channel0");
    umt::Subscriber<IMU_DATA_MSG> receive_sub("Electric_Data");

    for(int i=0; i < 4; i++){
        UVC uvc(cam_name[i].c_str());
        uvc.initUVC(300);
    }

    for(int i= 0; i < 4; i++){
        VideoCapture cap(cam_name[i], CAP_V4L);
        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(CAP_PROP_FPS, 30);
        cap.set(CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(CAP_PROP_FRAME_HEIGHT, 720);
        cap_set.push_back(cap);

    }


    namedWindow("operator_img", cv::WINDOW_NORMAL);
    resizeWindow("operator_img", 1200, 720);
    moveWindow("operator_img", 0, 0);

    while (mode != HALT)
    {
        logger.info("Vision Online");

        try {
            ecu_data_try = receive_sub.try_pop();
            if (ecu_data_try.first == true) {
                ecu_data = ecu_data_try.second;
            } else {
                ecu_data.cam_id = 1;
                ecu_data.mode = 1;
                ecu_data.position_id = 0;
            }
            logger.warn("ECU_DATA reiceve error!");
        } catch (exception e) {
            logger.warn("ECU_DATA reiceve error!");
        }


        logger.info("cam_id{}", ecu_data.cam_id);
        ecu_data.cam_id %= 4;

        operator_index = ecu_data.cam_id;

        if (ecu_data.mode == 0) {
            UVC uvc(cam_name[vision_index].c_str());
            uvc.initUVC(300);
            logger.info("Get Mine Mode");
        } else if (ecu_data.mode == 1) {
            UVC uvc(cam_name[vision_index].c_str());
            uvc.initUVC(25);
            logger.info("Exchange Mine Mode");
        }


        vision_cap = cap_set[vision_index];
        operator_cap = cap_set[operator_index];

        if (!vision_cap.isOpened()) {
            logger.error("Vision Camera is not opened");
            vision_img = Mat::zeros(480, 800, CV_8UC1);
            pub.push(vision_img.clone());
        } else {
            vision_cap >> vision_img;
            if (!vision_img.empty()) {
                imageRaw(index++, vision_img);
                imshow("vision_img", vision_img);
                waitKey(1);
            }
            pub.push(vision_img.clone());
        }

        if (!operator_cap.isOpened()) {
            logger.error("Operator Camera is not opened");
        } else {
            operator_cap >> operator_img;
            if (!operator_img.empty()) {
                imshow("operator_img", operator_img);
                waitKey(1);
            }
        }
    }
}

void Sensor::imageRaw(int index, Mat& img){
    auto now = std::chrono::system_clock::now();
    auto t_c = std::chrono::system_clock::to_time_t(now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&t_c));
    if(index % 10 == 0){
        imwrite("../raw/goldmine/"+std::string(timestamp)+ to_string(index) + ".jpg",img);
    }
}
