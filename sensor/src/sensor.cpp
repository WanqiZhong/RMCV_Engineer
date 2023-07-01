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


    for(auto num: param.cap_set){
        UVC uvc(param.cam_map.at(num).c_str());
        uvc.initUVC(param.exposure_time_mine);
        VideoCapture cap(param.cam_map.at(num), CAP_V4L);
        cap.set(CAP_PROP_FOURCC, param.codec);
        cap.set(CAP_PROP_FPS, param.frame_rate);
        cap.set(CAP_PROP_FRAME_WIDTH, param.frame_width);
        cap.set(CAP_PROP_FRAME_HEIGHT, param.frame_height);
        cap_map.insert({num, cap});
    }

    initVideoRaw();
    setCamera(param.get_run_mode());
    namedWindow("operator_img", cv::WINDOW_NORMAL);
    resizeWindow("operator_img", 1200, 720);
    moveWindow("operator_img", 0, 0);

    while (param.get_run_mode() != HALT)
    {
        logger.info("Vision Online!");
        // Pop electric control data
        try {
            ecu_data_try = receive_sub.try_pop();
            if (ecu_data_try.first == true) {
                ecu_data = ecu_data_try.second;
                logger.info("camp:{} visual_flag:{} view:{} mode:{}",ecu_data.camp, ecu_data.visual_flag, ecu_data.view, ecu_data.mode);
            } else {
                ecu_data.view = param.operator_cam_index;
                ecu_data.mode = 2;
                ecu_data.visual_flag = 1;
                if(param.camp == -1){
                    ecu_data.camp = 0;
                }
                else{
                    ecu_data.camp = param.camp;
                }
            }
        } catch (exception e) {
            logger.warn("ECU_DATA reiceve error!");
        }

        param.operator_cam_index = ecu_data.view % 4;
        param.camp = ecu_data.camp;
        param.visual_status = ecu_data.visual_flag;
        param.view = ecu_data.view;

        // Set mode when ecu data changed
        if(param.get_run_mode() != ecu_data.mode){
            // 0 for gold mine, 1 for silver mine, 2 for exchange site, 3 for halt
            param.set_run_mode((MODE)ecu_data.mode);
            logger.warn("Mode change to: {}", param.get_run_mode());
            setCamera(ecu_data.mode);
        }


        // Set camera index
        vision_cap = cap_map.at(param.vision_cam_index);
        operator_cap = cap_map.at(param.operator_cam_index);


        if (!vision_cap.isOpened()) {
            logger.error("Vision Camera is not opened");
        } else {
            vision_cap >> vision_img;
            if (!vision_img.empty()) {
                writeImageRaw(index++, vision_img);
                writeVideoRaw(vision_img);
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

void Sensor::writeImageRaw(int index, Mat& img){
    if(index % 5 == 0){
        for(auto num: param.writer_set){
            imwrite(param.get_log_path(num, param.sensor_prefix, index) + ".jpg", img);
        }
    }
}

void Sensor::initVideoRaw() {

    for(auto num: param.writer_set){
        VideoWriter videoWriter(param.get_log_path(num, param.sensor_prefix) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
        writer_map.insert(pair<int,VideoWriter>(num,videoWriter));
    }
}

void Sensor::writeVideoRaw(cv::Mat &img) {

    if(writer_map.empty()){
        return;
    }
    for(auto num: param.writer_set){
        VideoWriter videoWriter = writer_map.at(num);
        videoWriter.write(img);
        if (frame_index++ >= 1800) {
            frame_index = 0;
            videoWriter.release();
            writer_map.at(num) = VideoWriter(param.get_log_path(num, param.sensor_prefix) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
        }
    }
}

void Sensor::setCamera(int mode) {
    if (mode == 2) {
        UVC uvc(param.cam_map[param.vision_cam_index].c_str());
        uvc.initUVC(param.exposure_time_exchangesite);
        logger.info("Change to exchange mine mode.");
    } else{
        UVC uvc(param.cam_map[param.vision_cam_index].c_str());
        uvc.initUVC(param.exposure_time_mine);
        logger.info("Change to get mine mode.");
    }
}