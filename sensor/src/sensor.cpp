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
    umt::Publisher<EG_HEART_BEAT> hb_pub("check");

    for(auto num: param.cap_set){
        UVC uvc(param.cam_map.at(num).c_str());
        uvc.initUVC(param.exposure_time_exchangesite);
        VideoCapture cap(param.cam_map.at(num), CAP_V4L);
        cap.set(CAP_PROP_FOURCC, param.codec);
        cap.set(CAP_PROP_FPS, param.frame_rate);
        cap.set(CAP_PROP_FRAME_WIDTH, param.frame_width);
        cap.set(CAP_PROP_FRAME_HEIGHT, param.frame_height);
        cap_map.insert({num, cap});
        logger.info("Success set cam_{}", num);
    }

    initVideoRaw();
    setCamera(param.get_run_mode());

    while (param.get_run_mode() != HALT)
    {
        logger.info("Vision Online!");
        // Pop electric control data
        try {
            ecu_data_try = receive_sub.try_pop();
            if (ecu_data_try.first == true) {
                ecu_data = ecu_data_try.second;
                param.camp = ecu_data.camp;
                logger.info("camp:{} mode:{}",ecu_data.camp, ecu_data.mode);
            }
            // } else {
            //     logger.critical("Default");
            //     ecu_data.mode = param.default_mode;
            //     if(param.camp == -1){
            //         ecu_data.camp = 0;
            //     }else{
            //         ecu_data.camp = param.camp;
            //     }
            // }
        } catch (exception e) {
            logger.warn("ECU_DATA reiceve error!");
        }

        // [DEBUG]
        // ecu_data.mode = param.default_mode;
        // setCamera(ecu_data.mode);

        // Set mode when ecu data changed
        if(param.get_run_mode() != ecu_data.mode){
            // 0 for gold mine, 1 for silver mine, 2 for exchange site, 3 for halt
            param.set_run_mode((MODE)ecu_data.mode);
            setCamera((int)ecu_data.mode);
            logger.warn("Mode change to: {}", param.get_run_mode());
        }

        logger.warn("Mode: {}",param.get_run_mode());

        // Set camera index
        vision_cap = cap_map.at(param.vision_cam_index);

        if (!vision_cap.isOpened()) {
            logger.error("Vision Camera is not opened");
        } else {
            vision_cap >> vision_img;
            if (!vision_img.empty()) {
                // pub.push(EG_HEART_BEAT{});
                writeImageRaw(index++, vision_img);
                writeVideoRaw(vision_img);
                imshow("vision_img", vision_img);
                waitKey(1);
            }
            pub.push(vision_img.clone());
     
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
        VideoWriter videoWriter(param.get_video_log_path(num, 0) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
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
        // logger.warn("Write video to {}", param.get_video_log_path(num, 0) + ".mp4");
        if (frame_index++ >= 300) {
            frame_index = 0;
            videoWriter.release();
            writer_map.at(num) = VideoWriter(param.get_video_log_path(num, 0) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
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
