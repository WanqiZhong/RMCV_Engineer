#include "video.hpp"

void Video::Run()
{
    logger.info("Video Run");
    Video_thread = thread(&Video::Video_Run,this);
}

void Video::Join()
{
    logger.info("Waiting for [Video]");
    Video_thread.join();
    logger.sinfo("[Video] Join");
}

void Video::Video_Run() {

    int index = 0;

    IMU_DATA_MSG ecu_data;
    pair<bool, IMU_DATA_MSG> ecu_data_try;
    umt::Publisher<cv::Mat> pub("channel0");
    umt::Subscriber<IMU_DATA_MSG> receive_sub("Electric_Data");

    if(param.image_read){
        vision_img = imread(param.image_path);
    }
    else{
        vision_cap.open(param.image_path, CAP_V4L);
        vision_cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
        vision_cap.set(CAP_PROP_FPS, 30);
        vision_cap.set(CAP_PROP_FRAME_WIDTH, 1280);
        vision_cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    }

    while (param.get_run_mode() != HALT)
    {
        logger.info("Vision Online!");

        // Pop electric control data
        try {
            ecu_data_try = receive_sub.try_pop();
            if (ecu_data_try.first == true) {
                ecu_data = ecu_data_try.second;
            } else {
                ecu_data.view = 1;
                ecu_data.mode = 2;
                // ecu_data.position_id = 0;
            }
        } catch (exception e) {
            logger.warn("ECU_DATA reiceve error!");
        }

        // Set operator camera index when ecu data changed
        if(ecu_data.view % 4 != param.operator_cam_index){
            ecu_data.view %= 4;
            param.operator_cam_index = ecu_data.view;
            logger.warn("Operator camera index change to: {}", param.operator_cam_index);
        }


        // Set mode when ecu data changed
        if(param.get_run_mode() != ecu_data.mode){
            // 0 for gold mine, 1 for silver mine, 2 for exchange site, 3 for halt
            param.set_run_mode((MODE)ecu_data.mode);
            logger.warn("Mode change to: {}", param.get_run_mode());
        }


        if(param.image_read){
            if (!vision_img.empty()) {
                imshow("vision_img", vision_img);
                waitKey(1);
            }
            pub.push(vision_img.clone());
        }
        else{
             if (!vision_cap.isOpened()) {
                logger.error("Vision Camera is not opened");
                vision_img = Mat::zeros(720, 1280, CV_8UC1);
                pub.push(vision_img.clone());
            } else {
                vision_cap >> vision_img;
                if (!vision_img.empty()) {
                    imshow("vision_img", vision_img);
                    waitKey(1);
                }
                pub.push(vision_img.clone());
             }
        }
    }
}
