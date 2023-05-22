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

    for(int i=0; i < cam_name_maps.size(); i++){
        UVC uvc(cam_name_maps.at(i).c_str());
        uvc.initUVC(param.exposure_time_mine);
    }

    for(int i= 0; i < cam_name_maps.size(); i++){
        VideoCapture cap(cam_name_maps[i], CAP_V4L);
        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(CAP_PROP_FPS, param.frame_rate);
        cap.set(CAP_PROP_FRAME_WIDTH, param.frame_width);
        cap.set(CAP_PROP_FRAME_HEIGHT, param.frame_height);
        cap_set.push_back(cap);

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
            } else {
                ecu_data.view = 1;
                ecu_data.mode = 2;
                ecu_data.visual_flag = 1;
                // ecu_data.position_id = 0;
            }
        } catch (exception e) {
            logger.warn("ECU_DATA reiceve error!");
        }

        param.operator_cam_index = ecu_data.view % 4;

        // Set mode when ecu data changed
        if(param.get_run_mode() != ecu_data.mode){
            // 0 for gold mine, 1 for silver mine, 2 for exchange site, 3 for halt
            param.set_run_mode((MODE)ecu_data.mode);
            logger.warn("Mode change to: {}", param.get_run_mode());
            setCamera(ecu_data.mode);
        }

        param.visual_status = ecu_data.visual_flag;
        param.view = ecu_data.view;

        // Set camera index
        vision_cap = cap_set[param.vision_cam_index];
        operator_cap = cap_set[param.operator_cam_index];

        if (!vision_cap.isOpened()) {
            logger.error("Vision Camera is not opened");
            vision_img = Mat::zeros(param.frame_height, param.frame_width, CV_8UC1);
            pub.push(vision_img.clone());
        } else {
            vision_cap >> vision_img;
            if (!vision_img.empty()) {
                imageRaw(index++, vision_img);
                videoRaw(vision_img);
                imshow("vision_img", vision_img);
                waitKey(1);
            }
            if(param.visual_status!=0){
                pub.push(vision_img.clone());
            }
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
        fs::path dirPath = param.image_prefix;
        if (fs::exists(dirPath)) {
            logger.info("Path already exists.");
        } else {
            if (fs::create_directories(dirPath)) {
                logger.info("Directory created.");
            } else {
                logger.critical("Failed to create directory." );
            }
        }
        imwrite( param.image_prefix + '/' + std::string(timestamp)+ to_string(index) + ".jpg",img);
    }
}

void Sensor::initVideoRaw() {

    auto now = std::chrono::system_clock::now();
    auto t_c = std::chrono::system_clock::to_time_t(now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&t_c));

    for(auto num:writer_num){
        string path = param.sensor_prefix + cam_name_maps.at(num);
        fs::path dirPath = path;
        if (fs::exists(dirPath)) {
            logger.info("Path already exists.");
        } else {
            if (fs::create_directories(dirPath)) {
                logger.info("Directory created.");
            } else {
                logger.critical("Failed to create directory." );
            }
        }
        VideoWriter videoWriter(path + '/' + std::string(timestamp) + ".mp4", codec, fps, frameSize);
        writer_map.insert(pair<int,VideoWriter>(num,videoWriter));
    }
}

void Sensor::videoRaw(cv::Mat &img) {

    if(writer_map.empty()){
        return;
    }
    VideoWriter videoWriter = writer_map.at(writer_num[0]);
    videoWriter.write(img);
    if (frame_index++ >= 1800) {
        frame_index = 0;

        auto now = std::chrono::system_clock::now();
        auto t_c = std::chrono::system_clock::to_time_t(now);
        char timestamp[20];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&t_c));

        videoWriter.release();

        string path = param.sensor_prefix + cam_name_maps.at(writer_num[0]);
        writer_map.at(writer_num[0]) = VideoWriter(path + '/' + std::string(timestamp) + ".mp4", codec, fps, frameSize);
    }

}

void Sensor::videoRaw(vector<Mat> &img) {

    if(writer_map.empty()){
        logger.info("No need to log");
        return;
    }
    for(int i = 0; i < writer_num.size(); i++)
    {
        VideoWriter videoWriter = writer_map.at(writer_num[i]);
        videoWriter.write(img[i]);
        if (frame_index >= 1800) {
            frame_index = 0;

            auto now = std::chrono::system_clock::now();
            auto t_c = std::chrono::system_clock::to_time_t(now);
            char timestamp[20];
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&t_c));

            videoWriter.release();

            string path = param.sensor_prefix + cam_name_maps.at(writer_num[0]);
            writer_map.at(writer_num[i]) = VideoWriter(path + '/' + std::string(timestamp) + ".mp4", codec, fps, frameSize);
        }
    }
    frame_index++;

}

void Sensor::setCamera(int mode) {
    if (mode == 2) {
        UVC uvc(cam_name_maps[param.vision_cam_index].c_str());
        uvc.initUVC(param.exposure_time_exchangesite);
        logger.info("Change to exchange mine mode.");
    } else{
        UVC uvc(cam_name_maps[param.vision_cam_index].c_str());
        uvc.initUVC(param.exposure_time_mine);
        logger.info("Change to get mine mode.");
    }
}