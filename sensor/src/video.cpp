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

    umt::Publisher<cv::Mat> pub("channel0");

    if(param.image_read){
        vision_img = imread(param.image_path);
    }
    else{
        logger.info("Use video logs");
        vision_cap = VideoCapture(param.image_path);
        vision_cap.set(CAP_PROP_FOURCC, param.codec);
        vision_cap.set(CAP_PROP_FPS, param.detector_fps);
        vision_cap.set(CAP_PROP_FRAME_WIDTH, param.frame_width);
        vision_cap.set(CAP_PROP_FRAME_HEIGHT, param.frame_height);
    }

    // param.set_run_mode((MODE)(2));

    while (param.get_run_mode() != HALT){
        if(param.image_read){
            if (!vision_img.empty()) {
                imshow("vision_img", vision_img);
                waitKey(1);
                pub.push(vision_img.clone());
            }
        }
        else{
                if (!vision_cap.isOpened()) {
                logger.error("Vision Camera is not opened");
                } else {
                    vision_cap >> vision_img;
                    if (!vision_img.empty()) {
                        imshow("vision_img", vision_img);
                        waitKey(1);
                        pub.push(vision_img.clone());
                    }
                }
        }
    }
}