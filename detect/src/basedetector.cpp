#include "basedetector.hpp"

vector<vector<Point>> Basedetector::getAnchorPoint(){
    return anchor_point;
}

void Basedetector::clearAnchorPoint() {
    anchor_point.clear();
}


void Basedetector::img_light_enhance(Mat &img, Mat &img_hsv){
    if(!img.empty()){

        int hmin = 0; int hmax = 50; int smin = 87;
        int smax = 255; int vmin = 175; int vmax = 255;
        int rows = img.rows; int cols = img.cols;

        Mat hsv, color_mask;
        int max_val = 255;

        cvtColor(img, img, COLOR_BGR2HSV);
        vector<Mat> hsv_channels(3);
        split(img, hsv_channels);   // 对V通道进行自适应直方图均衡化
        // Ptr<CLAHE> clahe = createCLAHE(2.0,tileGridSize=(8,8)).apply(hsv_channels[2],hsv_channels[2]);
        createCLAHE(2.0,Size(8,8))->apply(hsv_channels[2],hsv_channels[2]);

        merge(hsv_channels, img);
        cvtColor(img, img, COLOR_HSV2BGR);
        imshow("[IMG_LIGHT_ENHANCE_BEF]", img);

        cvtColor(img, img, COLOR_BGR2HSV);
        inRange(img,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),color_mask);

        int v_sum = 0; int count = 1;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (color_mask.at<uchar>(i, j) != 0) {
                    int v = img.at<Vec3b>(i, j)[2];
                    v_sum += v;
                    count++;
                }
            }
        }
        
        int avg_v = v_sum / count; v_sum = v_sum / 10000;
        logger.info("[IMG_LIGHT_ENHANCE_BEF] avg_v: {}, v_sum: {}", avg_v, v_sum);

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int h = img.at<Vec3b>(i, j)[0];
                int s = img.at<Vec3b>(i, j)[1];
                int v = img.at<Vec3b>(i, j)[2];
                if(v_sum < 1500) {
                    v *= 1.4;
                    v += 30;
                }else if(v_sum < 3000) {
                    v *= 1.3;
                    v += 30;
                }else{
                    v *= 1.2;
                    v += 20;
                }
                v = std::min(v, max_val);  // 取max(255,提亮后)
                img.at<Vec3b>(i, j) = Vec3b(h, s, v);
            }
        }

        inRange(img,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),color_mask);
        v_sum = 0;count = 1;

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (color_mask.at<uchar>(i, j) != 0) {
                    int v = img.at<Vec3b>(i, j)[2];
                    v_sum += v;
                    count++;
                }
            }
        }
        avg_v = v_sum / count; v_sum = v_sum / 10000;
        logger.info("[IMG_LIGHT_ENHANCE_AFT] avg_v: {}, v_sum: {}", avg_v, v_sum);

        cvtColor(img, img, COLOR_HSV2BGR);
        imshow("[IMG_LIGHT_ENHANCE_AFT]", img);

        cvtColor(img, img_hsv, COLOR_BGR2HSV);
        inRange(img_hsv,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),img_hsv);

	    Mat kernel = getStructuringElement(MORPH_RECT,Size(2,2)); 
        erode(img_hsv, img_hsv, kernel);
        kernel = getStructuringElement(MORPH_ELLIPSE,Size(2,2));
        morphologyEx(img_hsv, img_hsv,MORPH_CLOSE, kernel); 
        kernel = getStructuringElement(MORPH_ELLIPSE,Size(3,3));
        morphologyEx(img_hsv, img_hsv,MORPH_OPEN, kernel); 
        medianBlur(img_hsv,img_hsv,1);
        
    }
}

void Basedetector::initVideoRaw() {

    for(auto num: param.detector_writer_set){
        VideoWriter videoWriter(param.get_log_path(num, param.detector_prefix) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
        detector_writer_map.insert({num,videoWriter});
        logger.critical("write video:{}", param.get_log_path(num, param.detector_prefix));
    }
}

void Basedetector::writeVideoRaw(cv::Mat &img) {

    if(detector_writer_map.empty()){
        return;
    }
    for(auto num: param.detector_writer_set){
        VideoWriter videoWriter = detector_writer_map.at(num);
        videoWriter.write(img);
        // logger.info("write video:{}", param.get_log_path(num, param.detector_prefix));
        if (frame_index++ >= 200) {
            frame_index = 0;
            videoWriter.release();
            detector_writer_map.at(num) = VideoWriter(param.get_log_path(num, param.detector_prefix) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
        }
    }
}

void Basedetector::writeImageRaw(int index, Mat& img){
    for(auto num: param.writer_set){
        cout<< param.get_log_path(num, param.shot_prefix, index) + ".jpg" <<endl;
        imwrite(param.get_log_path(num, param.shot_prefix, index) + ".jpg", img);
    }
}

