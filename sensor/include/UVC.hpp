//
// Created by future on 23-5-17.
//

#ifndef ENGCV_2023_UVC_HPP_
#define ENGCV_2023_UVC_HPP_ 

#include <iostream>
#include <unistd.h>
#include <error.h>
#include <string.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <log.hpp>

using namespace std;
using namespace cv;

class UVC{

    public:
        int fd;
        const char* camera_id;
        Logger logger = Logger("UVC");

        UVC(const char* camera_id);
        ~UVC();

        int setCamParam(struct v4l2_queryctrl *qctrl, int value);
        int setCamParam(int id, int value);
        void queryCamParam(int id, int fd);
        void initUVC(int explosure=250, int fps = 30);
        void queryUVC();
        VideoCapture getVideoStream();

};


#endif //ENGCV_2023_UVC_HPP_
