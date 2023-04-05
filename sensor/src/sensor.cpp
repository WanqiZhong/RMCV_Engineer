#include "sensor.hpp"
#define Laptop


void Sensor::Run()
{
    #ifndef Laptop
    logger.info("Sensor Run");
    #else
    cout<<"Sensor Run"<<endl;
    #endif
    Sensor_thread = thread(&Sensor::Sensor_Run,this);
}

void Sensor::Join()
{
    #ifndef Laptop
    logger.info("Waiting for [Sensor]");
    #else
    cout<<"Waiting for [Sensor]"<<endl;
    #endif
    Sensor_thread.join();
    #ifndef Laptop
    logger.sinfo("[Sensor] Join");
    #else
    cout<<"[Sensor] Join"<<endl;
    #endif
}

void Sensor::Sensor_Run()
{
    // cout<<"Try open1"<<endl;
    // cv::VideoCapture cap(0);
    // cout<<"Try open2"<<endl;


    // VideoCapture cap(0);
    // struct v4l2_control control_s;
    // control_s.id= V4L2_CID_EXPOSURE_AUTO;
    // control_s.value= V4L2_EXPOSURE_MANUAL;
    // ioctl(dev, VIDIOC_S_CTRL, &control_s);
    // cap.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
    // cap.set(CAP_PROP_FPS, 30);
    // cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    // cap.set(CAP_PROP_BRIGHTNESS, 0);//亮度 1
    // cap.set(CAP_PROP_CONTRAST,20);//对比度 40
    // cap.set(CAP_PROP_SATURATION, 10);//饱和度 50
    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    // cap.set(CAP_PROP_EXPOSURE, -10);//曝光 50

    // read mp4
    // VideoCapture cap("../mine2.mp4");

    Mat img = imread("../mine2.png");
    // Mat img;
    umt::Publisher<cv::Mat> pub("channel0");
    while(mode!=HALT) //((mode=param.get_run_mode())!=HALT)
    {
        if(false)
        // if(!cap.isOpened())
        {
            #ifndef Laptop
            logger.error("Camera is not opened");
            #else
            cout<<"Camera is not opened"<<endl;
            #endif
            img = Mat::zeros(480, 800, CV_8UC1);
            pub.push(img.clone());

        }
        else
        {
            // cout<<"Camera is opened!"<<endl;
            // cap >> img;
            if(!img.empty())
            {
                imshow("push_img",img);
                waitKey(1);
                // imwrite("../mine_new.jpg",img);
                // cout<<img.cols<<img.rows<<endl;
                pub.push(img.clone());
            }
            else
            {
                cout<<"empty img"<<endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000. /30)));
    }
    //cap.release();
    cv::destroyAllWindows();
    return;
}


// void Sensor::Sensor_Run()
// {
//     // cout<<"Try open1"<<endl;
//     // // cv::VideoCapture cap(0);
//     // cout<<"Try open2"<<endl;
//     Mat img = imread("../mine.jpg");
//     umt::Publisher<cv::Mat> pub("channel0");
//     while(mode!=HALT) //((mode=param.get_run_mode())!=HALT)
//     {
//         if(false)
//         // if(!cap.isOpened())
//         {
//             #ifndef Laptop
//             logger.error("Camera is not opened");
//             #else
//             cout<<"Camera is not opened"<<endl;
//             #endif
//             img = Mat::zeros(480, 800, 8UC1);
//             pub.push(img.clone());

//         }
//         else
//         {
//             // cout<<"Camera is opened!"<<endl;
//             // cap>>img;
//             if(!img.empty())
//             {
//                 // imshow("push_img",img);
//                 waitKey(1);
//                 pub.push(img.clone());
//             }
//             else
//             {
//                 cout<<"empty img"<<endl;
//             }
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(int(1000. /30)));
//     }
//     // cap.release();
//     cv::destroyAllWindows();
//     return;
// }
