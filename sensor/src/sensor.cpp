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
    cv::VideoCapture cap(0);
    umt::Publisher<cv::Mat> pub("channel1");
    cout<<"Camera is opened"<<endl;
    while(mode!=HALT) //((mode=param.get_run_mode())!=HALT)
    {
        if(!cap.isOpened())
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
            cout<<"Camera is opened!"<<endl;
            cap>>img;
            if(!img.empty())
            {
                imshow("push_img",img);
                waitKey(1);
                pub.push(img.clone());
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000. /30)));
    }
    cap.release();
    cv::destroyAllWindows();
    return;
}
