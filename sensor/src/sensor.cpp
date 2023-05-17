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
    int index = 0;
    Logger logger("Sensor");
    umt::Subscriber<IMU_DATA_MSG> receive_sub("Electric_Data");
    cv::VideoCapture cap1("/dev/cam1", cv::CAP_V4L);
    cv::VideoCapture cap2("/dev/video0", cv::CAP_V4L);
    cv::VideoCapture cap3("/dev/cam4", cv::CAP_V4L);
    cv::VideoCapture cap4("/dev/cam3", cv::CAP_V4L);
    cap1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap3.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap4.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    cap1.set(CAP_PROP_FPS, 30);
    cap1.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap1.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap2.set(CAP_PROP_FPS, 60);
    cap2.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap2.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap3.set(CAP_PROP_FPS, 30);
    cap3.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap3.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap4.set(CAP_PROP_FPS, 30);
    cap4.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap4.set(CAP_PROP_FRAME_HEIGHT, 720);

    namedWindow("ImageSend", cv::WINDOW_NORMAL);
    resizeWindow("ImageSend", 1200, 720);
    moveWindow("ImageSend", 0, 0);


    // Define video output properties
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    double fps = 30.0;
    Size frameSize = Size((int)cap1.get(CAP_PROP_FRAME_WIDTH), (int)cap1.get(CAP_PROP_FRAME_HEIGHT));

    // Define timestamp format for video file names
    auto now = std::chrono::system_clock::now();
    auto t_c = std::chrono::system_clock::to_time_t(now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&t_c));

    // Define VideoWriter objects
//    VideoWriter writer1("../raw/cam1/cam1_" + std::string(timestamp) + ".mp4", codec, fps, frameSize);
    VideoWriter writer2("../raw/cam2/cam2_1.mp4", codec, fps, frameSize);
    // VideoWriter writer3("../raw/cam3/cam3_" + std::string(timestamp) + ".mp4", codec, fps, frameSize);
    // VideoWriter writer4("../raw/cam4/cam4_" + std::string(timestamp) + ".mp4", codec, fps, frameSize);

    logger.warn("内录已开始");

    // Define timer and frame counter variables
    int interval = 60; // in seconds
    auto start_time = std::chrono::steady_clock::now();
    int frame_count1 = 0;
    int frame_count2 = 0;

    // Capture and write frames from cameras
    Mat frame1, frame2, frame3, frame4;
    IMU_DATA_MSG ecu_data;
    pair<bool,IMU_DATA_MSG> ecu_data_try;
    
    VideoCapture cap_on;
    umt::Publisher<cv::Mat> pub("channel0");
    while(mode!=HALT) //((mode=param.get_run_mode())!=HALT)
    {
        logger.info("Vision Online");
        
        try {
            ecu_data_try = receive_sub.try_pop(); 
            if(ecu_data_try.first == true){
                ecu_data = ecu_data_try.second;
            }
            else{
                ecu_data.cam_id = 0;
                ecu_data.mode = 0;
                ecu_data.position_id = 0;
                logger.info("ECU_DATA reiceve error!");
            }

        } catch(exception e) {
            logger.info("ECU_DATA reiceve error!");
        }
        logger.info("{}",ecu_data.cam_id);
        ecu_data.cam_id %= 4;

        if(ecu_data.cam_id == 0){
            logger.info("Defalut");
            cap_on = cap1;
        }
        else if(ecu_data.cam_id == 1){
            cap_on = cap2;
        }
        else if(ecu_data.cam_id == 2){
            cap_on = cap3;
        }
        else if(ecu_data.cam_id == 3){
            cap_on = cap4;
        }
        else{
            cap_on = cap1;
            logger.info("ECU_DATA camera error!");
        }

        if(ecu_data.mode == 0){
            logger.info("Get Mine");
        }
        else if(ecu_data.mode == 1){
            logger.info("Exchange Mine");
        }

        cap_on >> send_img;
        if(!send_img.empty())
        {
            imshow("ImageSend",send_img);
            waitKey(1);
        }
        logger.info("{}",send_img.empty());


        // cap1.read(frame1);
        // cap2.read(frame2);
        // cap3.read(frame3);
        // cap4.read(frame4);

    //    writer1.write(frame1);
        // writer2.write(frame2);
        // writer3.write(frame3);
        // writer4.write(frame4);
        // Update frame counters
        // frame_count1++;
        // frame_count2++;

        // Check if it's time to create new video files



        // if(false)
        if(!cap2.isOpened())
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
            cap2 >> img;
            if(!img.empty())
            {
                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();
                if (elapsed_time >= interval) {
                    // Define timestamp for new video files
                    auto now = std::chrono::system_clock::now();
                    auto t_c = std::chrono::system_clock::to_time_t(now);
                    char timestamp[20];
                    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&t_c));

                    // Close current VideoWriter objects and create new ones with new file names
                   writer2.release();
                   logger.critical("writer over");
                    // writer2.release();
                    // writer3.release();
                    // writer4.release();

                //    writer1.open("../raw/cam1/cam1_" + std::string(timestamp) + ".mp4", codec, fps, frameSize);
                    writer2.open("../raw/cam2/cam2_" + std::string(timestamp) + ".mp4", codec, fps, frameSize);
                    // writer3.open("../raw/cam3/cam3_" + std::string(timestamp) + ".mp4", codec, fps, frameSize);
                    // writer4.open("../raw/cam4/cam4_" + std::string(timestamp) + ".mp4", codec, fps, frameSize);

                    // frame_count1 = 0;
                    // frame_count2 = 0;

          //          logger.warn("内录结束");


                    start_time = std::chrono::steady_clock::now();
                }
                imshow("push_img",img);
                writer2.write(img);
                logger.info("img write");
                waitKey(1);
                if(index++ % 10 == 0){
                    imwrite("../raw/goldmine/"+std::string(timestamp)+ to_string(index) + ".jpg",img);
                }
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
