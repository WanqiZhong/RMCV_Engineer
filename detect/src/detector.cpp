#include "detector.hpp"

void Detector::Run()
{
    #ifndef Laptop
    logger.info("Detector Run");
    #else
    cout<<"Detector Run"<<endl;
    #endif
    Detector_thread = thread(&Detector::Detect_Run,this);
}

void Detector::Join()
{
    #ifndef Laptop
    logger.info("Waiting for [Detector]");
    #else
    cout<<"Waiting for [Detector]"<<endl;
    #endif
    
    Detector_thread.join();
    
    #ifndef Laptop
    logger.info("[Detector] joined");
    #else
    cout<<"[Detector] joined"<<endl;
    #endif
}

void Detector::Detect_Run()
{
    cv:Mat img;
    umt::Subscriber<cv::Mat> pub("channel1");
    umt::Publisher<MINE_POSITION_MSG> mine_sub("anchor_point_data");
    while(mode!=HALT)
    {
        try{
            img = pub.pop();
            imshow("img",img);
            waitKey(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(int(1000. /30)));
        }
        catch(const HaltEvent&){
            break;
        }
        // if(mode==GoldMode)
        // {
        //     GoldMineDetect_Run(img);
        //     // GoldMineDetect_Run2(img);
        //     mine_sub.push(MINE_POSITION_MSG(anchor_point));
        //     logger.info("GoldMineDetect_Run");
        // }
        // else if(mode==SilverMode)
        // {
        //     SilverMineDetect_Run(img);
        //     // mine_sub.push(MINE_POSITION_MSG(silver_mine_rect));
        //     logger.info("SilverMineDetect_Run");
        // }
        // else if(mode==ChangeSiteMode)
        // {
        //     ChangeSiteDetect_Run(img);
        // }
        // else
        // {
        //     logger.info("Detector mode error");
        // }
    }
}
