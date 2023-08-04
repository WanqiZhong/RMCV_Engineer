#include "detectormanager.hpp"
#include "sitedetector.hpp"
#include "minenetdetector.hpp"
#include "minedetector.hpp"

void Detectormanager::Run()
{
    logger.info("Detector Run");
    Detector_thread = thread(&Detectormanager::Manager_Run,this);
}

void Detectormanager::Join() {
    logger.info("Waiting for [Detector]");
    Detector_thread.join();
    logger.info("[Detector] joined");
}

void Detectormanager::Manager_Run()
{
    cv::Mat img;
    umt::Subscriber<cv::Mat> sub("channel0");
    umt::Publisher<MINE_POSITION_MSG> anchor_pub("anchor_point_data");
    while(param.get_run_mode()!=HALT)
    {
        try{
            img = sub.pop();
        }catch(const HaltEvent&){
            break;
        }
        if(!img.empty()){
            if( last_mode != param.get_run_mode()){
                last_mode = param.get_run_mode();
                switch (param.get_run_mode())
                {
                    case ExchangeSiteMode:
                        detector = make_shared<Sitedetector>();
                        break;
                    case GoldMode:
                        detector = make_shared<Minenetdetector>(param.detector_args.path2model_am, 0, 0);
                        break;
                    default:
                        detector = make_shared<Minedetector>();
                        break;
                }
            }
            detector->clearAnchorPoint();
            detector->Detector_Run(img);
            anchor_pub.push(MINE_POSITION_MSG{.goal=detector->getAnchorPoint()});
        }
    }
}