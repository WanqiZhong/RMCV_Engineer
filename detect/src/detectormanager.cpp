#include "detectormanager.hpp"
#include "sitedetector.hpp"
#include "sitedetectorpro.hpp"
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
    umt::Subscriber<cv::Mat> sub("channel1");
    umt::Publisher<MINE_POSITION_MSG> anchor_pub("anchor_point_data");
    while(param.get_run_mode()!=HALT)
    {
        // a red mat
        // img = cv::Mat(1280,720,CV_8UC3,cv::Scalar(0,0,255));
        if( last_mode != param.get_run_mode()){
                last_mode = param.get_run_mode();
                switch (param.get_run_mode())
                {
                    case ExchangeSiteMode:
                        detector = make_shared<Sitedetector>();
                        break;
                    case GoldMode:
                        detector = make_shared<Minedetector>();
                        break;
                    default:
                        detector = make_shared<Sitedetector>();
                        break;
                }
        }
        try{
            img = sub.pop();
            // logger.warn("get img");
        }catch(const HaltEvent&){
            break;
        }
        if(!img.empty()){
            UpdateParam();
            detector->clearAnchorPoint();
            detector->Detector_Run(img);
            anchor_pub.push(MINE_POSITION_MSG{.goal=detector->getAnchorPoint()});
        }
    }
}



void Detectormanager::UpdateParam()
{
    auto robot_constants = toml::parse(param.constants_path);

    auto &visual = robot_constants.at("visual");
    auto &visual_camera = visual.at("camera");
    param.visual_status = visual_camera.at("visual_status").as_integer();
    param.view = visual_camera.at("view").as_integer();
    param.camp = visual_camera.at("camp").as_integer();
    auto &transform = visual.at("transform");
    param.tran_tvecx = transform.at("tran_tvecx").as_floating();
    param.tran_tvecy = transform.at("tran_tvecy").as_floating();
    param.tran_tvecz = transform.at("tran_tvecz").as_floating();
    param.bias_tevcx = transform.at("bias_tevcx").as_floating();
    param.bias_tevcy = transform.at("bias_tevcy").as_floating();
    param.bias_tevcz = transform.at("bias_tevcz").as_floating();

    auto &calibration = robot_constants.at("calibration");
    param.cali_x = calibration.at("cali_x").as_floating();
    param.cali_y = calibration.at("cali_y").as_floating();
    param.cali_z = calibration.at("cali_z").as_floating();
    param.cali_roll = calibration.at("cali_roll").as_floating();
    param.cali_yaw = calibration.at("cali_yaw").as_floating();
    param.cali_pitch = calibration.at("cali_pitch").as_floating();

    auto &traditional = robot_constants.at("traditional");
    auto &gold = traditional.at("gold");
    auto &changesite = traditional.at("changesite");
    param.gold_maxval = gold.at("gold_maxval").as_integer();
    param.gold_thresh = gold.at("gold_thresh").as_integer();
    param.bound_small = gold.at("bound_small").as_integer();
    param.bound_big = gold.at("bound_big").as_integer();
    param.w = gold.at("w").as_integer();
    param.h = gold.at("h").as_integer();
    param.ratio_thres = gold.at("ratio_thres").as_floating();
    param.area_ratio_thres = gold.at("area_ratio_thres").as_floating();
    param.corner_thresh = gold.at("corner_thresh").as_integer();
    param.ratio_thres_min = gold.at("ratio_thres_min").as_floating();
    param.area_ratio_thres_min = gold.at("area_ratio_thres_min").as_floating();
    param.corner_contour_area_min = gold.at("corner_contour_area_min").as_integer();
    param.corner_contour_area_max = gold.at("corner_contour_area_max").as_integer();
    param.corner_rec_area_min = gold.at("corner_rec_area_min").as_integer();
    
    param.site_min_rate = changesite.at("site_min_rate").as_floating();
    param.site_max_rate = changesite.at("site_max_rate").as_floating();
    param.site_min_area = changesite.at("site_min_area").as_floating();
    param.site_max_area = changesite.at("site_max_area").as_floating();
    param.site_area_rate = changesite.at("site_area_rate").as_floating();
    param.G_avg_max = changesite.at("G_avg_max").as_integer();
}