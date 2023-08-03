#include "args.hpp"
#include "matrix_utils.hpp"
args::Args param(ROOT "config/config.toml", ROOT "config/local-config.toml", ROOT "config/local-config.default.toml");

namespace args
{

/**
 * @brief 返回正在运行的状态, 带读写锁
 * 
 * @return MODE 
 */
MODE Args::get_run_mode()
{
    pthread_rwlock_rdlock(&rwlock_);
    MODE result = _run_mode;
    pthread_rwlock_unlock(&rwlock_);
    return result;
}

void Args::set_run_mode(const MODE target)
{
    pthread_rwlock_wrlock(&rwlock_);
    if (_run_mode != HALT)
        _run_mode = target;
    pthread_rwlock_unlock(&rwlock_);
}

Args::Args(std::string config_path, std::string local_config_path, std::string local_config_path_default)
{
    config = toml::parse(config_path);

    if(!std::filesystem::exists(local_config_path)) {
        std::filesystem::copy_file(local_config_path_default, local_config_path);
    }
    local_config = toml::parse(local_config_path);

    // Global
    std::string mode = local_config.at("run_mode").as_string();
    if (mode == "GOLD_MODE") {
        _run_mode = GoldMode;
    } else if (mode == "SILVER_MODE"){
        _run_mode = SilverMode;
    } else if (mode == "EXCHANGE_SITE_MODE"){
        _run_mode = ExchangeSiteMode;
    }else {
        _run_mode = GoldMode;
    }
    pthread_rwlock_init(&rwlock_, NULL);


    // Detector
    auto &detector = config.at("detector");
    detector_args.init(detector);

    // Predictor
    auto &predictor = config.at("predictor");
    auto &predictor_local = local_config.at("predictor");

    std::string constants_prefix = std::string(ROOT) + std::string(predictor.at("constants_prefix").as_string());
    constants_path_default = constants_prefix + toml::find<std::string>(predictor, "constants_default");
    constants_path = constants_prefix + toml::find<std::string>(predictor_local, "constants");
    if(constants_path.find(".toml") == std::string::npos) {
        throw std::runtime_error("Wrong Constants Path: " + constants_path);
    }

    auto robot_constants = toml::parse(constants_path);

    //Camera
    auto &camrea = robot_constants.at("camera");
    toml_to_matrix(camrea.at("F"), F);
    toml_to_matrix(camrea.at("C"), C);

    // Visual
    auto &visual = robot_constants.at("visual");
    auto &visual_camera = visual.at("camera");
    visual_status = visual_camera.at("visual_status").as_integer();
    view = visual_camera.at("view").as_integer();
    camp = visual_camera.at("camp").as_integer();
    auto &transform = visual.at("transform");
    tran_tvecx = transform.at("tran_tvecx").as_floating();
    tran_tvecy = transform.at("tran_tvecy").as_floating();
    tran_tvecz = transform.at("tran_tvecz").as_floating();
    bias_tevcx = transform.at("bias_tevcx").as_floating();
    bias_tevcy = transform.at("bias_tevcy").as_floating();
    bias_tevcz = transform.at("bias_tevcz").as_floating();

    auto &calibration = robot_constants.at("calibration");
    cali_x = calibration.at("cali_x").as_floating();
    cali_y = calibration.at("cali_y").as_floating();
    cali_z = calibration.at("cali_z").as_floating();
    cali_roll = calibration.at("cali_roll").as_floating();
    cali_yaw = calibration.at("cali_yaw").as_floating();
    cali_pitch = calibration.at("cali_pitch").as_floating();

    auto &traditional = robot_constants.at("traditional");
    auto &gold = traditional.at("gold");
    auto &changesite = traditional.at("changesite");
    gold_maxval = gold.at("gold_maxval").as_integer();
    gold_thresh = gold.at("gold_thresh").as_integer();
    bound_small = gold.at("bound_small").as_integer();
    bound_big = gold.at("bound_big").as_integer();
    w = gold.at("w").as_integer();
    h = gold.at("h").as_integer();
    ratio_thres = gold.at("ratio_thres").as_floating();
    area_ratio_thres = gold.at("area_ratio_thres").as_floating();
    corner_thresh = gold.at("corner_thresh").as_integer();
    ratio_thres_min = gold.at("ratio_thres_min").as_floating();
    area_ratio_thres_min = gold.at("area_ratio_thres_min").as_floating();
    corner_contour_area_min = gold.at("corner_contour_area_min").as_integer();
    corner_contour_area_max = gold.at("corner_contour_area_max").as_integer();
    corner_rec_area_max = gold.at("corner_rec_area_max").as_integer();
    
    site_min_rate = changesite.at("site_min_rate").as_floating();
    site_max_rate = changesite.at("site_max_rate").as_floating();
    site_min_area = changesite.at("site_min_area").as_floating();
    site_max_area = changesite.at("site_max_area").as_floating();
    site_area_rate = changesite.at("site_area_rate").as_floating();

    // Sensor
    auto &local_sensor = robot_constants.at("sensor");
    toml::value &camera = local_sensor.at("camera");
    serial_number = camera.at("serial_number").as_string();
    frame_rate = camera.at("frame_rate").as_integer();
    frame_height = camera.at("frame_height").as_integer();
    frame_width = camera.at("frame_width").as_integer();
    codec =  cv::VideoWriter::fourcc('M','J','P','G');

    exposure_time_mine = camera.at("exposure_time_mine").as_floating();
    exposure_time_exchangesite = camera.at("exposure_time_exchangesite").as_floating();
    operator_cam_index = camera.at("operator_cam_index").as_integer();
    vision_cam_index = camera.at("vision_cam_index").as_integer();

    // Bridge
    auto &local_bridge = robot_constants.at("bridge");
    can_send_id = local_bridge.at("send_id").as_integer();
    can_recv_id = local_bridge.at("recv_id").as_integer();
    serial_name = local_bridge.at("serial_name").as_string();
    serial_rate = local_bridge.at("serial_rate").as_integer();
    use_can = local_bridge.at("use_can").as_boolean();

    // Video
    auto &video = config.at("video");
    image_read = video.at("image_read").as_boolean();
    image_log = video.at("image_log").as_boolean();
    image_path = std::string(ROOT) + std::string(video.at("image_path").as_string());
    cam_map = {{0,"/dev/video0"}};
    cap_set = {0};
    writer_set = {0};
    detector_writer_set = {0};

    // Log
    auto &log = config.at("log");
    auto &local_log = local_config.at("log");

    std::string level = log.at("level").as_string();
    if(level == "trace") {
        log_level = spdlog::level::trace;
    } else if(level == "debug") {
        log_level = spdlog::level::debug;
    } else if(level == "info") {
        log_level = spdlog::level::info;
    } else if(level == "warn") {
        log_level = spdlog::level::warn;
    } else if(level == "err") {
        log_level = spdlog::level::err;
    } else if(level == "critical") {
        log_level = spdlog::level::critical;
    } else if(level == "off") {
        log_level = spdlog::level::off;
    }

    info_prefix = std::string(ROOT) + std::string(log.at("info_prefix").as_string());
    error_prefix = std::string(ROOT) + std::string(log.at("error_prefix").as_string());

    auto &record = log.at("record");
    auto &local_record = local_log.at("record");

    sensor_need = local_record.at("sensor_need").as_boolean();
    detector_need = local_record.at("detector_need").as_boolean();
    image_need = local_record.at("image_need").as_boolean();

    sensor_prefix = std::string(ROOT) + std::string(record.at("sensor_prefix").as_string());
    detector_prefix = std::string(ROOT) + std::string(record.at("detector_prefix").as_string());
    image_prefix = std::string(ROOT) + std::string(record.at("image_prefix").as_string());

    sensor_fps = record.at("sensor_fps").as_integer();
    detector_fps = record.at("detector_fps").as_integer();
    image_stride = record.at("image_stride").as_integer();

    if(sensor_fps < 0) {
        sensor_fps = frame_rate; // use sensor config
    }

    // Debug
    auto &debug_cfg = local_config.at("debug");
    debug.init(debug_cfg);

    // Time
    begin_time = std::chrono::steady_clock::now();

    char buffer[20];
    auto now = std::chrono::system_clock::now();
    const time_t cnow = std::chrono::system_clock::to_time_t(now);
    std::strftime(buffer, 20, "%m_%d_%H:%M", std::localtime(&cnow));
    asc_begin_time = buffer;

}

void DebugArgs::init(const toml::value &config)
{
    video_path = std::string(ROOT) + std::string(config.at("video").as_string());
}

void DetectorArgs::init(const toml::value &config)
{
    auto &exchangesite = config.at("exchangesite");
    auto &mine = config.at("mine");
    auto &armor = config.at("armor");
    auto &wind = config.at("wind");

    path2model_am = std::string(ROOT) + std::string(armor.at("path").as_string());
    path2model_wm = std::string(ROOT) + std::string(wind.at("path").as_string());
    path2model_exchangesite =std::string(ROOT) + std::string(exchangesite.at("path").as_string());
    path2model_mine = std::string(ROOT) + std::string(mine.at("path").as_string());
}

string Args::get_log_path(int num, string prefix, int index){
    auto now = std::chrono::system_clock::now();
    auto t_c = std::chrono::system_clock::to_time_t(now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&t_c));
    string path = prefix + param.cam_map.at(num);
    fs::path dirPath = path;
    if (!fs::exists(dirPath)) {
        fs::create_directories(dirPath);
    }
    return std::string(path) + '/' + timestamp + to_string(index);
}

} // args