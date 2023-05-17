#include "args.hpp"

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
    if (debug.pseudo_mode)
    {
        size_t interval = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - begin_time).count();
        switch (interval / 5 % 4)
        {
        case 0:
            set_run_mode(GoldMode);
            break;
        case 1:
            set_run_mode(SilverMode);
            break;
        case 2:
            set_run_mode(ChangeSiteMode);
            break;
        case 3:
            set_run_mode(ChangeSiteMode);
            break;
        }
    }
        pthread_rwlock_rdlock(&rwlock_);
        MODE result = _run_mode;
        pthread_rwlock_unlock(&rwlock_);
        return result;
}

void Args::set_run_mode(const MODE target)
{
    pthread_rwlock_wrlock(&rwlock_);
    // if (_run_mode != HALT)
    //     _run_mode = target;
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
    pthread_rwlock_init(&rwlock_, NULL);

    // Sensor
    auto &sensor = config.at("sensor");
    auto &local_sensor = local_config.at("sensor");
    // assert(cameras.size() > 0); no need
    toml::value &camera = local_sensor.at("camera"); // TODO: multi-camera
    serial_number = camera.at("serial_number").as_string();
    camera_company = camera.at("camera_company").as_string();
    normal_grab_height = camera.at("normal_grab_height").as_integer();
    normal_grab_width = camera.at("normal_grab_width").as_integer();
    offset_x = camera.at("offset_x").as_integer();
    offset_y = camera.at("offset_y").as_integer();
    energy_offset_x = camera.at("energy_offset_x").as_integer();
    energy_offset_y = camera.at("energy_offset_y").as_integer();
    frame_rate = camera.at("frame_rate").as_integer();
    energy_grab_height = camera.at("energy_grab_height").as_integer();
    energy_grab_width = camera.at("energy_grab_width").as_integer();
    net_height = camera.at("net_height").as_integer();
    net_width = camera.at("net_width").as_integer();

    exposure_time = camera.at("exposure_time").as_floating();
    gain = camera.at("gain").as_floating();
    energy_exposure_time = camera.at("energy_exposure_time").as_floating();
    energy_gain = camera.at("energy_gain").as_floating();

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
        std::cout<<"Wrong Path"<<std::endl;
        throw std::runtime_error("Wrong Constants Path: " + constants_path);
    }


    // Bridge
    auto &bridge = config.at("bridge");
    auto &local_bridge = local_config.at("bridge");
    can_send_id = local_bridge.at("send_id").as_integer();
    can_recv_id = local_bridge.at("recv_id").as_integer();
    serial_name = local_bridge.at("serial_name").as_string();
    serial_rate = local_bridge.at("serial_rate").as_integer();
    use_can = local_bridge.at("use_can").as_boolean();

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

    detector_need = local_record.at("detector_need").as_boolean();
    detector_prefix = std::string(ROOT) + std::string(record.at("detector_prefix").as_string());
    detector_fps = record.at("detector_fps").as_integer();

    sensor_need = local_record.at("sensor_need").as_boolean();
    sensor_prefix = std::string(ROOT) + std::string(record.at("sensor_prefix").as_string());
    sensor_fps = record.at("sensor_fps").as_integer();
    if(sensor_fps < 0) {
        sensor_fps = frame_rate; // use sensor config
    }

    imu_need = local_record.at("imu_need").as_boolean();
    imu_prefix = std::string(ROOT) + std::string(record.at("imu_prefix").as_string());

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
    // asc_begin_time.replace(asc_begin_time.find('\n'), 1, "");
    // std::replace(asc_begin_time.begin(), asc_begin_time.end(), ' ', '_');
    // std::replace(asc_begin_time.begin(), asc_begin_time.end(), ':', '_');
}

void DebugArgs::init(const toml::value &config)
{
    pseudo_camera = config.at("pseudo_camera").as_boolean();
    video_path = std::string(ROOT) + std::string(config.at("video").as_string());
    pseudo_imu = config.at("pseudo_imu").as_boolean();
    skip_minutes = config.at("skip_minutes").as_floating();

    file_imu = config.at("file_imu").as_boolean();
    imu_path = std::string(ROOT) + std::string(config.at("imu_path").as_string());

    with_imshow_detector = config.at("with_imshow_detector").as_boolean();
    with_imshow_predictor = config.at("with_imshow_predictor").as_boolean();

    disable_auto_shoot = config.at("disable_auto_shoot").as_boolean();
    pseudo_mode = config.at("pseudo_mode").as_boolean();
}

void DetectorArgs::init(const toml::value &config)
{
    auto &armor = config.at("armor");
    auto &wind = config.at("wind");

    path2model_am = std::string(ROOT) + std::string(armor.at("path").as_string());
    path2model_wm = std::string(ROOT) + std::string(wind.at("path").as_string());

    nms_thresh[0] = armor.at("nms_thresh").as_floating();
    nms_thresh[1] = wind.at("nms_thresh").as_floating();
    conf_thresh[0] = armor.at("conf_thresh").as_floating();
    conf_thresh[1] = wind.at("conf_thresh").as_floating();
}

} // args
