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

    // Sensor
    auto &local_sensor = robot_constants.at("sensor");
    toml::value &camera = local_sensor.at("camera");
    serial_number = camera.at("serial_number").as_string();
    frame_rate = camera.at("frame_rate").as_integer();

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
    // asc_begin_time.replace(asc_begin_time.find('\n'), 1, "");
    // std::replace(asc_begin_time.begin(), asc_begin_time.end(), ' ', '_');
    // std::replace(asc_begin_time.begin(), asc_begin_time.end(), ':', '_');


}

void DebugArgs::init(const toml::value &config)
{
    video_path = std::string(ROOT) + std::string(config.at("video").as_string());
}

void DetectorArgs::init(const toml::value &config)
{
    auto &exchangesite = config.at("exchangesite");
    auto &mine = config.at("mine");

    path2model_exchangesite =std::string(ROOT) + std::string(exchangesite.at("path").as_string());
    path2model_mine = std::string(ROOT) + std::string(mine.at("path").as_string());
}

} // args