#include "log.hpp"


std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> Logger::console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
std::shared_ptr<spdlog::sinks::basic_file_sink_mt> Logger::err_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(param.error_prefix + param.asc_begin_time + ".log", true);
std::shared_ptr<spdlog::sinks::basic_file_sink_mt> Logger::info_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(param.info_prefix + param.asc_begin_time + ".log", true);


Logger::Logger(std::string name)
{
    /* TODO: add features: */
    /* configure multi sinks */
    std::vector<spdlog::sink_ptr> sinks{console_sink, err_sink, info_sink};
    logger = std::make_shared<spdlog::logger>(name, std::begin(sinks), std::end(sinks));

    /* setting output level and flush level */
    // logger->set_pattern("[%H:%M:%S.%e] [%=n] [%^%=l%$] %v");
    logger->flush_on(spdlog::level::critical);
}