#ifndef CRH_2022_LOG_HPP_
#define CRH_2022_LOG_HPP_
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <vector>
#include "args.hpp"

class Logger
{
    using string_view_t = fmt::basic_string_view<char>;
private:
    static std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink;
    static std::shared_ptr<spdlog::sinks::basic_file_sink_mt> err_sink;
    static std::shared_ptr<spdlog::sinks::basic_file_sink_mt> info_sink;
    std::shared_ptr<spdlog::logger> logger;

public:
    Logger(std::string name = "not_assigned");
    inline void flush()
    {
        logger->flush();
    }

    template <typename... Args>
    inline void sinfo(const std::string &fmt, const Args &...args)
    {
        logger->info("\033[47m\033[30m" + fmt + "\033[0m", args...);
    }

    template <typename... Args>
    inline void trace(string_view_t fmt, const Args &...args)
    {
        logger->trace(fmt, args...);
    }

    template <typename... Args>
    inline void debug(string_view_t fmt, const Args &...args)
    {
        logger->debug(fmt, args...);
    }

    template <typename... Args>
    inline void info(string_view_t fmt, const Args &...args)
    {
        logger->info(fmt, args...);
    }

    template <typename... Args>
    inline void warn(string_view_t fmt, const Args &...args)
    {
        logger->warn(fmt, args...);
    }

    template <typename... Args>
    inline void error(string_view_t fmt, const Args &...args)
    {
        logger->error(fmt, args...);
    }

    template <typename... Args>
    inline void critical(string_view_t fmt, const Args &...args)
    {
        logger->critical(fmt, args...);
    }

};

#endif