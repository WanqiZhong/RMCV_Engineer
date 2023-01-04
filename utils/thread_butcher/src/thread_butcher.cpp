#include "thread_butcher.hpp"
#include "log.hpp"

void crh::kill_thread(std::thread& victim, const std::string& description)
{
    auto native_handle = victim.native_handle();
    auto res = pthread_cancel(native_handle);
    Logger logger {"Butcher"};
    if (res != 0)
    {
        logger.error("Can't kill thread" + description + ": " + std::strerror(res));
        // should std::terminate() here?
    }
    else
    {
        logger.info("Thread " + description + "was killed successfully.");
    }
    victim.detach();
}