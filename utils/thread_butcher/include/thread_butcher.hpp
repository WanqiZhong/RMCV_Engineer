#include <pthread.h>
#include <thread>

namespace crh
{
    /**
     * @brief kill a thread by calling pthread_cancel
     * 
     * @param victim to be consumed
     * @param description who the victim is, this will be printed out for diagnose
     */
    void kill_thread(std::thread& victim, const std::string& description);
}