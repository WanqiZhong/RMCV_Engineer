#ifndef HITSZ_RMVISION_2022_UMT_OBJ_MANAGER_H_
#define HITSZ_RMVISION_2022_UMT_OBJ_MANAGER_H_
#include <mutex>
#include <vector>
#include <memory>
#include <unordered_map>
namespace umt{

    template<class T>
    class ObjManager : public T{    //inherit MessagePipe<message_type>
    public:
        using pipe_ptr_w = std::weak_ptr<T>;
        using pipe_ptr_s = std::shared_ptr<T>;

        ObjManager() = default;

        static pipe_ptr_s find_or_create(const std::string &msg_name){
            std::unique_lock<std::mutex> lock(mtx_);                    //lock this thread to make sure map cannot be changed
            auto iter = map_.find(msg_name);
            if(iter != map_.end()) return iter->second.lock();              //cast weak_ptr to shared_ptr
            pipe_ptr_s p_obj = std::make_shared<ObjManager<T>>();   //create shared ptr
            map_.template emplace(msg_name, p_obj);                         //add to map
            return p_obj;
        }
    private:
        static std::unordered_map<std::string,pipe_ptr_w> map_;     //inline?
        static std::mutex mtx_;


    };
    template<class T>
    inline std::unordered_map<std::string,typename ObjManager<T>::pipe_ptr_w> ObjManager<T>::map_;

    template<class T>
    inline std::mutex ObjManager<T>::mtx_;
}
#endif