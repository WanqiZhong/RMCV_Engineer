#ifndef HITSZ_RMVISION_2022_UMT_MESSAGE_HPP_
#define HITSZ_RMVISION_2022_UMT_MESSAGE_HPP_
#include "object_manager.hpp"
#include <memory>
#include <vector>
#include <mutex>
#include <list>
#include <queue>
#include <iostream>
#include <condition_variable>

#define HALT -1

namespace umt{
    class MessageError : public std::runtime_error {
    public: //protected
        using std::runtime_error::runtime_error;
    };

    class MessageError_Stopped : public MessageError {
    public:
        MessageError_Stopped() : MessageError("No publisher on this message!") {}
    };

    class MessageError_Nomsg : public MessageError{
    public:
        MessageError_Nomsg() : MessageError("No message available!"){}
    };

    template<class T>
    class Subscriber;

    template<class T>
    class Publisher;

    template<class T>                       //T is msg_type,commonly struct
    class MessagePipe{
        friend class Subscriber<T>;         //allow subscriber to edit message pipe
        friend class Publisher<T>;          //allow publisher to edit message pipe
    public:
        using MsgType = T;
    private:
        std::mutex subs_mtx;                //lock this thread,make it possible for adding or moving elements in subs.
        std::list<Subscriber<T> *> subs;    //contain all subscriber bind to this message.(ptr)
        std::mutex pubs_mtx;                //lock this thread,make it possible for adding or moving elements in pubs.
        std::list<Publisher<T> *> pubs;     //contain all publisher bind to this message.(ptr)
    };

    template<class T>                       //T is msg_type,commonly struct
    class Subscriber{
        friend Publisher<T>;
        using MsgManager = ObjManager<MessagePipe<T>>;
    public:
        Subscriber() = default;

        //explicit constructor
        explicit Subscriber(const std::string &msg_name, int quene_size = 1):fifo_size_(quene_size),msg_name_(msg_name){
            bind(msg_name);
        }

        //
        ~Subscriber() {reset();}

        void reset(){
            if(!fifo_.empty()) fifo_ = std::queue<T>();                  //if fifo_ not empty,create an empty one
            if(!ptr_msg_) return;                                        
            std::unique_lock<std::mutex> lock(ptr_msg_->subs_mtx);       //lock this thread,make it possible for adding or moving elements in subs.
            ptr_msg_->subs.remove(this);                                 //move this subscriber from msg's subs
            ptr_msg_.reset();                                            //reset the pointer
        }

        void bind(const std::string &msg_name){                          //bind this subscriber to message
            reset();                                                     //clear this subscriber
            ptr_msg_ = MsgManager::find_or_create(msg_name);             //get a ptr point to MessagePipe<Msg_Type>
            std::unique_lock<std::mutex> lock(ptr_msg_->subs_mtx);       //lock this thread,make it possible for adding or moving elements in subs.
            ptr_msg_->subs.emplace_front(this);                          //add this subscriber to msg's MessagePipe
        }

        size_t get_fifo_size() const {
            return fifo_size_;
        }

        void set_fifo_size(size_t size){
            fifo_size_ = size;
        }

        T pop(){
            if(!ptr_msg_) {
                throw MessageError_Nomsg();
            }
            std::unique_lock<std::mutex> lock(mtx_);

            /*Condition variable:only when pred is false,wait function choke this thread
            only when pred is true,and this thread receives a notification,wait function release this thread*/
            // cv_.wait(lock,[this](){
            //     logger->info("1");
            //     // return (!fifo_.empty()||ptr_msg_->pubs.empty())||param.get_run_mode() == HALT;           //only when message quene is empty
            //     //                                                         //and there exists a(or more) publisher bind to this message,choke
            //     return !fifo_.empty();
            // });

            while(true) {
                bool status = cv_.wait_for(lock, std::chrono::seconds(2), [this]() {
                    return !fifo_.empty();
                });
                if(status) {
                    break;
                } else {
                    //if(param == HALT) {
                        // throw HaltEvent();
                       	
                    //}
                }
            }
            
            // todo:
            // 在这里先进行一次是否进入停机状态的判断, 然后抛出另一种停机的异常, 并且在调用的地方进行捕获

            // if(ptr_msg_->pubs.empty()) {
            //     throw MessageError_Stopped();
            // }
            T temp = std::move(fifo_.front());
            fifo_.pop();
            return temp;
        }
        
        std::pair<bool, T> try_pop(){
            if(!ptr_msg_) {
                throw MessageError_Nomsg();
            }

            std::unique_lock<std::mutex> lock(mtx_);
            bool status = cv_.wait_for(lock, std::chrono::seconds(2), [this]() {
                return !fifo_.empty();
            });

            if(!status) {
                return std::make_pair(false, T());
            }

            T temp = std::move(fifo_.front());
            fifo_.pop();
            return std::make_pair(true, std::move(temp));
        }

        std::string get_msg_name() const{
            return msg_name_;
        }
    private:
        void write_msg(const T& msg){
            std::unique_lock<std::mutex> lock(mtx_);                        //lock thread,change fifo_
            if(fifo_size_ <= fifo_.size()){                                 //if overflow,pop
                fifo_.pop();
            }
            fifo_.push(msg);                                                 //push msg into fifo_,ready for pop
        }

        void notify_once() const{
            cv_.notify_one();
        }

    private:
        size_t fifo_size_ = {};
        std::queue<T> fifo_;
        mutable std::condition_variable cv_;
        mutable std::mutex mtx_;
        typename MsgManager::pipe_ptr_s ptr_msg_;
        std::string msg_name_;
    };

    template<class T>
    class Publisher{
        using MsgManager = ObjManager<MessagePipe<T>>;
    public:
        Publisher() = default;
        explicit Publisher(const std::string &msg_name){
            bind(msg_name);
        }

        void bind(const std::string &msg_name){                          //bind this publisher to message
            reset();                                                     //clear this subscriber
            ptr_msg_ = MsgManager::find_or_create(msg_name);             //get a ptr point to MessagePipe<Msg_Type>
            std::unique_lock<std::mutex> lock(ptr_msg_->pubs_mtx);       //lock this thread,make it possible for adding or moving elements in subs.
            ptr_msg_->pubs.emplace_front(this);                          //add this subscriber to msg's MessagePipe
        }

        void reset(){
            if(!ptr_msg_) return;                                        //if ptr empty,return
            std::unique_lock<std::mutex> lock(ptr_msg_->pubs_mtx);       //lock this thread,make it possible for adding or moving elements in pubs.
            ptr_msg_->pubs.remove(this);                                 //move this subscriber from msg's subs
            if(ptr_msg_->pubs.empty()){                                  //if the message has no publisher,release all subscribers
                for(const auto &sub:ptr_msg_->subs)
                    sub->notify_once();                                  //notify one subscriber to quit
            }
            ptr_msg_.reset();                                            //reset the pointer
        }

        void push(const T& msg){
            if(!ptr_msg_) return;
            std::unique_lock<std::mutex> lock(ptr_msg_->subs_mtx);      //lock thread,make is possible for notifying subscribers
            for(const auto& sub:ptr_msg_->subs){
                sub->write_msg(msg);
                sub->notify_once();                                     //push once,notify once
            }
        }

        explicit operator bool (){
            return ptr_msg_ != nullptr;
        }
    private:
        mutable std::mutex mtx_;
        typename MsgManager::pipe_ptr_s ptr_msg_;
    };
}

#endif
