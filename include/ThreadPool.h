#ifndef THREAD_POOLING
#define THREAD_POOLING

#include <functional>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <climits>
#include <iostream>

template <class T, class Function>
class ThreadPool
{
public:
    ThreadPool() : max_queue_size_(INT_MAX), hard_stop_(false), soft_stop_(false), blocking_enqueue_(false)
    {}
    
    ThreadPool(std::size_t count, Function func) : 
        max_queue_size_(INT_MAX), hard_stop_(false), soft_stop_(false), blocking_enqueue_(false), func_(func), 
        thread_pool_(count,std::bind(&ThreadPool::threadCore,this))
    {}
    
    void setBlocking(const bool& block = true)
    {
        blocking_enqueue_ = block;
    }
    
    void setMaxQueueSize(const std::size_t size)
    {
        max_queue_size_ = size;
    }
    
    void setFunction(Function func)
    {
        func_ = func;
    }
    
    void start(std::size_t count)
    {
        for (std::size_t i = 0; i < count; ++i)
            thread_pool_.emplace_back(std::bind(&ThreadPool::threadCore,this));
    }
    
    void stop()
    {
        hard_stop_ = true;
        enqueue_cv_.notify_all();
        data_extracted_cv_.notify_all();
        for (auto& t : thread_pool_) {
            t.join();
        }
    }
    
    void waitForStop()
    {
        soft_stop_ = true;
        enqueue_cv_.notify_all();
        data_extracted_cv_.notify_all();
        for (auto& t : thread_pool_) {
            t.join();
        }
    }
    
    void dispatch(const T& data)
    {
        //std::cout << "dispatching" << std::endl;
        std::unique_lock<std::mutex> lk(enqueue_mtx_);
        //std::cout << "dispatching with queue size " <<  data_queue_.size() << std::endl;
        if (data_queue_.size() >= max_queue_size_) {
            if (blocking_enqueue_) {
                //wait for free space
                data_extracted_cv_.wait(lk,[this]{return (data_queue_.size() < max_queue_size_) || hard_stop_ || soft_stop_;});
                if (hard_stop_ || soft_stop_) return;
            } else {
                //destroy the oldest thing in the queue
                data_queue_.pop();
            }
        }
        data_queue_.emplace(data);
        lk.unlock();
        //std::cout << "notifying enqueue" << std::endl;
        enqueue_cv_.notify_one();
    }

    void dispatch(T&& data)
    {
        std::unique_lock<std::mutex> lk(enqueue_mtx_);
        if (data_queue_.size() >= max_queue_size_) {
            if (blocking_enqueue_) {
                //wait for free space
                data_extracted_cv_.wait(lk,[this]{return (data_queue_.size() < max_queue_size_) || hard_stop_ || soft_stop_;});
                if (hard_stop_ || soft_stop_) return;
            } else {
                //destroy the oldest thing in the queue
                data_queue_.pop();
            }
        }
        data_queue_.emplace(std::forward<T>(data));
        lk.unlock();
        enqueue_cv_.notify_one();
    }

protected:
    void threadCore()
    {
        std::cout << "launching a threadCore()" <<std::endl;
        T local_data;
        while (!hard_stop_) {
            //wait for notification and extract from queue 
            {
                std::unique_lock<std::mutex> lk(enqueue_mtx_);
                if (data_queue_.empty()) {
                    if (soft_stop_ || hard_stop_) break;
                    //std::cout << "waiting for enqueue notification" <<std::endl;
                    enqueue_cv_.wait(lk,[this]{ return !data_queue_.empty() || hard_stop_ || soft_stop_; });
                    if (hard_stop_) break;
                    if (soft_stop_ && data_queue_.empty()) break;
                }
                std::swap(local_data,data_queue_.front());
                data_queue_.pop();
            }
            data_extracted_cv_.notify_one();
            func_(local_data);
        }
    }
    
    Function func_;
    
    std::size_t max_queue_size_;
    std::queue<T> data_queue_;

    bool hard_stop_;
    bool soft_stop_;
    bool blocking_enqueue_;
    
    std::vector<std::thread> thread_pool_;
    
    std::mutex enqueue_mtx_;
    std::condition_variable enqueue_cv_;
    std::condition_variable data_extracted_cv_;

};



#endif
