#include "Profiler.hpp"
std::chrono::steady_clock::time_point Profiler::last_time_;
std::unordered_map<std::string, double> Profiler::timer_;
std::unordered_map<std::string, int> Profiler::cnter_;
bool Profiler::enable_ = true;
int Profiler::Frm_cnter_ = 0;
std::mutex Profiler::p_mutex_;

void Profiler::save(ProfilerSpy& spy){
    //std::lock_guard<std::mutex> lock(p_mutex_);
    if (timer_.find(spy.name_)!=timer_.end()){
        timer_[spy.name_]+= std::chrono::duration<double>(spy.end_-spy.start_).count();
        cnter_[spy.name_]++;
    }else{
        timer_[spy.name_] = std::chrono::duration<double>(spy.end_-spy.start_).count();
        cnter_[spy.name_]=0;
    }
    
}