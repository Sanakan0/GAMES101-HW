#pragma once
#include <string>
#include <chrono>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <mutex>

#define SPY(name)\
            std::unique_ptr<ProfilerSpy> profiler_spy_  =\
            Profiler::enable_? std::make_unique<ProfilerSpy>(name):nullptr

class ProfilerSpy;

class Profiler{
public:
    Profiler(){
        last_time_ = std::chrono::steady_clock::now();
    }
    static void save(ProfilerSpy& spy);
    static void PrintInfo(){
        for (auto& i: timer_){
            printf("%s: %.6f ms/frm | ",i.first.c_str(),i.second*1000/Frm_cnter_ );
 
        }
        // for (auto& i: cnter_){
        //     printf("%s: %.6f calls/frm | ",i.first.c_str(),1.0*i.second/Frm_cnter_ );
 
        // }
        printf("\r");
        if (std::chrono::duration<double>(std::chrono::steady_clock::now()-last_time_).count()>1)
            Clear();
    }
    static void Update(){
        Frm_cnter_++;
    }
    static void Clear(){
        timer_.clear();
        cnter_.clear();
        last_time_=std::chrono::steady_clock::now();
        Frm_cnter_=0;
    }
    static std::unordered_map<std::string, double> timer_;
    static std::unordered_map<std::string, int> cnter_;
    static std::chrono::steady_clock::time_point last_time_;
    static int Frm_cnter_;
    static bool enable_;
    static std::mutex p_mutex_;
};


class ProfilerSpy{
public:
    ProfilerSpy(const std::string& name,bool flag=0):
    name_(name),flag_(flag){
        if (flag_) return;    
        start_=std::chrono::steady_clock::now();
    }

    ~ProfilerSpy(){
        if (flag_) return;
        end_ = std::chrono::steady_clock::now();
        Profiler::save(*this);
    }
    const std::string name_;
    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::time_point end_;
    bool flag_;
};

