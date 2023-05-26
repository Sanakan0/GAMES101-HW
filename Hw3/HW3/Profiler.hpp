#pragma once
#include <string>
#include <chrono>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
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
    static void PrintInfoOnMat(cv::Mat& canvas){
        int ypos=50;
        if (Frm_cnter_==0) return;
        std::lock_guard<std::mutex> lock(p_mutex_);
        for (auto& i: timer_){
            ypos+=40;
            
            cv::putText(canvas, i.first +": "+ std::to_string( i.second*1000/Frm_cnter_)+" ms/f", cv::Point2f(50,ypos), cv::FONT_HERSHEY_DUPLEX, 0.7, CV_RGB(255, 122, 22),1);
        }
        
        
    }
    static void PrintInfo(){
        for (auto& i: timer_){
            //printf("%s: %.6f ms/frm | ",i.first.c_str(),i.second*1000/Frm_cnter_ );
 
        }
        // for (auto& i: cnter_){
        //     printf("%s: %.6f calls/frm | ",i.first.c_str(),1.0*i.second/Frm_cnter_ );
 
        // }
        //printf("\r");
        if (std::chrono::duration<double>(std::chrono::steady_clock::now()-last_time_).count()>1)
            Clear();
    }
    static void Update(){
        Frm_cnter_++;
    }
    static void Clear(){
        std::lock_guard<std::mutex> lock(p_mutex_);
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

