#pragma once
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <sstream>

static std::chrono::steady_clock::time_point last_tick_time;
void PrintFps(cv::Mat& canvas){
   
    static int fcnt=0;
    static float fps=0;
    using namespace std::chrono;
    fcnt++;
    std::chrono::steady_clock::time_point tick_time = steady_clock::now();
    duration<float>time_span = duration_cast<duration<float>>(tick_time-last_tick_time);
    float delta_time=time_span.count();//seconds
   
    if (delta_time>=1){
        last_tick_time=tick_time;
        
        fps = fcnt/delta_time;
        fcnt=0;
    }
    static std::stringstream ss;
    ss.str("");
        ss<<"fps: "<<fps;
        auto txt = ss.str();
    cv::putText(canvas, txt, cv::Point2f(50,50), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 122, 22),2);
    

}