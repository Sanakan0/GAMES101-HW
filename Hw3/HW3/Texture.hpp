//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v,bool linear=false)
    {
        if(linear){
            return getColor_linear(u, v);
        }
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColor_linear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        
        cv::Vec3b color(0,0,0);
        int u_ldpixel = u_img-0.5;
        int v_ldpixel = v_img-0.5;
        const static int mvx[]={0,1,0,1};
        const static int mvy[]={0,0,1,1};
        bool flg=0;
        for (int i=0;i<4;++i){
            int nu=u_ldpixel+mvx[i];
            int nv=v_ldpixel+mvy[i];
            if (nu<0||nv<0||nu>=width||nv>=height){
                flg=1;
                break;
            }
            cv::Point2f tmpcnter(nu+0.5,nv+0.5);
            auto vec = cv::Point2f(u_img,v_img)-tmpcnter;
            auto weight = (1.0-abs(vec.x))*(1.0-abs(vec.y));
            //std::cout<< weight << std::endl;
            //color+=cv::Vec3b(1,1,1)*weight;
            color+=image_data.at<cv::Vec3b>(nv, nu)*weight;
        }
        
        if (flg){
            color = image_data.at<cv::Vec3b>(v_img, u_img);
        }
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
