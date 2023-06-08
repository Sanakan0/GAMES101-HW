#pragma once

#include "rasterizer.hpp"
class Postprocesser{
public:
    Postprocesser(rst::rasterizer& ras);

    void BloomPass();
    void FragFilter(int src_tex_id,int dst_tex_id);
    void FragGblurH(int src_tex_id,int dst_tex_id);
    void FragGblurV(int src_tex_id,int dst_tex_id);
    void Upsample(int src_high_tex_id,int src_low_tex_id,int dst_tex_id);
    void Merge(int src_tex_id,int src_bloom_tex_id,int dst_tex_id);
    void switchbloom(){bsw_^=1;}
private:
    Eigen::Vector3f linearclampsample(float u,float v,int tex_id);

    rst::rasterizer& ras_;

    std::vector< std::vector<Eigen::Vector3f>> textures_;
    std::vector<std::pair<int,int>> texsz_;
    const float threshold=0.8;
    const float thresholdKnee=0.1;
    const float scatter=0.8;
    bool bsw_=1;
};


Eigen::Vector3f Postprocesser::linearclampsample(float u,float v,int tex_id)
{
    auto width = texsz_[tex_id].first;
    auto height = texsz_[tex_id].second;
    auto& img=textures_[tex_id];
    
    u=std::clamp(u,0.0f,1.0f-0.0001f);
    v=std::clamp(v,0.0f,1.0f-0.0001f);

    auto u_img = u * width;
    auto v_img = v * height;
    
    Vector3f color(0,0,0);
    int u_ldpixel = std::floor( u_img-0.5);
    int v_ldpixel = std::floor( v_img-0.5);
    constexpr static int mvx[]={0,1,0,1};
    constexpr static int mvy[]={0,0,1,1};
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
        color+=img[nu+nv*width]*weight;
    }
    
    if (flg){
        color = img[int(u_img)+int(v_img)*width];
    }
    return color;
}

Postprocesser::Postprocesser(rst::rasterizer& ras):ras_(ras){
    textures_.resize(16);
    int w,h;
    w=ras.width;
    h=ras.height;
  
    for (int i=0;i<7;++i){
        w>>=1;
        h>>=1;
        textures_[i*2].resize(w*h);
        textures_[i*2+1].resize(w*h);
        texsz_.push_back({w,h});
        texsz_.push_back({w,h});
    }
    textures_[14].resize(ras.width*ras.height);
    texsz_.push_back({ras.width,ras.height});
    textures_[15].resize(ras.width*ras.height);
    texsz_.push_back({ras.width,ras.height});
}

void Postprocesser::FragGblurH(int src_tex_id,int dst_tex_id){
    auto dstw=texsz_[dst_tex_id].first;
    auto dsth=texsz_[dst_tex_id].second;
    auto texelsz_x=1.0f/dstw;
    for (int iv=0;iv<dsth;++iv){
        for (int iu=0;iu<dstw;++iu){
            int ind=iu+iv*dstw;
            float u = (iu+0.5f)/dstw;
            float v = (iv+0.5f)/dsth;
            auto c0=linearclampsample(u-texelsz_x*4.0, v, src_tex_id);
            auto c1=linearclampsample(u-texelsz_x*3.0, v, src_tex_id);
            auto c2=linearclampsample(u-texelsz_x*2.0, v, src_tex_id);
            auto c3=linearclampsample(u-texelsz_x*1.0, v, src_tex_id);
            auto c4=linearclampsample(u, v, src_tex_id);
            auto c5=linearclampsample(u+texelsz_x*1.0, v, src_tex_id);
            auto c6=linearclampsample(u+texelsz_x*2.0, v, src_tex_id);
            auto c7=linearclampsample(u+texelsz_x*3.0, v, src_tex_id);
            auto c8=linearclampsample(u+texelsz_x*4.0, v, src_tex_id);
            auto  color = c0 * 0.01621622 + c1 * 0.05405405 + c2 * 0.12162162 + c3 * 0.19459459
                        + c4 * 0.22702703
                        + c5 * 0.19459459 + c6 * 0.12162162 + c7 * 0.05405405 + c8 * 0.01621622;
            textures_[dst_tex_id][ind] = color;
        }    
    }
}
void Postprocesser::FragGblurV(int src_tex_id,int dst_tex_id){
    auto dstw=texsz_[dst_tex_id].first;
    auto dsth=texsz_[dst_tex_id].second;
    auto texelsz_y=1.0f/dsth;
    for (int iv=0;iv<dsth;++iv){
        for (int iu=0;iu<dstw;++iu){
            int ind=iu+iv*dstw;
             float u = (iu+0.5f)/dstw;
            float v = (iv+0.5f)/dsth;
            auto c0 = linearclampsample(u,v-texelsz_y* 3.23076923, src_tex_id);
            auto c1 = linearclampsample(u,v-texelsz_y* 1.38461538, src_tex_id);
            auto c2 = linearclampsample(u,v                         , src_tex_id);
            auto c3 = linearclampsample(u,v+texelsz_y* 1.38461538, src_tex_id);
            auto c4 = linearclampsample(u,v+texelsz_y* 3.23076923, src_tex_id);
            
            auto color = c0 * 0.07027027 + c1 * 0.31621622
                        + c2 * 0.22702703
                        + c3 * 0.31621622 + c4 * 0.07027027;
            textures_[dst_tex_id][ind] = color;
        }    
    }
}

void Postprocesser::FragFilter(int src_tex_id,int dst_tex_id){
    auto dstw=texsz_[dst_tex_id].first;
    auto dsth=texsz_[dst_tex_id].second;
    
    for (int iv=0;iv<dsth;++iv){
        for (int iu=0;iu<dstw;++iu){
            int ind=iu+iv*dstw;
             float u = (iu+0.5f)/dstw;
            float v = (iv+0.5f)/dsth;
            auto color = linearclampsample(u,v,src_tex_id);
           
            auto t=threshold*255.f;
            auto tk=thresholdKnee*255.f;
            auto brightness = std::max({color.x(),color.y(),color.z()});
            auto softness = std::clamp(brightness-t+tk,0.0f,2.0f*tk);
            softness = (softness * softness) / (4.0 * tk + 1e-4);
            auto multiplier = std::max(brightness - t, softness) / std::max(brightness, 1e-4f);
            color*=std::max(multiplier,0.f);
            // auto brightness = color.x()*0.2126+color.y()*0.7152+color.z()*0.0722;
            // float multiplier = brightness>t?1:0;
            // color*=multiplier;
            color= color.cwiseProduct(color)*0.2;
            textures_[dst_tex_id][ind]=color;
        }
    }
}

void Postprocesser::Upsample(int src_high_tex_id,int src_low_tex_id,int dst_tex_id){
    auto dstw=texsz_[dst_tex_id].first;
    auto dsth=texsz_[dst_tex_id].second;
    
    for (int iv=0;iv<dsth;++iv){
        for (int iu=0;iu<dstw;++iu){
            int ind=iu+iv*dstw;
             float u = (iu+0.5f)/dstw;
            float v = (iv+0.5f)/dsth;
            auto highmip = linearclampsample(u,v,src_high_tex_id);
            auto lowmip = linearclampsample(u,v,src_low_tex_id);
            textures_[dst_tex_id][ind]=highmip*(1.f-scatter)+lowmip*scatter;
        }
    }
}


void Postprocesser::Merge(int src_tex_id,int src_bloom_tex_id,int dst_tex_id){
    auto dstw=texsz_[dst_tex_id].first;
    auto dsth=texsz_[dst_tex_id].second;
    
    for (int iv=0;iv<dsth;++iv){
        for (int iu=0;iu<dstw;++iu){
            int ind=iu+iv*dstw;
             float u = (iu+0.5f)/dstw;
            float v = (iv+0.5f)/dsth;
            auto c0 = linearclampsample(u,v,src_tex_id);
            auto c1 = linearclampsample(u,v,src_bloom_tex_id);
            textures_[dst_tex_id][ind]=c0+c1*1;
        }
    }
}

void Postprocesser::BloomPass(){
    if (!bsw_) return;
    auto& origin = ras_.active_frame_buffer();
    textures_[14].clear();
    textures_[14]=origin;
    constexpr static int D[]={0,2,4,6,8,10,12};
    constexpr static int U[]={1,3,5,7,9,11,13};
    FragFilter(14, D[0]);

    FragGblurH(D[0], U[1]);
    FragGblurV(U[1],D[1]);

    FragGblurH(D[1], U[2]);
    FragGblurV(U[2],D[2]);

    FragGblurH(D[2], U[3]);
    FragGblurV(U[3],D[3]);

    FragGblurH(D[3], U[4]);
    FragGblurV(U[4],D[4]);

    FragGblurH(D[4], U[5]);
    FragGblurV(U[5],D[5]);

    FragGblurH(D[5], U[6]);
    FragGblurV(U[6],D[6]);
    
    Upsample(D[5],D[6], U[5]);

    Upsample(D[4],U[5], U[4]);

    Upsample(D[3],U[4], U[3]);

    Upsample(D[2],U[3], U[2]);

    Upsample(D[1],U[2], U[1]);

    Upsample(D[0],U[1], U[0]);

    Upsample(U[0],U[0], 15);
    
    //Merge(14, U[0], 15);
    origin=textures_[15];
}