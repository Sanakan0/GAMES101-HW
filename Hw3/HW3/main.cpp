#include <iostream>
#include <opencv2/opencv.hpp>


#include "global.hpp"
#include "rasterizer.hpp"
#include "Postprocess.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"
#include "FpsPrinter.hpp"
#include "Profiler.hpp"
#include <thread>
#include <random>
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float half_radialangle = eye_fov*MY_PI/360;
    float t=zNear*tan(half_radialangle);
    float r=aspect_ratio*t;
    projection <<   zNear/r,0,0,0,
                    0,zNear/t,0,0,
                    0,0,-(zFar+zNear)/(zFar-zNear),-2*zFar*zNear/(zFar-zNear),
                    0,0,-1,0;
                    


    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};


float fastpow(float x,int n){
    float res=1.0f;
    float tmp=x;
    while(n){
        if (n&1) res*=tmp;
        tmp*=tmp;
        n>>=1;
    }
    return res;
}


rst::rasterizer r(700, 700);
Postprocesser pp(r);

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f texture_color= payload.color;
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        texture_color=payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    
    

    const static Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    const static Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    const static auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    const static auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    const static std::vector<light> lights = {l1, l2};
    const static Eigen::Vector3f amb_light_intensity{10, 10, 10};
    const static Eigen::Vector3f eye_pos{0, 0, 10};

    const static float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    auto ambient =  ka.cwiseProduct(amb_light_intensity);
    Vector3f ldir,I,diffuse,h,specular;
    
    auto np = (-point).normalized();
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
                // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        
        auto tmpvec = light.position-point;
        ldir = tmpvec.normalized();
        float r = tmpvec.x()/ldir.x();
        I =light.intensity/(r*r);
        diffuse = kd.cwiseProduct(I*std::max(0.0f,normal.dot(ldir)));
        h = (np+ldir).normalized();
        auto tmpdot=std::max(0.0f,normal.dot(h));
        auto tmps=tmpdot>0.94f?fastpow(tmpdot,p):0.0f;
        specular = ks.cwiseProduct(I*tmps);
        result_color=result_color+ambient+diffuse+specular; 

    }

    return result_color * 255.f;
}

float random_f(){
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<float> dist(0.0, 1.0);
    //SPY("RANDOM");
    return dist(mt);
}


inline uint32_t xorshf32() {          //period 2^96-1
    static thread_local uint32_t x=998244353;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

inline uint64_t xorshf64() {          //period 2^96-1
    static thread_local uint64_t x=998244353;

	x ^= x >> 12;
	x ^= x << 25;
	x ^= x >> 27;
	return x*0x2545F4914F6CDD1DULL;
}


Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    //return {0,0,0};
    //SPY("PhongShader");
    Eigen::Vector3f texture_color= payload.color;
    if (payload.texture&&payload.tex_coords.x()>=0)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        texture_color=payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y())/255.f;
    }

    const static Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color;
    const static Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    const static auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    const static auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    const static std::vector<light> lights = {l1, l2};
    const static Eigen::Vector3f amb_light_intensity{10, 10, 10};
    const static Eigen::Vector3f eye_pos{0, 0, 10};

    const static float p = 150;
    Vector3f color,point,normal,result_color;
    color = payload.color;
    point = payload.view_pos;
    normal = payload.normal;
    result_color = {0, 0, 0};
    //return {1,1,1};
    Vector3f ldir,I,diffuse,h,specular,ambient;
    ambient =  ka.cwiseProduct(amb_light_intensity);
    auto np = (-point).normalized();
    //return {0,0,0};
    auto& ras=r;
    
    
    
    
    auto& ltmp = ras.getlights();
    auto& smap = ras.getshadowmap();

    const float bias = 0.00005;
    for (int i=0;i<ltmp.size();++i)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        auto& light = ltmp[i];
        auto tmpvec = light.pos_in_view-point;
        ldir = tmpvec.normalized();
        float r = tmpvec.x()/ldir.x();
        I =light.intensity/(r*r);
        diffuse = kd.cwiseProduct(I*std::max(0.0f,normal.dot(ldir)));
        h = (np+ldir).normalized();
        auto tmpdot=std::max(0.0f,normal.dot(h));
        auto tmps=tmpdot>0.94f?fastpow(tmpdot,p):0.0f;
        specular = ks.cwiseProduct(I*tmps);
        if (!ras.shadowon){
            result_color=result_color+(ambient+diffuse+specular);
            continue;
        }

        auto tmpp=Vector4f(point.x(),point.y(),point.z(),1);
        Vector4f ndcp= ras.lightprojection*ltmp[i].mat*ras.invview*tmpp;

        ndcp.x()/=ndcp.w();
        ndcp.y()/=ndcp.w();
        ndcp.z()/=ndcp.w();
        ndcp.x() = 0.5*ras.width*(ndcp.x()+1.0);
        ndcp.y() = 0.5*ras.height*(ndcp.y()+1.0);
        
        
        if (0<=ndcp.x()&&ndcp.x()<=ras.width&&0<=ndcp.y()&&ndcp.y()<=ras.height&&ndcp.w()>=0.1){
            int iy = std::min((int)ndcp.y(),ras.height-1);
            int ix = std::min((int)ndcp.x(),ras.width-1);
            

            int lightsz=5000;
            

            int testsz=lightsz/r;
            testsz=63;
            float dblocker=1;
            float dreceiver=1;
            int blockercnt=0;
            
            int samplecnt=20;
            if (ras.pcsson){
                //SPY("blockerTest");
                auto tmpdiv=testsz/(double)UINT32_MAX;
                for (int j=0;j<samplecnt;++j){
                    auto tmp=xorshf64();
                    int rx=ix+(tmp>>32)*tmpdiv-(testsz>>1);
                    int ry=iy+(tmp%UINT32_MAX)*tmpdiv-(testsz>>1);
                    auto ind = (ras.height-1-ry)*ras.width + rx;
                    if (ind<0||ind>=smap[i].size()) continue;
                    
                    auto sample = smap[i][ind];
                    auto depth = -10.0/(sample*(49.9)-(50.1));//ndcz 2 -z
                    if (sample+bias<=ndcp.z()){
                        dblocker+=depth;
                        blockercnt++;
                    }
                }
                
                // for (int j=-testsz/2;j<testsz-testsz/2;j+=4){
                //     for (int k=-testsz/2;k<testsz-testsz/2;k+=4){
                //         //if (rseed=xorshf32(rseed);rseed>0.9) continue;
                //         int ny=iy+j;
                //         int nx=ix+k;
                //         auto ind = (ras.height-1-ny)*ras.width + nx;
                //         if (ind<0||ind>=smap[i].size()) continue;
                        
                //         auto sample = smap[i][ind];

                //         if (sample+bias<=ndcp.w()){
                //             dblocker+=sample;
                //             blockercnt++;
                //         }
                        
                //     }
                // }


                dreceiver =  ndcp.w();
                dblocker/=blockercnt;

            }
            //SPY("Conv");
            int filtersz=std::max(1,int((dreceiver - dblocker)*125/dblocker));
            auto ads=ambient+diffuse+specular;
            int cntshadow=0,cntlight=0;
            for (int j=-filtersz/2;j<filtersz-filtersz/2;++j){
                for (int k=-filtersz/2;k<filtersz-filtersz/2;++k){
                    int ny=iy+j;
                    int nx=ix+k;
                    auto ind = (ras.height-1-ny)*ras.width + nx;
                    if (ind<0||ind>=smap[i].size()) continue;
                    auto sample = smap[i][ind];
                    if (sample+bias<=ndcp.z()){
                        //result_color=result_color+ambient/(fsz2);
                        cntshadow++;
                    }else{
                        cntlight++;
                    }
                }
            }
            result_color+=ambient*(double)cntshadow/(cntshadow+cntlight)+ads*(double)cntlight/(cntshadow+cntlight);
            // samplecnt=filtersz;
           
            // for (int j=0;j<samplecnt;++j){
            //     auto tmp=xorshf64();
            //     int rx=ix+(tmp>>32)/(double)UINT32_MAX*filtersz-filtersz/2;
            //     int ry=iy+(tmp%UINT32_MAX)/(double)UINT32_MAX*filtersz-filtersz/2;
            //     auto ind = (ras.height-1-ry)*ras.width + rx;
            //     if (ind<0||ind>=smap[i].size()) continue;
            //     auto sample = smap[i][ind];
            //         if (sample+bias<=ndcp.w()){
            //             result_color=result_color+ambient/(samplecnt);
            //         }else{
            //             result_color=result_color+(ads)/(samplecnt);
            //         }
            // }


                
                
            
            
        }    
    }
    
  



    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    auto n = normal.normalized();
    auto t = Eigen::Vector3f(
        n.x() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()),
        sqrt(n.x() * n.x() + n.z() * n.z()),
        n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z())
    );
    auto b = n.cross(t);

    auto tbn_mat = Eigen::Matrix3f();
    tbn_mat << t.x(), b.x(), n.x(),
    t.y(), b.y(), n.y(),
    t.z(), b.z(), n.z();

    auto tex_height = payload.texture->height;
    auto tex_width = payload.texture->width;
    float u = payload.tex_coords.x() - floor(payload.tex_coords.x());
    float v = payload.tex_coords.y() - floor(payload.tex_coords.y());

    auto dU = kh * kn * (payload.texture->getColor(u + 1.0f / tex_width, v).norm() - payload.texture->getColor(u, v).norm());
    auto dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / tex_height).norm() - payload.texture->getColor(u, v).norm());

    auto ln = Eigen::Vector3f(-dU, -dV, 1);
    normal = (tbn_mat*ln).normalized();
    point = point+kn*n*payload.texture->getColor(u, v).norm();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        auto ambient =  ka.cwiseProduct(amb_light_intensity);
        float r = (point-light.position).norm();
        auto ldir = (light.position-point).normalized();
        auto I =light.intensity/(r*r);
        auto diffuse = kd.cwiseProduct(I*std::max(0.0f,normal.dot(ldir)));
        auto h = ((-point).normalized()+ldir).normalized();
        auto specular = ks.cwiseProduct(I*std::pow(std::max(0.0f,normal.dot(h)),p));
        result_color=result_color+ambient+diffuse+specular;     

    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    auto n = normal.normalized();
    auto t = Eigen::Vector3f(
        n.x() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()),
        sqrt(n.x() * n.x() + n.z() * n.z()),
        n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z())
    );
    auto b = n.cross(t);

    auto tbn_mat = Eigen::Matrix3f();
    tbn_mat << t.x(), b.x(), n.x(),
    t.y(), b.y(), n.y(),
    t.z(), b.z(), n.z();

    auto tex_height = payload.texture->height;
    auto tex_width = payload.texture->width;
    float u = payload.tex_coords.x() - floor(payload.tex_coords.x());
    float v = payload.tex_coords.y() - floor(payload.tex_coords.y());

    auto dU = kh * kn * (payload.texture->getColor(u + 1.0f / tex_width, v).norm() - payload.texture->getColor(u, v).norm());
    auto dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / tex_height).norm() - payload.texture->getColor(u, v).norm());

    auto ln = Eigen::Vector3f(-dU, -dV, 1);
    normal = (tbn_mat*ln).normalized();
    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}
std::vector<Triangle*> TriangleList,PanelTriangleList;

float langle[2]={-45,45};
float angle = 140.0;
float angle2 = 0;
float angleud = 0;
bool command_line = false;

std::string filename = "output.png";
objl::Loader Loader;
std::string obj_path = "../models/spot/";

// Load .obj File
bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");

Quaternionf cam_quat(1,0,0,0);
Eigen::Vector3f eye_pos = {0,0,10};
int key = 0;


void RenderThread(){
    while(1)
    {
        {    
            SPY("RenderLoop");
            r.clear(rst::Buffers::Color | rst::Buffers::Depth);
            auto halfa = angle * MY_PI / 360.f;
            auto cosah = cos(halfa);
            auto sinah = sin(halfa);
            auto quata = Quaternionf(cosah,0,sinah,0);
            auto quatb = Quaternionf(cosah,0,-sinah,0);
            static Matrix4f m1,mview;
            m1.setIdentity();
            m1.block<3,3>(0,0) = 2.5*quata.toRotationMatrix();
            m1.block<3,1>(0,3) = Vector3f(-2,0,0);
            r.set_model(m1);
            mview.setIdentity();
            auto halfa2 = angle2 * MY_PI / 360.f;
            auto halfb2 = angleud * MY_PI / 360.f;
            cam_quat = Quaternionf(cos(halfa2),0,sin(halfa2),0)*Quaternionf(cos(halfb2),sin(halfb2),0,0);
            auto tmpconj = cam_quat.conjugate();
            mview.block<3,3>(0,0) =tmpconj.toRotationMatrix();
            mview.block<3,1>(0,3) = tmpconj*-(cam_quat*eye_pos);
            //mview.block<3,1>(0,3) = tmpconj*-eye_pos;
            //mview=get_view_matrix(eye_pos);
            int lidx=0;
            for (auto la:langle){
                auto halfla1 = la * MY_PI / 360.f;
                const static Quaternionf q1(cos(-MY_PI/8.0),sin(-MY_PI/8.0),0,0);
                Quaternionf q2(cos(halfla1),0,sin(halfla1),0);
                auto tmpq=q2*q1;
                Vector3f pos = tmpq*Vector3f(0,0,30);
                r.getlights()[lidx].position=pos;
                r.getlights()[lidx].orien=tmpq;
                lidx++;
            }
            
            
            
            r.set_view(mview);
            r.UpdateLightPass();
            r.lightprojection=get_projection_matrix(30, 1, 0.1, 50);
            r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
            // std::cout << angle2 << "     \r";
            //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
            // r.draw(TriangleList);
            r.multidraw(TriangleList);
            r.AddTriangleForShadow(TriangleList);
            m1.setIdentity();
            r.set_model(m1);
            r.multidraw(PanelTriangleList);
            r.AddTriangleForShadow(PanelTriangleList);
            m1.block<3,3>(0,0) = 2.5*quatb.toRotationMatrix();
            m1.block<3,1>(0,3) = Vector3f(2,0,0);
            r.set_model(m1);
            r.multidraw(TriangleList);
            r.AddTriangleForShadow(TriangleList);
            // r.draw(TriangleList);
            {
                SPY("Rasterization");
                r.multiras();
                pp.BloomPass();
            }
            
            //std::this_thread::sleep_for(std::chrono::milliseconds(500));
            {
                SPY("SWAP");
                r.Swapbuffer();
            }
        }
        Profiler::Update();
        r.fps=Profiler::Frm_cnter_/Profiler::timer_["RenderLoop"];
        
        Profiler::PrintInfo();
    }
}

void CVshowthread(){
    while(1)
    {
            //SPY("CVLOOP");
           
            cv::Mat image;
            float fps;
            r.Frm2cv(image,fps);
            image.convertTo(image, CV_8UC3, 1.0f);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            static std::stringstream ss;
            ss.str("");
            ss<<"fps: "<<fps;
            auto txt = ss.str();
            cv::putText(image, txt, cv::Point2f(50,50), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 122, 22),2);
            //Profiler::PrintInfoOnMat(image);
            cv::imshow("image", image);
            //cv::imwrite(filename, image);
            
            {
                //SPY("CVwaitkey cost");
                key = cv::waitKey(1);

                
                switch(key){
                    case 'b':
                        pp.switchbloom();
                        break;

                    case 'p':
                        r.switchpcss();
                        break;

                    case 'o':
                        r.switchshadow();
                        break;

                    case 'a':
                        angle -= 2;
                        break;

                    case 'd':
                        angle += 2;
                        break;

                    case 'q':
                        angle2 -= 2;
                        break;

                    case 'e':
                        angle2 += 2;
                        break;

                    case 'w':
                        angleud -= 2;
                        break;

                    case 's':
                        angleud += 2;
                        break;
                    
                    case '1':
                        langle[0] -= 2;
                        break;

                    case '2':
                        langle[0] += 2;
                        break;

                    case '3':
                        langle[1] -= 2;
                        break;

                    case '4':
                        langle[1] += 2;
                        break;

                }


            }
        }
}


int main(int argc, const char** argv)
{
   
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    
    float w=5,h=5,hght=-1.8;
    Vector4f panelvertex[]={{w,hght,h,1},{w,hght,-h,1},{-w,hght,-h,1},{-w,hght,h,1}};
    {
        Triangle* tmpt = new Triangle();
        for (int i=0;i<3;++i){
            tmpt->setVertex(i, panelvertex[i]);
            tmpt->setNormal(i, Vector3f(0,1,0));
            tmpt->setTexCoord(i, Vector2f(-1,-1));
        }
        PanelTriangleList.push_back(tmpt);
    }
    {
        Triangle* tmpt = new Triangle();
        for (int i=0;i<3;++i){
            tmpt->setVertex(i, panelvertex[(2+i)%4]);
            tmpt->setNormal(i, Vector3f(0,1,0));
            tmpt->setTexCoord(i, Vector2f(-1,-1));
        }
        PanelTriangleList.push_back(tmpt);
    }




    auto texture_path = "spot_texture.png";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);
    r.preparerasthreads();
    
    int frame_count = 0;

    // if (command_line)
    // {
    //     r.clear(rst::Buffers::Color | rst::Buffers::Depth);
    //     r.set_model(get_model_matrix(angle));
    //     r.set_view(get_view_matrix(eye_pos));
    //     r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

    //     r.draw(TriangleList);
    //     cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    //     image.convertTo(image, CV_8UC3, 1.0f);
    //     cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    //     cv::imwrite(filename, image);

    //     return 0;
    // }

    
    Quaternionf q1(cos(-MY_PI/8.0),sin(-MY_PI/8.0),0,0);
    Quaternionf q2(cos(MY_PI/4),0,sin(MY_PI/4),0);
    auto tmpq=q1;
    Vector3f pos = tmpq*Vector3f(0,0,30);
    r.AddLight({pos,tmpq,{500, 500, 500}});
    r.AddLight({pos,tmpq,{500, 500, 500}});

    auto profiler = Profiler();

    std::thread renderloop(RenderThread);
    std::thread viewloop(CVshowthread);

    renderloop.join();
    viewloop.join();

    return 0;

    // while(1)
    // {   
    //     {
    //         SPY("RenderLoop");
    //         r.clear(rst::Buffers::Color | rst::Buffers::Depth);
    //         auto halfa = angle * MY_PI / 360.f;
    //         auto cosah = cos(halfa);
    //         auto sinah = sin(halfa);
    //         auto quata = Quaternionf(cosah,0,sinah,0);
    //         auto quatb = Quaternionf(cosah,0,-sinah,0);
    //         static Matrix4f m1,mview;
    //         m1.setIdentity();
    //         m1.block<3,3>(0,0) = 2.5*quata.toRotationMatrix();
    //         m1.block<3,1>(0,3) = Vector3f(-2,0,0);
    //         r.set_model(m1);
    //         mview.setIdentity();
    //         auto tmpconj = cam_quat.conjugate();
    //         mview.block<3,3>(0,0) =tmpconj.toRotationMatrix();
    //         mview.block<3,1>(0,3) = tmpconj*-(cam_quat*eye_pos);
    //         r.set_view(mview);
    //         r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
            
    //         //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
    //         r.draw(TriangleList);

    //         m1.setIdentity();
    //         m1.block<3,3>(0,0) = 2.5*quatb.toRotationMatrix();
    //         m1.block<3,1>(0,3) = Vector3f(2,0,0);
    //         r.set_model(m1);
    //         //r.draw(TriangleList);
    //     }


    //     {
    //         SPY("CVLOOP");
    //         cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    //         image.convertTo(image, CV_8UC3, 1.0f);
    //         cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    //         PrintFps(image);
    //         cv::imshow("image", image);
    //         //cv::imwrite(filename, image);
            
    //         {
    //             SPY("CVwaitkey cost");
    //             key = cv::waitKey(1);

    //             if (key == 'a' )
    //             {
    //                 angle -= 0.5;
    //             }
    //             else if (key == 'd')
    //             {
    //                 angle += 0.5;
    //             }
    //             else if(key=='q'){
                    
    //                 cam_quat*=Quaternionf(cos(0.01),0,sin(0.01),0);
    //             }else if(key=='e'){
    //                 cam_quat*=Quaternionf(cos(-0.01),0,sin(-0.01),0);
    //             }
    //         }
    //     }
    //     Profiler::Update();
    //     Profiler::PrintInfo();

    // }
    // return 0;
}
