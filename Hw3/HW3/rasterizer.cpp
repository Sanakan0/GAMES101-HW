//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include "Profiler.hpp"

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Vector4f* _v){
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
        return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {
    
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;
    Eigen::Matrix4f mv = view * model;
    Eigen::Matrix4f mvp = projection * mv;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (mv * t->v[0]),
                (mv * t->v[1]),
                (mv * t->v[2])
        };
        
        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };


        

        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        //back cull use ndc coord
        auto veca = (v[1]-v[0]).head<3>();
        auto vecb = (v[2]-v[0]).head<3>();
        auto trivec = veca.cross(vecb);
        if (trivec.z()<0) {
            continue;
        }

        Eigen::Vector4f n[] = {
                mv * to_vec4(t->normal[0], 0.0f),
                mv * to_vec4(t->normal[1], 0.0f),
                mv * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

void rst::rasterizer::multidraw(std::vector<Triangle *> &TriangleList) {
    
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;
    Eigen::Matrix4f mv = view * model;
    Eigen::Matrix4f mvp = projection * mv;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (mv * t->v[0]),
                (mv * t->v[1]),
                (mv * t->v[2])
        };
        
        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };


        

        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        //back cull use ndc coord
        auto veca = (v[1]-v[0]).head<3>();
        auto vecb = (v[2]-v[0]).head<3>();
        auto trivec = veca.cross(vecb);
        if (trivec.z()<0) {
            continue;
        }

        Eigen::Vector4f n[] = {
                mv * to_vec4(t->normal[0], 0.0f),
                mv * to_vec4(t->normal[1], 0.0f),
                mv * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        triangle_buf.push_back(newtri);
        world_pos_buf.push_back(viewspace_pos);
        //rasterize_triangle(newtri, viewspace_pos);
    }
}


void rst::rasterizer::shadingthread(int xmin,int ymin,int xmax,int ymax){

    while(1){
        {
            
            std::unique_lock<std::mutex> lk(cv_m);

            cv.wait(lk,[&]{return workercnt;});
            workercnt--;
            //std::cout << xmin << " " <<ymin << " " <<xmax << " " <<ymax << "\n ";
        }
      
        for (int i=0;i<triangle_buf.size();++i){
            
            rasterize_t(triangle_buf[i], world_pos_buf[i],  xmin, ymin, xmax, ymax);
        }
        {
            std::unique_lock<std::mutex> lk(cv_m2);
            workleft--;
            cv2.wait(lk,[&]{return workleft==0;});
            //std::cout << workercnt<<" "<<workleft<<std::endl;
        }
        cv2.notify_all();
    }
}


void rst::rasterizer::preparerasthreads(){
    
    auto h=30;
    for (int i=0;i<height;i+=h+1){
        gbufthreads.emplace_back(&rst::rasterizer::shadingthread,this,0,i,width,std::min(i+h,height));
    }
}

void rst::rasterizer::multiras(){
    std::lock_guard<std::mutex> lock(frmmutex);
    {
        std::unique_lock<std::mutex> lk(cv_m);
        workercnt=gbufthreads.size();
        workleft=workercnt;
    }
    cv.notify_all();
    
    {
        std::unique_lock<std::mutex> lk(cv_m2);
        cv2.wait(lk,[&]{return workleft==0;});
    }
    //std::cout << workercnt<<" "<<workleft<<std::endl;
}


void rst::rasterizer::gbufshadingthread(int st,int ed){
    while(1){
        for (int i=st;i<ed;++i){
            auto& t = g_buf[i];
            auto alpha = g_buf[i].alpha;
            auto beta = g_buf[i].beta;
            auto gamma = g_buf[i].gamma;

            auto interpolated_color=t.color[0];
            auto interpolated_normal=alpha*t.normal[0] + beta*t.normal[1]+gamma*t.normal[2];
            auto interpolated_texcoords=alpha*t.tex_coords[0] + beta*t.tex_coords[1]+gamma*t.tex_coords[2];
            auto interpolated_shadingcoords=alpha*t.view_pos[0] + beta*t.view_pos[1]+gamma*t.view_pos[2];
            fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
            payload.view_pos = interpolated_shadingcoords;
            Vector3f pixel_color(255,255,255); 
            //g_buf[ind]=payload;
            //std::cout << alpha<<" "<<beta<<" "<<gamma <<std::endl;
            pixel_color= fragment_shader(payload);
            
            auto ix = i%width;
            auto iy = height-1- i/width;
           
            
            frame_buf[backbufidx][i] = pixel_color;
            //t=gbuf();
        }
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}


void rst::rasterizer::rasterize_t(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos,int xmin,int ymin,int xmax,int ymax){
   
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);

    auto& v = t.v; //screen space xyz, w=-z
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
   
    // culling
    


    float maxx = 0;
    float minx = width;
    float maxy = 0;
    float miny = height;
    for (auto& i:v){
        auto tmpx = i.x();
        auto tmpy = i.y();
        maxx=std::max(maxx,tmpx);
        minx=std::min(minx,tmpx);
        maxy=std::max(maxy,tmpy);
        miny=std::min(miny,tmpy);
    }
    maxx = std::min(maxx,(float)xmax);
    maxy = std::min(maxy,(float)ymax);
    minx = std::max(minx,(float)xmin);
    miny = std::max(miny,(float)ymin);

    // for (int ix=xmin;ix<=std::min((int)xmax,width-1);++ix){
    //         for (int iy=ymin;iy<=std::min((int)ymax,height-1);++iy){
    //             auto ind = (height-1-iy)*width + ix;
    //             frame_buf[backbufidx][ind] = {255,0,0};
    //             //set_pixel({ix,iy}, {255,0,0});
    //         }
    // }
    
    //std::cout <<minx<<" "<<maxx<<std::endl;
    if (1){
        //SPY("pixel");
        for (int ix=minx;ix<=std::min((int)maxx,width-1);++ix){
            for (int iy=miny;iy<=std::min((int)maxy,height-1);++iy){
                //SPY("pixin");
                //ProfilerSpy spy("pixin",1);
                auto cnter = Vector3f(ix+0.5,iy+0.5,0);
                Vector3f last(0,0,0);
                bool flag=0;
                    
                for (int i=0;i<3;++i){
                    auto& a = v[i];
                    auto& b =v[(i+1)%3];
                    Vector3f vec = (b-a).head<3>();
                    Vector3f vec2 = cnter-a.head<3>();
                    vec.z()=0;
                    vec2.z()=0;
                    Vector3f tmp = vec.cross(vec2);
                    if (last.dot( tmp)<0){
                        flag=1;
                        break;
                    }
                    last = tmp;
                }
                
                
                if (!flag){
                    
                    //SPY("flag");
                    auto[alpha, beta, gamma] = computeBarycentric2D(cnter.x(), cnter.y(), t.v);
                    alpha/=v[0].w();
                    beta/=v[1].w();
                    gamma/=v[2].w();
                    float sum =1.0/(alpha+beta+gamma);
                    alpha*=sum;
                    beta*=sum;
                    gamma*=sum;
                    float z_interpolated = alpha * v[0].z()  + beta * v[1].z()  + gamma * v[2].z();
                    //z_interpolated *= w_reciprocal;
                    auto ind = (height-1-iy)*width + ix;
                    
                    auto hisdepth = depth_buf[backbufidx][ind];
                    
                    if (z_interpolated<hisdepth){ //depth test
                        
                        //SPY("shading");
                        
                        depth_buf[backbufidx][ind]=z_interpolated;
                        Vector3f pixel_color(255,255,255); 
                        //g_buf[ind]=payload;

                        // set_pixel({ix,iy}, pixel_color);
                        // continue;
                        // g_buf[ind] = gbuf{alpha,beta,gamma,
                        // {t.color[0],t.color[1],t.color[2]},
                        // {t.normal[0],t.normal[1],t.normal[2]},
                        // {t.tex_coords[0],t.tex_coords[1],t.tex_coords[2]},
                        // {view_pos[0],view_pos[1],view_pos[2]}
                        // ,texture ? &*texture : nullptr};
                        
                        //auto interpolated_color=alpha*t.color[0] + beta*t.color[1]+gamma*t.color[2];
                        auto interpolated_color=t.color[0];
                        auto interpolated_normal=alpha*t.normal[0] + beta*t.normal[1]+gamma*t.normal[2];
                        auto interpolated_texcoords=alpha*t.tex_coords[0] + beta*t.tex_coords[1]+gamma*t.tex_coords[2];
                        auto interpolated_shadingcoords=alpha*view_pos[0] + beta*view_pos[1]+gamma*view_pos[2];
                        fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                        payload.view_pos = interpolated_shadingcoords;
                        //Vector3f pixel_color(255,255,255); 
                        //g_buf[ind]=payload;
                        pixel_color= fragment_shader(payload);
                        frame_buf[backbufidx][ind] = pixel_color;
                        //set_pixel({ix,iy}, pixel_color);
                    }
                }
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    SPY("Rasterizer");
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);

    auto& v = t.v; //screen space xyz, w=-z
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
   
    // culling
    


    float maxx = 0;
    float minx = width;
    float maxy = 0;
    float miny = height;
    for (auto& i:v){
        auto tmpx = i.x();
        auto tmpy = i.y();
        maxx=std::max(maxx,tmpx);
        minx=std::min(minx,tmpx);
        maxy=std::max(maxy,tmpy);
        miny=std::min(miny,tmpy);
    }
    maxx = std::min(maxx,(float)700);
    maxy = std::min(maxy,(float)500);
    minx = std::max(minx,0.0f);
    miny = std::max(miny,400.0f);

    
    if (1){
        //SPY("pixel");
        for (int ix=minx;ix<=std::min((int)maxx,width-1);++ix){
            for (int iy=miny;iy<=std::min((int)maxy,height-1);++iy){
                //SPY("pixin");
                //ProfilerSpy spy("pixin",1);
                auto cnter = Vector3f(ix+0.5,iy+0.5,0);
                Vector3f last(0,0,0);
                bool flag=0;
            
                    
                for (int i=0;i<3;++i){
                    auto& a = v[i];
                    auto& b =v[(i+1)%3];
                    Vector3f vec = (b-a).head<3>();
                    Vector3f vec2 = cnter-a.head<3>();
                    vec.z()=0;
                    vec2.z()=0;
                    Vector3f tmp = vec.cross(vec2);
                    if (last.dot( tmp)<0){
                        flag=1;
                        break;
                    }
                    last = tmp;
                }
                
                
                if (!flag){
                    
                    //SPY("flag");
                    auto[alpha, beta, gamma] = computeBarycentric2D(cnter.x(), cnter.y(), t.v);
                    alpha/=v[0].w();
                    beta/=v[1].w();
                    gamma/=v[2].w();
                    float sum =1.0/(alpha+beta+gamma);
                    alpha*=sum;
                    beta*=sum;
                    gamma*=sum;
                    float z_interpolated = alpha * v[0].z()  + beta * v[1].z()  + gamma * v[2].z();
                    //z_interpolated *= w_reciprocal;
                    auto ind = (height-1-iy)*width + ix;
                    
                    auto hisdepth = depth_buf[backbufidx][ind];
                    
                    if (z_interpolated<hisdepth){ //depth test
                        
                        //SPY("shading");
                        //std::cout << "fk"<<std::endl;
                        depth_buf[backbufidx][ind]=z_interpolated;
                        // g_buf[ind] = gbuf{alpha,beta,gamma,
                        // {t.color[0],t.color[1],t.color[2]},
                        // {t.normal[0],t.normal[1],t.normal[2]},
                        // {t.tex_coords[0],t.tex_coords[1],t.tex_coords[2]},
                        // {view_pos[0],view_pos[1],view_pos[2]}
                        // ,texture ? &*texture : nullptr};
                        
                        //auto interpolated_color=alpha*t.color[0] + beta*t.color[1]+gamma*t.color[2];
                        auto interpolated_color=t.color[0];
                        auto interpolated_normal=alpha*t.normal[0] + beta*t.normal[1]+gamma*t.normal[2];
                        auto interpolated_texcoords=alpha*t.tex_coords[0] + beta*t.tex_coords[1]+gamma*t.tex_coords[2];
                        auto interpolated_shadingcoords=alpha*view_pos[0] + beta*view_pos[1]+gamma*view_pos[2];
                        fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                        payload.view_pos = interpolated_shadingcoords;
                        Vector3f pixel_color(255,255,255); 
                        //g_buf[ind]=payload;
                        pixel_color= fragment_shader(payload);
                        set_pixel({ix,iy}, pixel_color);
                    }
                }
            }
        }
    }
    // else{
    //     //SSAA
    //     for (int ix=minx;ix<=std::min((int)maxx,width-1);++ix){
    //         for (int iy=miny;iy<=std::min((int)maxy,height-1);++iy){
    //             float alphaa=0;
    //             for (int k=0;k<4;k++){
    //                 static float mvx[]={0.25,0.75,0.25,0.75};
    //                 static float mvy[]={0.25,0.25,0.75,0.75};
    //                 auto cnter = Vector3f(ix+mvx[k],iy+mvy[k],0);
    //                 Vector3f last(0,0,0);
    //                 bool flag=0;
    //                 for (int i=0;i<3;++i){
    //                     auto& a = v[i];
    //                     auto& b =v[(i+1)%3];
    //                     Vector3f vec = (b-a).head<3>();
    //                     Vector3f vec2 = cnter-a.head<3>();
    //                     vec.z()=0;
    //                     vec2.z()=0;
    //                     Vector3f tmp = vec.cross(vec2);
    //                     if (last.dot( tmp)<0){
    //                         flag=1;
    //                         break;
    //                     }
    //                     last = tmp;
    //                 }
    //                 if (!flag){
    //                     auto[alpha, beta, gamma] = computeBarycentric2D(cnter.x(), cnter.y(), t.v);
    //                     float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                     float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //                     z_interpolated *= w_reciprocal;
    //                     auto ind = (height-1-iy)*width + ix;
    //                     auto hisdepth = depth_buf[backbufidx][ind];
    //                     if (z_interpolated<hisdepth){
                    
    //                         alphaa+=0.25;
    //                     }
            
                        
    //                 }
    //             }
    //             {
    //                 auto cnter = Vector3f(ix+0.5,iy+0.5,0);
    //                 Vector3f last(0,0,0);
    //                 bool flag=0;
    //                 for (int i=0;i<3;++i){
    //                     auto& a = v[i];
    //                     auto& b =v[(i+1)%3];
    //                     Vector3f vec = (b-a).head<3>();
    //                     Vector3f vec2 = cnter-a.head<3>();
    //                     vec.z()=0;
    //                     vec2.z()=0;
    //                     Vector3f tmp = vec.cross(vec2);
    //                     if (last.dot( tmp)<0){
    //                         flag=1;
    //                         break;
    //                     }
    //                     last = tmp;
    //                 }
    //                 if (!flag){
    //                     auto[alpha, beta, gamma] = computeBarycentric2D(cnter.x(), cnter.y(), t.v);
    //                     float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                     float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //                     z_interpolated *= w_reciprocal;
    //                     auto ind = (height-1-iy)*width + ix;
    //                     auto hisdepth = depth_buf[backbufidx][ind];
    //                     if (z_interpolated<hisdepth){
    //                         depth_buf[backbufidx][ind]=z_interpolated;
    //                         set_pixel(Vector3f(ix,iy,0), t.getColor());
    //                     }
                        
                        
    //                 }
    //             }
            
    //             if (alphaa>0){
    //                 Vector3f c(t.getColor().x()*alphaa,t.getColor().y()*alphaa,t.getColor().z()*alphaa);
    //                 set_pixel(Vector3f(ix,iy,0), c);
    //             }
                
    //         }   
    //     }
        
    // }
 
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf[backbufidx].begin(), frame_buf[backbufidx].end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf[backbufidx].begin(), depth_buf[backbufidx].end(), std::numeric_limits<float>::infinity());
    }
    triangle_buf.clear();
    world_pos_buf.clear();
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf[0].resize(w * h);
    frame_buf[1].resize(w * h);
    depth_buf[0].resize(w * h);
    depth_buf[1].resize(w * h);
    g_buf.resize(w*h);
    texture = std::nullopt;
    
}

void rst::rasterizer::InitThreads(){
    int threadscnt=4;
    return;
    auto step = width*height/threadscnt;
    for (int i=0;i<width*height;i+=step){
        gbufthreads.emplace_back(&rst::rasterizer::gbufshadingthread,this,i,std::min(i+step,width*height));
    }
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[backbufidx][ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

