//
// Created by goksu on 4/6/19.
//

#pragma once

#include <eigen3/Eigen/Eigen>
#include <optional>
#include <algorithm>
#include "global.hpp"
#include "Shader.hpp"
#include "Triangle.hpp"

using namespace Eigen;

namespace rst
{
    struct gbuf{
        float alpha=0,beta=0,gamma=0;
        Vector3f color[3];
        Vector3f normal[3];
        Vector2f tex_coords[3];
        Vector3f view_pos[3];
        Texture* texture=nullptr;
    };

    enum class Buffers
    {
        Color = 1,
        Depth = 2
    };

    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int)a | (int)b);
    }

    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int)a & (int)b);
    }

    enum class Primitive
    {
        Line,
        Triangle
    };

    /*
     * For the curious : The draw function takes two buffer id's as its arguments. These two structs
     * make sure that if you mix up with their orders, the compiler won't compile it.
     * Aka : Type safety
     * */
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    struct ind_buf_id
    {
        int ind_id = 0;
    };

    struct col_buf_id
    {
        int col_id = 0;
    };

    struct Light
    {
        Eigen::Vector3f position;
        Quaternionf orien;
        Eigen::Vector3f intensity;
        Matrix4f mat;
        Vector3f pos_in_view;
    };


    class rasterizer
    {
    public:
        rasterizer(int w, int h);
        pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
        ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
        col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);
        col_buf_id load_normals(const std::vector<Eigen::Vector3f>& normals);
        void gbufshadingthread(int st,int ed);
        void shadingthread( int xmin, int ymin, int xmax, int ymax);
        void lightpass();
        void multiras();
        void preparerasthreads();
        void set_model(const Eigen::Matrix4f& m);
        void set_view(const Eigen::Matrix4f& v);
        void set_projection(const Eigen::Matrix4f& p);
        void UpdateLightPass();
        void set_texture(Texture tex) { texture = tex; }
        void StartRenderThreads();
        void WaitRenderThreads();
        void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
        void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);

        void set_pixel(const Vector2i &point, const Eigen::Vector3f &color);

        void clear(Buffers buff);

        void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
        void draw(std::vector<Triangle *> &TriangleList);

        void multidraw(std::vector<Triangle *> &TriangleList);
        void AddTriangleForShadow(std::vector<Triangle *> &TriangleList);
        std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf[backbufidx^1]; }
        void Swapbuffer(){
            std::lock_guard<std::mutex> lock(frmmutex);
            backbufidx^=1;
        }
        void Frm2cv(cv::Mat& mat,float& fpsp){
            std::lock_guard<std::mutex> lock(frmmutex);
            cv::Mat tmp(700, 700, CV_32FC3, frame_buf[backbufidx^1].data());
            tmp.copyTo(mat);
            fpsp=fps;
        }
        void InitThreads();
        void AddLight(const Light& light);
        std::vector<Light>& getlights(){return lights;}
        const std::vector<std::vector<float>>& getshadowmap(){return light_depth_buf;}
        volatile int backbufidx=0;
        bool msaa_sw=false;
        float fps=0;
        Eigen::Matrix4f model;
        Eigen::Matrix4f view,invview;
        Eigen::Matrix4f projection,lightprojection;
        int width, height;
        bool shadowon=false;
        bool switchbtn=false;
        bool pcsson=false;
        bool pcssbtn=false;
        void switchshadow(){
            std::unique_lock<std::mutex> lock(sw_m);
            switchbtn=true;
        }
        void switchpcss(){
            std::unique_lock<std::mutex> lock(sw_m);
            pcssbtn=true;
        }
    private:
        void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
        
        void rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos);
        void rasterize_t(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos,int minx,int miny,int maxx,int maxy);
        void rasterize_shadowmap(const std::array<Eigen::Vector4f, 3>& view_pos,int xmin,int ymin,int xmax,int ymax);
        
        // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER
        void rasterize_task();
    private:
        

        int normal_id = -1;

        std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
        std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
        std::map<int, std::vector<Eigen::Vector3f>> col_buf;
        std::map<int, std::vector<Eigen::Vector3f>> nor_buf;

        std::optional<Texture> texture;

        std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;
        std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;

        std::vector<Eigen::Vector3f> frame_buf[2];
        std::vector<gbuf> g_buf;
        std::vector<std::vector<float>> light_depth_buf;
        std::vector<std::thread> renderthreads;
        std::vector<float> depth_buf[2];
        int get_index(int x, int y);

        

        int next_id = 0;
        int get_next_id() { return next_id++; }
        std::mutex frmmutex;
        std::mutex cv_m,cv_m2;
        std::mutex sw_m;
        std::condition_variable cv,cv2;
        int workercnt=0;
        int workleft=0;

        std::vector<Light> lights;

        std::vector<Triangle> triangle_buf;
        std::vector< std::vector<std::array<Eigen::Vector4f, 3>>> shadow_triangle_buf;
        std::vector<std::array<Eigen::Vector3f, 3>> view_pos_buf;

        int cur_light_id=-1;
    };
}
