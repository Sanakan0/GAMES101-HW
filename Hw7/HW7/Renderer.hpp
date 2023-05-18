//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"
#include <mutex>

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);
    void ParallelRender(const Scene& scene);
    void SampleThread(const Scene& scene,std::vector<Vector3f>& framebuffer,float scale,float imageAspectRatio,Vector3f eye_pos);
private:
    std::mutex samplemutex;
};
