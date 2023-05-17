//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{   

    


    //

    Vector3f ans(0,0,0);
    // TO DO Implement Path Tracing Algorithm here
    ///get intersect,
    if (depth > maxDepth){
        return ans;
    }
    auto inter = intersect(ray);
    if (inter.happened==false) return ans;
    ///half sphere monte carlo query
    int samplecnt =16;
    
    for (int i=0;i<samplecnt;++i){
        Intersection lightinter;
        float lpdf;
        sampleLight(lightinter,lpdf);
       // std::cout << lightinter.coords << std::endl;
        auto ldir = lightinter.coords-inter.coords;
        auto nldir = normalize( ldir);
        auto ldist = ldir.norm();
        auto ray2light = Ray(inter.coords,nldir);
        auto tmpinter = intersect(ray2light);
        
        

        if (abs(tmpinter.distance-ldist)<0.0001){ //DIR light
            // std::cout << lightinter.coords << std::endl;
            // std::cout << tmpinter.coords << std::endl;
            //std::cout << "fk" <<std::endl;

            ans+=lightinter.emit*inter.m->eval(-ray2light.direction, -ray.direction, inter.normal)*dotProduct(inter.normal, ray2light.direction)*dotProduct(lightinter.normal, -ray2light.direction)/(ldist*ldist)/lpdf;
            
        }
        


        // auto outray = Ray(inter.coords,inter.m->sample(ray.direction, inter.normal));
        // auto pdf = inter.m->pdf(ray.direction, outray.direction ,inter.normal);
        // ans+=castRay(outray, depth+1)*inter.m->eval(-outray.direction, -ray.direction, inter.normal)*dotProduct(inter.normal, -outray.direction)/pdf/RussianRoulette;
    }
    ans=ans/(float)samplecnt;

    ///calc color





    return ans;
}