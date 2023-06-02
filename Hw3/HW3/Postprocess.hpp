#pragma once

#include "rasterizer.hpp"
class Postprocesser{
public:
    Postprocesser(rst::rasterizer& ras):ras_(ras){}

    void BloomPass();
    void FragFilter();
    void FragGblurH();
    void FragGblurV();
    void Upsample();

private:
    rst::rasterizer& ras_;


};