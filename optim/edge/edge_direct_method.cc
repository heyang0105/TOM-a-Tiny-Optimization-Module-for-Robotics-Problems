#include"optim/edge/edge_direct_method.h"

namespace Optim{



void EdgeMonoDirectMethod::SetPatchSize(size_t patch_size){
    size_t patch_size_ = patch_size;
}

bool EdgeMonoDirectMethod::SetImageDate(uchar* data, 
    size_t width, size_t height){
    uchar* data_ = data;
    size_t width_= width;
    size_t height_ = height;
}

void EdgeMonoDirectMethod::SetCamIntrinsics(double fx, double fy,
                     double cx, double cy){
    double fx_ = fx; 
    double fy_ = fy;
    double cx_ = cx;
    double cy_ = cy;
}

inline double EdgeMonoDirectMethod::GetPixelValue(const Vec2& pos){

}

inline Vec2 EdgeMonoDirectMethod::CamProject(const Vec3& pts){

}

void EdgeMonoDirectMethod::ComputeResidual(){

}

void EdgeMonoDirectMethod::ComputeJacobians(){

}

}//namespace