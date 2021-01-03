#ifndef OPTIM_EDGE_DIRECT_METHOD_H
#define OPTIM_EDGE_DIRECT_METHOD_H

#include "optim/sophus/se3.hpp"
#include "optim/core/eigen_types.h"
#include "optim/core/edge.h"

#include "optim/vertex/vertex_pose.h"


namespace Optim {

typedef unsigned char uchar;

// first, the single pxiel direct method, than expand it to the patch type

class EdgeMonoDirectMethod : public Edge{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeMonoDirectMethod(const double& ref_grayscale, const Vec3 &landmark_ref, const size_t& id):
        Edge(id, 1, 1, std::vector<std::string>{VertexPose::type_name_}),
        ref_grayscale_(ref_grayscale),
        landmark_ref_(landmark_ref){}

    // void SetHalfPatchSize(size_t half_patch_size);
    // the image should be in gray scale
    bool SetImageDate(uchar* cur_img_data, size_t width, size_t height);
    void SetCamIntrinsics(double fx, double fy, double cx, double cy);
    //
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_;} 

    virtual void ComputeResidual() override;
    virtual void ComputeJacobians() override;

    // all the edges share the same values
    // static size_t half_patch_size_;
    
    static uchar* cur_img_data_;
    static size_t width_, height_ ;

    static double fx_;
    static double fy_;
    static double cx_;
    static double cy_;
	static Mat33 K_;

private:

    inline double GetPixelValue(uchar* img_data,
     const int& rows, const int& cols,
     const double& in_x, const double& in_y);
    
    inline Vec2 CamProject(const Vec3& pts);
    
    // measurement
    double ref_grayscale_;
    Vec3 landmark_ref_; //project to ref and cur， compare the Intensity diff
    //Vec2 obs_;
}; 

}// namespace

#endif
