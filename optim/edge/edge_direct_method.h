#ifndef OPTIM_EDGE_DIRECT_METHOD_H
#define OPTIM_EDGE_DIRECT_METHOD_H

#include "optim/core/eigen_types.h"
#include "optim/core/edge.h"

#include "optim/vertex/vertex_pose.h"


namespace Optim {

typedef unsigned char uchar;

class EdgeMonoDirectMethod : public Edge{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeMonoDirectMethod(const Vec3 &landmark_ref, const size_t& id):
        Edge(id, 1, 1, std::vector<std::string>{VertexPose::type_name_}),
        landmark_ref_(landmark_ref){}

    void SetPatchSize(size_t patch_size);
    bool SetImageDate(uchar* data, size_t width, size_t height);
    void SetCamIntrinsics(double fx, double fy, double cx, double cy);
    //
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_;} 

    virtual void ComputeResidual() override;
    virtual void ComputeJacobians() override;

    // all the edges share the same values
    static size_t patch_size_;
    
    static uchar* data_;
    static size_t width_, height_ ;

    static double fx_;
    static double fy_;
    static double cx_;
    static double cy_;

private:

    inline double GetPixelValue(const Vec2& pos);
    inline Vec2 CamProject(const Vec3& pts);
    //
    Vec3 landmark_ref_; //project to ref and cur， compare the Intensity diff
    //Vec2 obs_;
}; 

}// namespace

#endif
