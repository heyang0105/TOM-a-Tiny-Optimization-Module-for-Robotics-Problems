#ifndef OPTIM_VERTEX_POSEVERTEX_H
#define OPTIM_VERTEX_POSEVERTEX_H

//#include <iostream>
#include <memory>
#include <string>
#include "optim/core/vertex.h"

namespace Optim {

// NOTE:
//  1. tx, ty, tz, qw, qx, qy, qz
//  2. pose is represented as T_c_w
class VertexPose : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPose(const size_t& id) : Vertex(id, 7, 6){}

    virtual void Plus(const VecX &delta) override;
    
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_;} 
};


}
#endif
