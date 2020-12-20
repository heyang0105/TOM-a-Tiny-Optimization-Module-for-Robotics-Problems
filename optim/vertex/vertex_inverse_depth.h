#ifndef OPTIM_VERTEX_INVERSE_DEPTH_H
#define OPTIM_VERTEX_INVERSE_DEPTH_H

#include "optim/core/vertex.h"

namespace Optim {

class VertexInverseDepth : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexInverseDepth(const size_t& id) : Vertex(id, 1, 1) {}
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_; }
};

}
#endif
