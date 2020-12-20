#ifndef OPTIM_VERTEX_POINTVERTEX_H
#define OPTIM_VERTEX_POINTVERTEX_H

#include "optim/core/vertex.h"


namespace Optim {

class VertexPointXYZ : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPointXYZ(const size_t& id) : Vertex(id, 3, 3) {}
    
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_; }
};

}


#endif
