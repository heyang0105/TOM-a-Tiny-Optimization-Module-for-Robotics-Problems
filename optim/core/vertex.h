#ifndef OPTIM_CORE_VERTEX_H
#define OPTIM_CORE_VERTEX_H

#include "optim/core/eigen_types.h"
#include <iostream>

namespace Optim {

class Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit Vertex(const size_t& id,
      int num_dimension, 
      int local_dimension = -1){
        parameters_.resize(num_dimension, 1);
        local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
        id_ = id;

        std::cout << "Vertex num_dimension: " << num_dimension
              << " local_dimension: " << local_dimension 
              << " id_: " << id_ << std::endl;
    }

    virtual ~Vertex(){}

    // --1.get
    inline int Dimension() const { return parameters_.rows();}
    inline int LocalDimension() const { return local_dimension_;};
    inline unsigned long Id() const { return id_; }
    inline VecX Parameters() const { return parameters_; }
    inline VecX &Parameters() { return parameters_; }
    inline int OrderingId() const { return ordering_id_; }
    bool IsFixed() const { return fixed_; }

    // --2.set
    inline void SetId(size_t id){id_ = id;}
    inline void SetParameters(const VecX &params) { parameters_ = params; }
    inline void SetOrderingId(unsigned long id) { ordering_id_ = id; }
    inline void SetFixed(bool fixed = true) {fixed_ = fixed;}
        // used in the LM strategy
    inline void BackUpParameters() { parameters_backup_ = parameters_; }
    inline void RollBackParameters() { parameters_ = parameters_backup_; }

    // --3. virtual
    virtual void Plus(const VecX &delta) { parameters_ += delta;}
    virtual std::string TypeInfo() const = 0;
    
protected:
    VecX parameters_;   
    VecX parameters_backup_; 
    int local_dimension_;   
    size_t id_;  
    size_t ordering_id_ = 0;
    bool fixed_ = false;    

// public:
//     static size_t global_vertex_id_ ;

};

}// namespace

#endif
