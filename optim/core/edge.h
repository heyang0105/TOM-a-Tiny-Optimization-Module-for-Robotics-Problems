#ifndef OPTIM_CORE_EDGE_H
#define OPTIM_CORE_EDGE_H

#include <memory>
#include <string>
#include "optim/core/eigen_types.h"
#include <eigen3/Eigen/Dense>
#include "optim/core/loss_function.h"

namespace Optim {

class Vertex;

class Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit Edge(size_t id,
                  int residual_dimension, 
                  int num_verticies,
      const std::vector<std::string> &verticies_types = 
                           std::vector<std::string>());

    virtual ~Edge();        

    // ---1. get
    
    size_t Id() const;
    
    std::shared_ptr<Vertex> GetVertex(const int& i);

    std::vector<std::shared_ptr<Vertex>> Verticies() const;

    size_t NumVertices() const;
    
    VecX Residual() const;

    VecMatXX Jacobians() const;

    MatXX Information() const;

    MatXX SqrtInformation() const;

    LossFunction* GetLossFunction() const;

    VecX Observation() const;

    int OrderingId() const;

    // ---2. set

    inline void SetId(size_t id) {id_ = id;}

    void AddVertex(std::shared_ptr<Vertex> vertex);

    void SetVertices(const std::vector<std::shared_ptr<Vertex>> &vertices);

    void SetInformation(const MatXX &information);

    void SetLossFunction(LossFunction* ptr);

    void SetObservation(const VecX &observation);

    void SetOrderingId(int id);

    // ---3. calc
    double Chi2() const;

    double RobustChi2() const;

    void RobustInfo(double& drho, MatXX& info) const;

    bool CheckValid();

    // ---4.virtual
    virtual std::string TypeInfo() const = 0;

    virtual void ComputeResidual() = 0;

    virtual void ComputeJacobians() = 0;    

protected:
    size_t id_;  
    size_t ordering_id_;   //edge id in problem
    std::vector<std::string> verticies_types_; 
    std::vector<std::shared_ptr<Vertex>> verticies_; 
    VecX residual_;                 
    VecMatXX jacobians_;  // residual x vertex[i]
    MatXX information_;             
    MatXX sqrt_information_;
    VecX observation_ ;              

    LossFunction *lossfunction_ = nullptr;

// public:
//     static size_t global_edge_id_ ;
};

//--------------------------Impletation------------------------
// ---1. get
inline size_t Edge::Id() const { return id_; }

inline std::shared_ptr<Vertex> Edge::GetVertex(const int& i) {
    return verticies_[i];
}

inline std::vector<std::shared_ptr<Vertex>> Edge::Verticies() const {
    return verticies_;
}

inline size_t Edge::NumVertices() const { return verticies_.size(); }

inline VecX Edge::Residual() const { return residual_; }

inline VecMatXX Edge::Jacobians() const { return jacobians_; }

inline MatXX Edge::Information() const {return information_;}

inline MatXX Edge::SqrtInformation() const { return sqrt_information_;}

inline LossFunction* Edge::GetLossFunction() const{ return lossfunction_;}

inline VecX Edge::Observation() const { return observation_; }

inline int Edge::OrderingId() const { return ordering_id_; }

// ---2. set
inline void Edge::AddVertex(std::shared_ptr<Vertex> vertex) {
    // size_t cur_num = verticies_.size() + 1;
    // if(verticies_types_[cur_num] == vertex->TypeInfo()){
    //     verticies_.emplace_back(vertex);
    // }
    // else{
    //     assert("wrong order of vertex in edge");
    // }
    verticies_.emplace_back(vertex);
}

inline void Edge::SetVertices(const std::vector<std::shared_ptr<Vertex>> &vertices) {
    // bool is_in_order = true;
    // size_t n = 0;
    // for(auto& v : vertices){
    //     if(v->TypeInfo() != verticies_types_[n]){
    //         is_order = false;
    //         break;
    //     }
    // }
    // if(is_in_order){
    //     verticies_ = vertices;
    // }
    // else{
    //     assert("wrong order of vertex in edge");
    // }    
     verticies_ = vertices;
}

inline void Edge::SetInformation(const MatXX &information) {
    information_ = information;
    sqrt_information_ = Eigen::LLT<MatXX>(information_).matrixL().transpose();
}

inline void Edge::SetLossFunction(LossFunction* ptr){ lossfunction_ = ptr; }

inline void Edge::SetObservation(const VecX &observation) {
    observation_ = observation;
}

} //namespace


#endif
