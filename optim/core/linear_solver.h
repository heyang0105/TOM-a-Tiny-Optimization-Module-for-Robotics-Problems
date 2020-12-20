// 2020/12/14

#ifndef OPTIM_CORE_LINEAR_SOLVER_H
#define OPTIM_CORE_LINEAR_SOLVER_H

#include"optim/core/eigen_types.h"

// TODO: 
//    1. using Eigen::SparseMatrix to add some classes
//    2. CUDA solver

namespace Optim{

namespace internal{
    void CheckMatrixDimension(const MatXX& H, const VecX& b);
}

class DenseLinearSolverBase{
public:
    DenseLinearSolverBase(){} 
    virtual void Solve(const MatXX& H, const VecX& b, VecX& x) = 0;
};

class CholeskySolver : public DenseLinearSolverBase{
public:
    CholeskySolver() :
        DenseLinearSolverBase(){}
    virtual void Solve(const MatXX& H, const VecX& b, VecX& x) override;
};

class QRSolver : public DenseLinearSolverBase{
public:
    QRSolver() :
        DenseLinearSolverBase(){}
    virtual void Solve(const MatXX& H, const VecX& b, VecX& x) override;
};

class JacobiPCGSolver : public DenseLinearSolverBase{
public:
    JacobiPCGSolver() :
        DenseLinearSolverBase(){
            max_iter_ = -1;}

    void SetMaxIteration(const size_t& iter_num){
        max_iter_ = iter_num;
    }

    virtual void Solve(const MatXX& H, const VecX& b, VecX& x) override;
private:
    int max_iter_;
};

}// namespace
#endif