#ifndef OPTIM_PROBLEM_SPARSE_VO_H
#define OPTIM_PROBLEM_SPARSE_VO_H

#include"optim/core/problem_base.h"

namespace Optiom{

class ProblemSparseVO : public ProblemBase{
public:
    ProblemSparseVO(const ProblemBase::Problem_Config problem_config):
      ProblemBase(problem_config){}

    virtual bool Solve() override; 
private:


};

}//namespace
#endif