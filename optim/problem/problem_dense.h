#ifndef OPTIM_PROBLEM_DENSE_H
#define OPTIM_PROBLEM_DENSE_H

#include"optim/core/problem_base.h"

namespace Optim{

class ProblemDense : public ProblemBase{
public:
    ProblemDense(const ProblemBase::Problem_Config problem_config):
      ProblemBase(problem_config){}

    virtual bool Solve() override; 
private:


};

}//namespace
#endif