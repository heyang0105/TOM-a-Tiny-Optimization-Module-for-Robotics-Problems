#include"optim/problem/problem_dense.h"

namespace Optim{

bool ProblemDense::Solve(){
    //---1. uisng default
    SetSolveLinearFunc();
    //---2. uisng default
    SetOrdering();
    //---3.
    MakeHessian();
    //---4.
    ProcessNonlinear();

    return true;
}

}//namespace