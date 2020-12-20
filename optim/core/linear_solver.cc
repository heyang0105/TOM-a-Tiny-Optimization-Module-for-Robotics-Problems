// 2020/12/14
#include"optim/core/linear_solver.h"

namespace Optim{

namespace internal{
    void CheckMatrixDimension(const MatXX& H, const VecX& b){
        assert(H.rows() == H.cols() && 
            "DenseLinearSolver ERROR: H is not a square matrix");
    
        assert(H.rows() == b.rows() && 
            "DenseLinearSolver ERROR: H's dimension is not compatible with b matrix");
    }
}

void CholeskySolver:: Solve(const MatXX& H, const VecX& b, VecX& x){
    internal::CheckMatrixDimension(H, b);
    x = H.ldlt().solve(b);
}


void QRSolver:: Solve(const MatXX& H, const VecX& b, VecX& x){
    internal::CheckMatrixDimension(H, b);
    x = H.colPivHouseholderQr().solve(b);
}

void JacobiPCGSolver:: Solve(const MatXX& H, const VecX& b, VecX& x){
    
    internal::CheckMatrixDimension(H, b);
    // set iteration
    int rows = b.rows();
    int n = max_iter_ < 0 ? rows : max_iter_;
    //VecX x(VecX::Zero(rows));
    MatXX M_inv = H.diagonal().asDiagonal().inverse();
    VecX r0(b);  // initial r = b - A*0 = b
    VecX z0 = M_inv * r0;
    VecX p(z0);
    VecX w = H * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VecX r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n) {
        i++;
        VecX z1 = M_inv * r1;
        double r1z1 = r1.dot(z1);
        double belta = r1z1 / r0z0;
        z0 = z1;
        r0z0 = r1z1;
        r0 = r1;
        p = belta * p + z1;
        w = H * p;
        alpha = r1z1 / p.dot(w);
        x += alpha * p;
        r1 -= alpha * w;
    }
}

}// namespace