#include "optim/core/loss_function.h"

namespace Optim {

/** @brief implements the a new DCS like loss function which is closer related to the mathematical meaning of a gauss distribution
 *
 * @param s   squared non-weighted error
 * @param rho [p(s) p'(s) p''(s)]
 */

void HuberLoss::Compute(double e, Eigen::Vector3d& rho) const {
    double dsqr = delta_ * delta_;
    if (e <= dsqr) { // inlier
        rho[0] = e;
        rho[1] = 1.;
        rho[2] = 0.;
    } else { // outlier
        double sqrte = sqrt(e); // absolut value of the error
        rho[0] = 2*sqrte*delta_ - dsqr; // rho(e)   = 2 * delta * e^(1/2) - delta^2
        rho[1] = delta_ / sqrte;        // rho'(e)  = delta / sqrt(e)
        rho[2] = - 0.5 * rho[1] / e;    // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
    }
}

void CauchyLoss::Compute(double err2, Eigen::Vector3d& rho) const {
    double dsqr = delta_ * delta_;       // c^2
    double dsqrReci = 1. / dsqr;         // 1/c^2
    double aux = dsqrReci * err2 + 1.0;  // 1 + e^2/c^2
    rho[0] = dsqr * log(aux);            // c^2 * log( 1 + e^2/c^2 )
    rho[1] = 1. / aux;                   // rho'
    rho[2] = -dsqrReci * std::pow(rho[1], 2); // rho''
}

void TukeyLoss::Compute(double e2, Eigen::Vector3d& rho) const{
    const double e = sqrt(e2);
    const double delta2 = delta_ * delta_;
    if (e <= delta_) {
        const double aux = e2 / delta2;
        rho[0] = delta2 * (1. - std::pow((1. - aux), 3)) / 3.;
        rho[1] = std::pow((1. - aux), 2);
        rho[2] = -2. * (1. - aux) / delta2;
    } else {
        rho[0] = delta2 / 3.;
        rho[1] = 0;
        rho[2] = 0;
    }
}

void DCSLoss::Compute(double e2, Eigen::Vector3d& rho) const{
    if(e2 <= phi_){
      rho[0] = e2;
      rho[1] = 1;
      rho[2] = 0;
    }
    else{
      rho[0] = phi_ * (3 * e2 - phi_) / (e2 + phi_);
      rho[1] = (3*phi_)/(phi_ + e2) + (phi_*(phi_ - 3*e2))/pow((phi_ + e2), 2);
      rho[2] = - (6*phi_)/pow((phi_ + e2), 2) - (2*phi_*(phi_ - 3*e2))/pow((phi_ + e2), 3);
    }
}

void cDCELoss::Compute(double e2, Eigen::Vector3d& rho) const{
    double e = sqrt(e2);

    if(e <= Sigma_)
    {
        rho[0] = e / (Sigma_ * Sigma_);
        rho[1] = 1.0 / (Sigma_ * Sigma_);
        rho[2] = 0.0;
    }
    else
    {
        rho[0] = 1.0 + log(e / Sigma_) * 2;
        rho[1] = 1.0 / (e2);
        rho[2] = -1.0 / (e2 * e2);
    }
}

}// namespace

