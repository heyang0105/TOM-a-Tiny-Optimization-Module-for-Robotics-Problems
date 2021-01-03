
#ifndef OPTIM_CORE_LOSS_FUNCTION_H
#define OPTIM_CORE_LOSS_FUNCTION_H

#include "optim/core/eigen_types.h"

namespace Optim {

/**
 * compute the scaling factor for a error:
 * The error is e^T Omega e
 * The output rho is
 * rho[0]: The actual scaled error value
 * rho[1]: First derivative of the scaling function
 * rho[2]: Second derivative of the scaling function
 *
 * LossFunction是各核函数的基类，它可以派生出各种Loss
 */
class LossFunction {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual ~LossFunction() {}

//    virtual double Compute(double error) const = 0;
    virtual void Compute(double err2, Eigen::Vector3d& rho) const = 0;
};

/**
 * 平凡的Loss，不作任何处理
 * 使用nullptr作为loss function时效果相同
 *
 * TrivalLoss(e) = e^2
 */
class TrivalLoss : public LossFunction {
public:
    virtual void Compute(double err2, Eigen::Vector3d& rho) const override
    {
        // TODO:: whether multiply 1/2
        rho[0] = err2;
        rho[1] = 1;
        rho[2] = 0;
    }
};

/**
 * Huber loss
 *
 * Huber(e) = e^2                      if e <= delta
 * huber(e) = delta*(2*e - delta)      if e > delta
 */
class HuberLoss : public LossFunction {
public:
    explicit HuberLoss(double delta) : delta_(delta) {}

    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

private:
    double delta_;

};

/*
    * Cauchy loss
    *
    */
class CauchyLoss : public LossFunction
{
public:
    explicit CauchyLoss(double delta) : delta_(delta) {}

    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

private:
    double delta_;
};

class TukeyLoss : public LossFunction
{
public:
    explicit TukeyLoss(double delta) : delta_(delta) {}

    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

private:
    double delta_;
};


/** \brief The robust Dynamic Covariance Scaling loss function
 * Based on:
 * P. Agarwal, G. D. Tipaldi, L. Spinello, C. Stachniss and W. Burgard
 * "Robust map optimization using dynamic covariance scaling"
 * 2013 IEEE International Conference on Robotics and Automation, Karlsruhe, 2013
 * DOI: 10.1109/ICRA.2013.6630557
 *
 * \param Phi Tuning parameter of DCS
 *
 */

class DCSLoss : public LossFunction{
public:
    explicit DCSLoss(double phi) : phi_(phi) {};
    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;
private:
    const double phi_;
 };

/** \brief The robust closed form of Dynamic Covariance Estimation
 * Based on:
 * T. Pfeifer, S. Lange and P. Protzel
 * "Dynamic Covariance Estimation — A parameter free approach to robust Sensor Fusion"
 * 2017 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI), Daegu, 2017
 * DOI: 10.1109/MFI.2017.8170347
 *
 * \param Sigma Standard deviation without outliers
 *
 */

class cDCELoss : public LossFunction{
public:
    explicit cDCELoss(double Sigma) : Sigma_(Sigma) {};

    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

private:
    const double Sigma_;
};

}// namespace

#endif //
