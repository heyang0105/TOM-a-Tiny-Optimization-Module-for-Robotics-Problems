#include "optim/sophus/se3.hpp"
#include "optim/vertex/vertex_pose.h"
#include "optim/edge/edge_reprojection.h"


#include <iostream>

namespace Optim {

EdgeMonoInvDepReprojection::EdgeMonoInvDepReprojection(
    const Vec3 &pts_i, const Vec3 &pts_j, const size_t& id)
    : Edge(id, 2, 3, std::vector<std::string>{
        VertexInverseDepth::type_name_, 
        VertexPose::type_name_,
        VertexPose::type_name_}), 
    pts_i_(pts_i),
    pts_j_ (pts_j){

#ifdef UNIT_SPHERE_ERROR
    Vec3 b1, b2;
    Vec3 a = pts_j.normalized();
    Vec3 tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base_.block<1, 3>(0, 0) = b1.transpose();
    tangent_base_.block<1, 3>(1, 0) = b2.transpose();
#endif
}

void EdgeMonoInvDepReprojection::ComputeResidual() {
    // --- 1. get params
    double inv_dep_i = verticies_[0]->Parameters()[0];
   
    VecX param_i = verticies_[1]->Parameters();
    Sophus::SE3d pose_i(
        Quatd(param_i[3], param_i[4], param_i[5], param_i[6]),
        param_i.head<3>());

    VecX param_j = verticies_[2]->Parameters();
    Sophus::SE3d pose_j(
        Quatd(param_j[3], param_j[4], param_j[5], param_j[6]),
        param_j.head<3>());

    //--- 2. pts_c_j' = T_w_cj.inv() * T_w_ci * pts_c_i
    Vec3 pts_c_i = pts_i_ / inv_dep_i;
    Vec3 pts_c_j = pose_j.inverse() * pose_i * pts_c_i;

    //---3. 
#ifdef UNIT_SPHERE_ERROR 
    residual_ =  tangent_base_ * (pts_c_j.normalized() - pts_j_.normalized());
#else
    double dep_j = pts_c_j.z();
    residual = (pts_c_j / dep_j).head<2>() - pts_j_.head<2>();
#endif
}

void EdgeMonoInvDepReprojection::ComputeJacobians() {
    // --- 1. get params
    double inv_dep_i = verticies_[0]->Parameters()[0];
   
    VecX param_i = verticies_[1]->Parameters();
    Sophus::SE3d pose_i(
        Quatd(param_i[3], param_i[4], param_i[5], param_i[6]),
        param_i.head<3>());

    VecX param_j = verticies_[2]->Parameters();
    Sophus::SE3d pose_j(
        Quatd(param_j[3], param_j[4], param_j[5], param_j[6]),
        param_j.head<3>());

    //--- 2. pts_c_j' = T_w_cj.inv() * T_w_ci * pts_c_i
    Vec3 pts_c_i = pts_i_ / inv_dep_i;
    Vec3 pts_c_j = pose_j.inverse() * pose_i * pts_c_i;

    //--- 3. reduce
    Mat23 reduce(2, 3);
#ifdef UNIT_SPHERE_ERROR
    double norm = pts_c_j.norm();
    Mat33 norm_jaco;
    double x1, x2, x3;
    x1 = pts_c_j(0); x2 = pts_c_j(1); x3 = pts_c_j(2);
    double norm_inv_3 = 1 / pow(norm, 3);
    norm_jaco << 1.0 / norm - x1 * x1 * norm_inv_3, - x1 * x2* norm_inv_3,            - x1 * x3 * norm_inv_3,
                     - x1 * x2 * norm_inv_3,      1.0 / norm - x2 * x2 * norm_inv_3, - x2 * x3 * norm_inv_3,
                     - x1 * x3* norm_inv_3,       - x2 * x3 * norm_inv_3,            1.0 / norm - x3 * x3 * norm_inv_3;
    reduce = tangent_base_ * norm_jaco;
#else
    double dep_inv = 1. / pts_c_j[2];
    double dep_2_inv = dep_inv * dep_inv; 

    reduce << dep_inv, 0,  - pts_c_j[0] * dep_2_inv,
              0, dep_inv, - pts_c_j[1] * dep_2_inv; 
#endif
    /*
    R_cj_w * [ I  -skew(pts_c_i)]
    */

    Mat36 jacob_i;
    jacob_i.leftCols<3>() = pose_j.rotationMatrix().transpose(); /*Pi*/
    jacob_i.rightCols<3>() = /*Qi*/
        pose_j.rotationMatrix().transpose() * (-1.0) * skewSymmetric(pts_c_i);

    Mat36 jacob_j;
    jacob_i.leftCols<3>() = - pose_j.rotationMatrix().transpose();
    jacob_j.rightCols<3>() = skewSymmetric(pts_c_j);

    Vec3 jacob_dep = 
        pose_j.rotationMatrix().transpose() * 
        pose_i.rotationMatrix() * pts_i_ * 
        (-1.0) / (inv_dep_i * inv_dep_i);

    
    jacobians_[0] = reduce * jacob_dep;
    jacobians_[1] = reduce * jacob_i;
    jacobians_[2] = reduce * jacob_j;
}

void EdgeMonoReprojectionXYZ::ComputeResidual() {
    Vec3 pts_w = verticies_[0]->Parameters();
    VecX param_i = verticies_[1]->Parameters();
    Quatd Qi( param_i[3], param_i[4], param_i[5], param_i[6]);
    Vec3 Pi = param_i.head<3>();

    Vec3 pts_c = Qi * pts_w + Pi;
    //
    residual_ = (pts_c / pts_c[2]).head<2>() - obs_;
}


void EdgeMonoReprojectionXYZ::ComputeJacobians() {

    //
    Vec3 pts_w = verticies_[0]->Parameters();
    VecX param_i = verticies_[1]->Parameters();
    Quatd Qi( param_i[3], param_i[4], param_i[5], param_i[6]);
    Vec3 Pi = param_i.head<3>();

    Vec3 pts_c = Qi * pts_w + Pi;
    //
    double dep_inv = 1. / pts_c[2];
    double dep_2_inv = dep_inv * dep_inv; 

    Mat23 reduce;
    reduce << dep_inv, 0,  - pts_c[0] * dep_2_inv,
              0, dep_inv, - pts_c[1] * dep_2_inv; 

    Mat36 jaco_i;
    jaco_i.block(0, 0, 3, 3) = Mat33::Identity();
    jaco_i.block(0, 3, 3, 3) = - skewSymmetric(pts_c);

    Mat23 jacob_point = reduce * Qi.toRotationMatrix();
    Mat26 jacob_pose = reduce * jaco_i;

    jacobians_[0] = jacob_point;
    jacobians_[1] = jacob_pose;
}

//----
void EdgeMonoReprojectionPoseOnly::ComputeResidual() {
    //
    VecX& pose_params = verticies_[0]->Parameters();
    Sophus::SE3d pose(
        Quatd(pose_params[3], pose_params[4], pose_params[5], pose_params[6]),
        pose_params.head<3>());

    Vec3 pc = pose * landmark_world_;
    //
    pc = pc / pc[2];
    residual_ =  pc.head<2>() - obs_;

}

void EdgeMonoReprojectionPoseOnly::ComputeJacobians() {
    //
    VecX& pose_params = verticies_[0]->Parameters();
    Sophus::SE3d pose(
        Quatd(pose_params[3], pose_params[4], pose_params[5], pose_params[6]),
        pose_params.head<3>());

    Vec3 pts_c = pose * landmark_world_;
    //

    double dep_inv = 1. / pts_c[2];
    double dep_2_inv = dep_inv * dep_inv; 

    Mat23 reduce;
    reduce << dep_inv, 0,  - pts_c[0] * dep_2_inv,
              0, dep_inv, - pts_c[1] * dep_2_inv; 

    Mat36 jaco_i;
    jaco_i.block(0, 0, 3, 3) = Mat33::Identity();
    jaco_i.block(0, 3, 3, 3) = - skewSymmetric(pts_c);
    Mat26 jacob_pose = reduce * jaco_i;

    jacobians_[0] = jacob_pose;
}

}// namespace

