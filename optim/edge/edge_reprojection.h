#ifndef OPTIM_EDGE_REPROJECTION_H
#define OPTIM_EDGE_REPROJECTION_H

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "optim/core/eigen_types.h"
#include "optim/core/edge.h"

#include "optim/vertex/vertex_point_xyz.h"
#include "optim/vertex/vertex_pose.h"

#include "optim/vertex/vertex_inverse_depth.h"

namespace Optim {

#define UNIT_SPHERE_ERROR

class EdgeMonoInvDepReprojection : public Edge{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    EdgeMonoInvDepReprojection(const Vec3 &pts_i, const Vec3 &pts_j, const size_t& id);

    // aforementioned methods are identical in every child class
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_;} 

    virtual void ComputeResidual() override;
    virtual void ComputeJacobians() override;
private:
    //measurements
    Vec3 pts_i_, pts_j_;
#ifdef UNIT_SPHERE_ERROR
    Mat23 tangent_base_;
#endif
};

class EdgeMonoReprojectionXYZ : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeMonoReprojectionXYZ(const Vec2 &pts_i, const size_t& id)
        : Edge(id, 2, 2, std::vector<std::string>{
            VertexPointXYZ::type_name_,
            VertexPose::type_name_}) {
        obs_ = pts_i;
    }

    // aforementioned methods are identical in every child class
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_;} 

    virtual void ComputeResidual() override;
    virtual void ComputeJacobians() override;

private:
    //measurements
    Vec2 obs_;
};

class EdgeMonoReprojectionPoseOnly : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeMonoReprojectionPoseOnly(const Vec3 &landmark_world, const Vec2 &pts_i, const size_t& id) :
        Edge(id, 2, 1, std::vector<std::string>{VertexPose::type_name_}),
        landmark_world_(landmark_world),
        obs_(pts_i){}

    //
    const static std::string type_name_;
    std::string TypeInfo() const { return type_name_;} 

    virtual void ComputeResidual() override;
    virtual void ComputeJacobians() override;

private:
    Vec3 landmark_world_;
    Vec2 obs_;
};

}


#endif
