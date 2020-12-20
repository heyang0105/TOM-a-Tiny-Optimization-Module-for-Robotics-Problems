#include"optim/vertex/vertex_pose.h"
#include"optim/sophus/so3.hpp"

namespace Optim{

//#define Use_Sophus

//---------------Implementation-----------------------
void VertexPose::Plus(const VecX &delta) {
    VecX &parameters = Parameters();
    parameters.head<3>() += delta.head<3>(); // translation
    Quatd q(parameters[3], parameters[4], parameters[5], parameters[6]);
#ifdef Use_Sophus
    //q = q * Sophus::SO3d::exp(Vec3(delta[3], delta[4], delta[5])).unit_quaternion();  // right multiplication with so3
    q =  Sophus::SO3d::exp(Vec3(delta[3], delta[4], delta[5])).unit_quaternion() * q;
#else
    Quatd dq = deltaQ(Vec3(delta[3], delta[4], delta[5]));
    //q = q * dq;
    q = dq * q;
#endif
    q.normalized();
    parameters[3] = q.w();
    parameters[4] = q.x();
    parameters[5] = q.y();
    parameters[6] = q.z();
    
}

}
