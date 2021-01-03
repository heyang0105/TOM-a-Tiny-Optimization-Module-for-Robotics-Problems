#include"optim/edge/edge_direct_method.h"

namespace Optim{

// init the static camera intrinsics, which must be inited outside the class
    
uchar* EdgeMonoDirectMethod::cur_img_data_ = nullptr;
size_t EdgeMonoDirectMethod::width_ = 0;
size_t EdgeMonoDirectMethod::height_ = 0;

double EdgeMonoDirectMethod::fx_ = 0.0;
double EdgeMonoDirectMethod::fy_= 0.0;
double EdgeMonoDirectMethod::cx_= 0.0;
double EdgeMonoDirectMethod::cy_= 0.0;
Mat33 EdgeMonoDirectMethod::K_ = Mat33::Zero();

// void EdgeMonoDirectMethod::SetHalfPatchSize(size_t half_patch_size){
//     half_patch_size_ = half_patch_size;
// }

bool EdgeMonoDirectMethod::SetImageDate( uchar* cur_img_data, size_t width, size_t height){
	uchar* cur_img_data_ = cur_img_data;
    size_t width_= width;
    size_t height_ = height;
}

void EdgeMonoDirectMethod::SetCamIntrinsics(double fx, double fy,
                     double cx, double cy){
    // they are used for jacobian
    fx_ = fx; 
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;

    Mat33 K ;
	K<< fx, 0, cx,
         0, fy, cy,
         0, 0,  1.;
    EdgeMonoDirectMethod::K_ = K;
}

inline double EdgeMonoDirectMethod::GetPixelValue(uchar* img_data,
     const int& rows, const int& cols,
     const double& in_x, const double& in_y){
	// boundary check
    double x, y;

    if (in_x < 0.0){
        x = 0.0;
    }
    else if (in_x >= cols){
        x = cols - 1.0;
    }
    else{
        x = in_x;
    }

    if (in_y < 0.0){
        y = 0.0;
    }
    else if (in_y >= rows){
        y = rows - 1.0;
    }
    else{
        y = in_y;
    }
    
    if (y >= rows) y = rows - 1;
    uchar *data = &img_data[int(y) * cols + int(x)];
    double xx = x - floor(x);
    double yy = y - floor(y);
    
    return double(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[cols] +
        xx * yy * data[cols + 1]);
}

inline Vec2 EdgeMonoDirectMethod::CamProject(const Vec3& pts){
	Vec3 normal_pts(pts(0, 0)/pts(2, 0), pts(1, 0)/pts(2, 0), 1.0);
    Vec3 img_pts = K_ * normal_pts;
	return Vec2(img_pts(0, 0), img_pts(1, 0) );
}

void EdgeMonoDirectMethod::ComputeResidual(){
	
    //---1. param T_cur_ref
    VecX& pose_params = verticies_[0]->Parameters();
    Sophus::SE3d T_cur_ref(
        Quatd(pose_params[3], pose_params[4], pose_params[5], pose_params[6]),
        pose_params.head<3>());

	//---2. reprojection in cur
    Vec3 cur_pts = T_cur_ref * landmark_ref_;
    Vec2 cur_uv = CamProject( cur_pts);

	//---3. filter points on the boarder
    Vec1 diff;

    if( cur_uv(0, 0) - 4 < 0 || cur_uv(0, 0) + 4 > width_ ||
        cur_uv(1, 0) -4 < 0 || cur_uv(1, 0) + 4 > height_){
        diff<< 0.0;
    }
    else{
        diff << ref_grayscale_ - 
              GetPixelValue(cur_img_data_,
               height_, width_, cur_uv(0, 0), cur_uv(1, 0));
    }

    residual_ = diff;

}

void EdgeMonoDirectMethod::ComputeJacobians(){
        //---1. param T_cur_ref
    VecX& pose_params = verticies_[0]->Parameters();
    Sophus::SE3d T_cur_ref(
        Quatd(pose_params[3], pose_params[4], pose_params[5], pose_params[6]),
        pose_params.head<3>());

	//---2. reprojection in cur
    Vec3 cur_pts = T_cur_ref * landmark_ref_;
    Vec2 cur_uv = CamProject( cur_pts);

    // ---3. filter
    if( cur_uv(0, 0) - 4 < 0 || cur_uv(0, 0) + 4 > width_ ||
        cur_uv(1, 0) -4 < 0 || cur_uv(1, 0) + 4 > height_){
        
        jacobians_[0] = Eigen::Matrix<double, 6 , 1>::Zero();
        return;
    }

    //---3. grad(1*2) I/uv * (2*3) uv/pts * (3*6) pts/transform
    //  ---3.1 grad
    Vec2 gradient_uv;
    gradient_uv<< GetPixelValue(cur_img_data_,
               height_, width_, cur_uv(0, 0) + 1, cur_uv(1, 0) ) - 
               GetPixelValue(cur_img_data_,
               height_, width_, cur_uv(0, 0) - 1, cur_uv(1, 0) ),
               GetPixelValue(cur_img_data_,
               height_, width_, cur_uv(0, 0), cur_uv(1, 0) + 1 ) - 
               GetPixelValue(cur_img_data_,
               height_, width_, cur_uv(0, 0), cur_uv(1, 0) - 1 );

    //  ---3.2 uv/pts
    double dep_inv = 1. / cur_pts[2];
    double dep_2_inv = dep_inv * dep_inv; 
    
    Mat23 reduce;
    reduce << fx_ * dep_inv, 0, - fx_ * cur_pts[0] * dep_2_inv,
              0, fy_ * dep_inv, - fy_ * cur_pts[1] * dep_2_inv; 

    //  ---3.3 pts/ manifold
    Mat36 jaco_i;
    jaco_i.block(0, 0, 3, 3) = Mat33::Identity();
    jaco_i.block(0, 3, 3, 3) = - skewSymmetric(cur_pts);

    // --- 4.res
    Eigen::Matrix<double, 1, 6> jacob_pose = 
        gradient_uv.transpose()  * reduce * jaco_i;

    jacobians_[0] = jacob_pose;
}

}//namespace
