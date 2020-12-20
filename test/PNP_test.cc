#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <memory>
#include"optim/optim.h"

using namespace std;
using namespace cv;
using namespace Optim;

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char **argv) {
    // if (argc != 5) {
    //     cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
    //     return 1;
    // }
    //-- 读取图像
    Mat img_1 = imread("/home/yang/odometry_project/Optim_module/bin/1.png", CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread("/home/yang/odometry_project/Optim_module/bin/2.png", CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat d1 = imread("/home/yang/odometry_project/Optim_module/bin/1_depth.png", CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for (DMatch m:matches) {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)   // bad depth
            continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }

    cout << "3d-2d pairs: " << pts_3d.size() << endl;


    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    //----------------------------------------------------------------------------------------
  #if 0
    std::shared_ptr<Optim::LossFunction> lossfunction_ptr;
    lossfunction_ptr.reset(new  Optim::CauchyLoss(1.0));
    
    std::shared_ptr<Optim::Problem> problem_ptr;
    problem_ptr.reset(new Optim::Problem(Optim::Problem::ProblemType::GENERIC_PROBLEM));

    // ---1. vertex
    std::shared_ptr<Optim::VertexPose> pose_v_ptr(
        new Optim::VertexPose());

    Eigen::Vector3d tvec(0.0, 0.0, 0.0);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::VectorXd pose_param_vec(7);
    pose_param_vec << tvec(0, 0), tvec(1, 0), tvec(2, 0), 
        quat.w(), quat.x(), quat.y(), quat.z();

    pose_v_ptr->SetParameters(pose_param_vec);
    problem_ptr->AddVertex(pose_v_ptr);

    //--- 2. edge
    std::vector<std::shared_ptr<Optim::VertexPointXYZ>> pts_ptr_vecs;

    for (DMatch m:matches) {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)   // bad depth
            continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Eigen::Vector3d pts_w(p1.x * dd, p1.y * dd, dd); // w is image_1
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Eigen::Vector2d observaion_pts(p2.x, p2.y);

        //---point vertex
        std::shared_ptr<Optim::VertexPointXYZ> point_v_ptr(new Optim::VertexPointXYZ());
        Eigen::VectorXd point_param_vec(3);
        point_param_vec << pts_w(0, 0), pts_w(1, 0), pts_w(2, 0);
        point_v_ptr->SetParameters(point_param_vec);
        point_v_ptr->SetFixed();
        problem_ptr->AddVertex(point_v_ptr);
        pts_ptr_vecs.push_back(point_v_ptr);
        
        //---edge
        std::shared_ptr<Optim::EdgeMonoReprojectionXYZ> edge_ptr(
            new Optim::EdgeMonoReprojectionXYZ(observaion_pts));

        std::vector<std::shared_ptr<Optim::Vertex>> edge_vs;
        edge_vs.push_back(point_v_ptr);
        edge_vs.push_back(pose_v_ptr);

        edge_ptr->SetVertices(edge_vs);
        edge_ptr->SetLossFunction(lossfunction_ptr.get());

        problem_ptr->AddEdge(edge_ptr);
    }
    
    problem_ptr->Solve(10);

    Eigen::VectorXd res_pose = pose_v_ptr->Parameters();

    Eigen::Vector3d res_tvec(res_pose[0], res_pose[1], res_pose[2]);
    Eigen::Quaterniond res_quat(res_pose[3], res_pose[4], res_pose[5], res_pose[6]);
#endif
//----------------------------------------------------------------------------------

#define TEST_POSE_ONLY
#if 0
    std::shared_ptr<Optim::LossFunction> lossfunction_ptr;
    lossfunction_ptr.reset(new  Optim::CauchyLoss(1.0));
    
    std::shared_ptr<Optim::Problem> problem_ptr;
    problem_ptr.reset(new Optim::Problem(Optim::Problem::ProblemType::GENERIC_PROBLEM));

    // ---1. vertex
    std::shared_ptr<Optim::VertexPose> pose_v_ptr(
        new Optim::VertexPose());

    Eigen::Vector3d tvec(0.0, 0.0, 0.0);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::VectorXd pose_param_vec(7);
    pose_param_vec << tvec(0, 0), tvec(1, 0), tvec(2, 0), 
        quat.w(), quat.x(), quat.y(), quat.z();

    pose_v_ptr->SetParameters(pose_param_vec);
    problem_ptr->AddVertex(pose_v_ptr);

    //--- 2. edge
    std::vector<std::shared_ptr<Optim::VertexPointXYZ>> pts_ptr_vecs;

    for (DMatch m:matches) {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)   // bad depth
            continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Eigen::Vector3d pts_w(p1.x * dd, p1.y * dd, dd); // w is image_1
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Eigen::Vector2d observaion_pts(p2.x, p2.y);

        //---edge
        std::shared_ptr<Optim::EdgeMonoReprojectionPoseOnly> edge_ptr(
            new Optim::EdgeMonoReprojectionPoseOnly(pts_w, observaion_pts));

        std::vector<std::shared_ptr<Optim::Vertex>> edge_vs;
        edge_vs.push_back(pose_v_ptr);

        edge_ptr->SetVertices(edge_vs);
        edge_ptr->SetLossFunction(lossfunction_ptr.get());

        problem_ptr->AddEdge(edge_ptr);
    }
    
    problem_ptr->Solve(10);

    Eigen::VectorXd res_pose = pose_v_ptr->Parameters();

    Eigen::Vector3d res_tvec(res_pose[0], res_pose[1], res_pose[2]);
    Eigen::Quaterniond res_quat(res_pose[3], res_pose[4], res_pose[5], res_pose[6]);
#endif

#if 1
    std::shared_ptr<Optim::LossFunction> lossfunction_ptr;
    lossfunction_ptr.reset(new  Optim::CauchyLoss(1.0));

    Optim::ProblemBase::Problem_Config config;
    config.iteration_num = 20;
    config.nonlinear_strategy = 
        Optim::ProblemBase::IterationStrategy::LEVENBERGER_MARQUARDT_METHOD;
    config.linear_solver_method = 
        Optim::ProblemBase::LinearSolverMethod::CHOLESKY_METHOD;

    std::shared_ptr<Optim::ProblemBase> problem_ptr;
    problem_ptr.reset(new Optim::ProblemDense(config));

    // ---1. vertex
    std::shared_ptr<Optim::VertexPose> pose_v_ptr(
        new Optim::VertexPose(problem_ptr->ProvideVertexId()));

    Eigen::Vector3d tvec(0.0, 0.0, 0.0);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::VectorXd pose_param_vec(7);
    pose_param_vec << tvec(0, 0), tvec(1, 0), tvec(2, 0), 
        quat.w(), quat.x(), quat.y(), quat.z();

    pose_v_ptr->SetParameters(pose_param_vec);
    problem_ptr->AddVertex(pose_v_ptr);

    //--- 2. edge
    std::vector<std::shared_ptr<Optim::VertexPointXYZ>> pts_ptr_vecs;

    for (DMatch m:matches) {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)   // bad depth
            continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Eigen::Vector3d pts_w(p1.x * dd, p1.y * dd, dd); // w is image_1
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Eigen::Vector2d observaion_pts(p2.x, p2.y);

        //---edge
        std::shared_ptr<Optim::EdgeMonoReprojectionPoseOnly> edge_ptr(
            new Optim::EdgeMonoReprojectionPoseOnly(
                pts_w, observaion_pts, 
                problem_ptr->ProvideEdgeId()));

        std::vector<std::shared_ptr<Optim::Vertex>> edge_vs;
        edge_vs.push_back(pose_v_ptr);

        edge_ptr->SetVertices(edge_vs);
        edge_ptr->SetLossFunction(lossfunction_ptr.get());

        problem_ptr->AddEdge(edge_ptr);
    }

    problem_ptr->Solve();

    Eigen::VectorXd res_pose = pose_v_ptr->Parameters();

    Eigen::Vector3d res_tvec(res_pose[0], res_pose[1], res_pose[2]);
    Eigen::Quaterniond res_quat(res_pose[3], res_pose[4], res_pose[5], res_pose[6]);
#endif

    cout<< "R" << endl << res_quat.toRotationMatrix().matrix() << endl;
    cout<< "t" << endl << res_tvec.transpose().matrix() << endl;

    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;

}


void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> match;
  // BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}