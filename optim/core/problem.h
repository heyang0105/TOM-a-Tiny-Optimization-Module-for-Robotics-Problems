#ifndef OPTIM_CORE_PROBLEM_H
#define OPTIM_CORE_PROBLEM_H

#include <unordered_map>
#include <map>
#include <memory>

#include "optim/core/eigen_types.h"
#include "optim/core/edge.h"
#include "optim/core/vertex.h"


namespace Optim {

typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

class Problem {
public:

    enum class ProblemType {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Problem(ProblemType problemType);

    ~Problem();

    // --- 1. set/remove
    bool AddVertex(std::shared_ptr<Vertex> vertex);
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);
    bool RemoveEdge(std::shared_ptr<Edge> edge);

    inline void SetHessianPrior(const MatXX& H){H_prior_ = H;}
    inline void SetbPrior(const VecX& b){b_prior_ = b;}
    inline void SetErrPrior(const VecX& b){err_prior_ = b;}
    inline void SetJtPrior(const MatXX& J){Jt_prior_inv_ = J;}

    // --- 2. get
    MatXX GetHessianPrior(){ return H_prior_;}
    VecX GetbPrior(){ return b_prior_;}
    VecX GetErrPrior(){ return err_prior_;}
    MatXX GetJtPrior(){ return Jt_prior_inv_;}

    // --- 3. process
    bool Solve(int iterations = 10);

    bool Marginalize(const std::vector<std::shared_ptr<Vertex> > frameVertex,int pose_dim);
    // TODO: not impl
    bool Marginalize(std::shared_ptr<Vertex> frameVertex,
                     const std::vector<std::shared_ptr<Vertex>> &landmarkVerticies);
    bool Marginalize(const std::shared_ptr<Vertex> frameVertex);

    // TODO: not impl
    void GetOutlierEdges(std::vector<std::shared_ptr<Edge>> &outlier_edges);

    void ExtendHessiansPriorSize(int dim);

    // TODO: not impl
    void TestComputePrior();

private:

    /// 设置各顶点的ordering_index
    void SetOrdering(); // --base

    /// set ordering for new vertex in slam problem
    void AddOrderingSLAM(std::shared_ptr<Vertex> v);// used in "SetOrdering()"
    // ---sparse vo

    /// 构造大H矩阵
    void MakeHessian(); //--base

    /// 解线性方程
    void SolveLinearSystem();

    /// 更新状态变量
    void UpdateStates(); //--base

    void RollbackStates(); // 有时候 update 后残差会变大，需要退回去，重来

    /// 判断一个顶点是否为Pose顶点
    bool IsPoseVertex(std::shared_ptr<Vertex> v);// --spec

    /// 判断一个顶点是否为landmark顶点
    bool IsLandmarkVertex(std::shared_ptr<Vertex> v); //--spec

    /// 在新增顶点后，需要调整几个hessian的大小
    void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);//used in "AddVertex()"
    // for prior mat, --spec

    /// 检查ordering是否正确
    bool CheckOrdering();

    /// 获取某个顶点连接到的边
    std::vector<std::shared_ptr<Edge>> GetConnectedEdges(std::shared_ptr<Vertex> vertex);
    // --base

    /// Levenberg
    /// 计算LM算法的初始Lambda
    void ComputeLambdaInitLM();

    /// Hessian 对角线加上或者减去  Lambda
    void AddLambdatoHessianLM();

    void RemoveLambdaHessianLM();

    /// LM 算法中用于判断 Lambda 在上次迭代中是否可以，以及Lambda怎么缩放
    bool IsGoodStepInLM();

    /// PCG 迭代线性求解器
    VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);

    double currentLambda_;
    double currentChi_;
    double stopThresholdLM_;    // LM 迭代退出阈值条件
    double ni_;                 //控制 Lambda 缩放大小

    ProblemType problemType_;

    /// 整个信息矩阵
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    /// 先验部分信息
    MatXX H_prior_;
    VecX b_prior_;
    VecX b_prior_backup_;
    VecX err_prior_backup_;

    MatXX Jt_prior_inv_;
    VecX err_prior_;

    /// SBA的Pose部分 --> 特有的移到专门的child class
    MatXX H_pp_schur_;
    VecX b_pp_schur_;
    // Heesian 的 Landmark 和 pose 部分
    MatXX H_pp_;
    VecX b_pp_;
    MatXX H_ll_;
    VecX b_ll_;

    /// all vertices
    HashVertex verticies_;

    /// all edges
    HashEdge edges_;

    /// 由vertex id查询edge
    HashVertexIdToEdge vertexToEdge_;

    /// Ordering related
    size_t ordering_poses_ = 0;
    size_t ordering_landmarks_ = 0;
    size_t ordering_generic_ = 0;

    //[ SET IN SPEC PROBLEM]
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;        // 以ordering排序的pose顶点
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices_;    // 以ordering排序的landmark顶点

    // verticies need to marg. <Ordering_id_, Vertex>
    HashVertex verticies_marg_; // not used

    bool bDebug = false;
    double t_hessian_cost_ = 0.0;
    double t_PCGsovle_cost_ = 0.0;
};

} // namespace


#endif
