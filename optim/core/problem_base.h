//2020/12/10

#ifndef OPTIM_CORE_PROBLEM_BASE_H
#define OPTIM_CORE_PROBLEM_BASE_H

#include <assert.h>
#include <unordered_map>
#include <map>
#include <memory>
#include <functional>

#include "optim/core/eigen_types.h"
#include "optim/core/edge.h"
#include "optim/core/vertex.h"
#include "optim/core/linear_solver.h"

namespace Optim {

using namespace std;
// Q: can they be used in the outside???


/*
    Problem_Base actually holds the 
        1. data struct of the Guassain Markov graph
        2. the nonlinear strategy
        3. the linear solver
        4. give specific id of vertex, and edge
    The specific MLE problem is elaborated via the child class in the /problem directory 
*/

/*
    Note:
    Pronlem controls its vertice and edges ids
    ProvideVertexId()
*/


/*
    an example to override Sovle():
    1. SetOrdering()
    -- override it in specific problem to construct the pose at 
      the front, and the sparse landmark at the back
    2. MakeHessian()
    -- should be fixed, and its struct is directly from the SetOrdering()
    3. SetSolveLinearFunc()
    -- override it in specific probelm. e.g. the shur component trick
      can be used if we know the ordering of pose and landmark. And
      that ordering is override in the "SetOrdering"

    4. ProcessNonlinear()
    -- the three method can be override in the child class for its 
       special usage. e.g. the EM method can be expanded it the child class
*/

/*
    NOTE:
        the results are extracted form the corresponding vertex->Parameters()
*/

class ProblemBase{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::map<size_t, std::shared_ptr<Vertex>> 
                                                    HashVertex;
    typedef std::unordered_map<size_t, std::shared_ptr<Edge>> 
                                                    HashEdge;
    typedef std::unordered_multimap<size_t, std::shared_ptr<Edge>> 
                                                    HashVertexIdToEdge;

    enum class IterationStrategy{
        GAUSSIAN_NEWTON_METHOD,
        LEVENBERGER_MARQUARDT_METHOD,
        DOGLEG_METHOD,
        EXPECTATION_MAXIMIZATION
    };

    enum class LinearSolverMethod{
        CHOLESKY_METHOD,
        QR_METHOD,
        JACOBI_PCG_METHOD,
    }; 

    struct Problem_Config{
        // bool if_calc_pose_prior = false;
        size_t iteration_num = 10;
        IterationStrategy nonlinear_strategy = 
            IterationStrategy::LEVENBERGER_MARQUARDT_METHOD;
        LinearSolverMethod linear_solver_method = 
            LinearSolverMethod::CHOLESKY_METHOD;
    };

    ProblemBase(const Problem_Config& problem_config);

    ~ProblemBase();

    // --- 1. set/remove
    bool AddVertex(std::shared_ptr<Vertex> vertex);
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);
    bool RemoveEdge(std::shared_ptr<Edge> edge);

    // ---2. prior related process
    inline MatXX GetHessianPrior(){ return H_prior_;}
    inline VecX GetbPrior(){ return b_prior_;}
    inline VecX GetErrPrior(){ return err_prior_;}
    inline MatXX GetJtPrior(){ return Jt_prior_inv_;}
    
    inline void SetHessianPrior(const MatXX& H){H_prior_ = H;}
    inline void SetbPrior(const VecX& b){b_prior_ = b;}
    inline void SetErrPrior(const VecX& b){err_prior_ = b;}
    inline void SetJtPrior(const MatXX& J){Jt_prior_inv_ = J;}
    
    // --- 3. process
    virtual bool Solve() = 0;

    //---- 4. help to identify the edge and vertex
    inline size_t ProvideVertexId(){
         return accumulated_vertex_id_++;}

    inline size_t ProvideEdgeId(){
        return accumulated_edge_id_++;}

protected:    
    // CORE COMPONENT OF ‘Solve()’
    
    //the ordinary process is using the addeding-vertex ordering
    // override this method in the child class, to distinguish the
    // dense pose-vertex and sparse landmark-vertex
    virtual void SetOrdering();
    
    // if the ordering is fixed, this function should be fixed
    // in all types of problem
    void MakeHessian();
    
    // To set the "linear_solver_lambda_func_",
    // which will be used in nonlinear process
    //explicit schur method should be implement this in vo problem
    virtual void SetSolveLinearFunc(); 

    // usually fixed  in a problem, 
    void UpdateStates();

    std::vector<std::shared_ptr<Edge>> 
        GetConnectedEdges(std::shared_ptr<Vertex> vertex);

    // <Define nonlinear methods>
    void ProcessNonlinear();

    // --<nonlinear methods 1>
    virtual void ProcessGaussionNewton();

    // --<nonlinear methods 2>
    virtual void ProcessLevenbergMarquardt();

    // --<nonlinear methods 3>
    virtual void ProcessDogLeg();

    // prior manipulation 
    void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);
    void ExtendHessiansPriorSize(int dim);

    // --- 0. 
    const Problem_Config problem_config_;

    // --- 1. core matrix
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    // ---2. linear Solver
    std::shared_ptr<DenseLinearSolverBase> 
        liner_solver_ptr_ = nullptr;

    // in child class, more types of alternatives 
    // of "SetDefaultSolveLinearSystem" will be define, such that
    // the schur trick can be used in some sparse problem
    std::function<void(const MatXX& H, const VecX& b, VecX& x)> 
        ExecuteLinearSolverFunc_;

    // accumulate by the dim of vertex to set the "ordering_id"
    size_t ordering_generic_ = 0;

    /// all vertices
    HashVertex verticies_;

    // all edges
    HashEdge edges_;

    // 由vertex id查询edge
    HashVertexIdToEdge vertexToEdge_; // for "GetConnectedEdges()"

    // provide id for vertex and edges
    size_t accumulated_vertex_id_;
    size_t accumulated_edge_id_;

    // --- prior related Mat
    MatXX H_prior_;
    VecX b_prior_;
    VecX b_prior_backup_;
    VecX err_prior_backup_;

    MatXX Jt_prior_inv_;
    VecX err_prior_;

    // time
    double t_hessian_cost_ = 0.0;
};

}// namespace

#endif