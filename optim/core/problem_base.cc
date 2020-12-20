#include"optim/core/problem_base.h"
#include"optim/util/tic_toc.h"

namespace Optim{

ProblemBase::ProblemBase(const Problem_Config& problem_config) :
    problem_config_(problem_config){
    //verticies_marg_.clear(); // TODO: is it necessary in base

    if(LinearSolverMethod::CHOLESKY_METHOD == 
        problem_config_.linear_solver_method){
        liner_solver_ptr_.reset(new CholeskySolver());
    }
    else if(LinearSolverMethod::QR_METHOD == 
        problem_config_.linear_solver_method){
        liner_solver_ptr_.reset(new QRSolver() );
    }
    else if(LinearSolverMethod::JACOBI_PCG_METHOD == 
        problem_config_.linear_solver_method){
        liner_solver_ptr_.reset(new  JacobiPCGSolver());
    }
    else{
        assert(false && "Unspecified linear solver!!!");
    }

    accumulated_vertex_id_ = 0;
    accumulated_edge_id_ = 0;
}

ProblemBase::~ProblemBase() {
    std::cout << "Problem Is Deleted"<<std::endl;
}

bool ProblemBase::AddVertex(std::shared_ptr<Vertex> vertex) {
    if (verticies_.find(vertex->Id()) != verticies_.end()) {
        std::cout << "Vertex " << vertex->Id() << " has been added before";
        return false;
    } 
    else {
        verticies_.insert(make_pair(vertex->Id(), vertex));
    }
    return true;
}

bool ProblemBase::RemoveVertex(std::shared_ptr<Vertex> vertex) {
    //check if the vertex is in map_verticies_
    if (verticies_.find(vertex->Id()) == verticies_.end()) {
        // LOG(WARNING) << "The vertex " << vertex->Id() << " is not in the problem!" << endl;
        return false;
    }

    // 这里要 remove 该顶点对应的 edge.
    vector<shared_ptr<Edge>> remove_edges = GetConnectedEdges(vertex);
    for (size_t i = 0; i < remove_edges.size(); i++) {
        RemoveEdge(remove_edges[i]);
    }

    vertex->SetOrderingId(-1);      // used to debug
    verticies_.erase(vertex->Id());
    vertexToEdge_.erase(vertex->Id());

    return true;
}

bool ProblemBase::AddEdge(shared_ptr<Edge> edge) {
    if (edges_.find(edge->Id()) == edges_.end()) {
        edges_.insert(pair<ulong, std::shared_ptr<Edge>>(edge->Id(), edge));
    } else {
        // LOG(WARNING) << "Edge " << edge->Id() << " has been added before!";
        return false;
    }

    for (auto &vertex: edge->Verticies()) {
        vertexToEdge_.insert(pair<ulong, shared_ptr<Edge>>(vertex->Id(), edge));
    }
    return true;
}

bool ProblemBase::RemoveEdge(std::shared_ptr<Edge> edge) {
    //check if the edge is in map_edges_
    if (edges_.find(edge->Id()) == edges_.end()) {
        // LOG(WARNING) << "The edge " << edge->Id() << " is not in the problem!" << endl;
        return false;
    }

    edges_.erase(edge->Id());
    return true;
}


vector<shared_ptr<Edge>> ProblemBase::GetConnectedEdges(std::shared_ptr<Vertex> vertex) {
    vector<shared_ptr<Edge>> edges;
    auto range = vertexToEdge_.equal_range(vertex->Id());
    for (auto iter = range.first; iter != range.second; ++iter) {

        // 并且这个edge还需要存在，而不是已经被remove了
        if (edges_.find(iter->second->Id()) == edges_.end())
            continue;

        edges.emplace_back(iter->second);
    }
    return edges;
}

void ProblemBase::SetOrdering() {

    ordering_generic_ = 0;

    // Note:: verticies_ is map 
    for (auto vertex: verticies_) {
        ordering_generic_ += vertex.second->LocalDimension(); 

    }
//    CHECK_EQ(CheckOrdering(), true);
}

void ProblemBase::MakeHessian() {
    TicToc t_h;

    size_t size = ordering_generic_;
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size));

    for (auto &edge: edges_) {

        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        // TODO:: robust cost
        auto jacobians = edge.second->Jacobians();
        auto verticies = edge.second->Verticies();
        assert(jacobians.size() == verticies.size());
        for (size_t i = 0; i < verticies.size(); ++i) {
            auto v_i = verticies[i];
            if (v_i->IsFixed()) continue;    // Hessian 里不需要添加它的信息，也就是它的雅克比为 0

            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();

            // 鲁棒核函数会修改残差和信息矩阵，如果没有设置 robust cost function，就会返回原来的
            double drho;
            MatXX robustInfo(edge.second->Information().rows(),edge.second->Information().cols());
            edge.second->RobustInfo(drho,robustInfo);

            MatXX JtW = jacobian_i.transpose() * robustInfo;
            for (size_t j = i; j < verticies.size(); ++j) {
                auto v_j = verticies[j];

                if (v_j->IsFixed()) continue;

                auto jacobian_j = jacobians[j];
                ulong index_j = v_j->OrderingId();
                ulong dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);
                MatXX hessian = JtW * jacobian_j;

                // 所有的信息矩阵叠加起来
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                if (j != i) {
                    // 对称的下三角
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();

                }
            }
            b.segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose()* edge.second->Information() * edge.second->Residual();
        }

    }
    Hessian_ = H;
    b_ = b;
    t_hessian_cost_ += t_h.toc();

    delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;

}

void ProblemBase::SetSolveLinearFunc(){
    ExecuteLinearSolverFunc_ = 
    [&](const MatXX& H, const VecX& b, VecX& x){
        if(liner_solver_ptr_){
            if( H.rows()  == 0 || 
                b.rows() == 0 ||
                x.rows() == 0){
                assert(false && "MakeHessian() is not implemented!!!");
            }
            else{
                liner_solver_ptr_->Solve(H, b, x);
            }
        }
        else{
            assert(false&&"linear solver is not defined");
        }
    };
}

void ProblemBase::UpdateStates(){
    // update vertex
    for (auto vertex: verticies_) {
        vertex.second->BackUpParameters();    // 保存上次的估计值

        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);
        vertex.second->Plus(delta);
    }

}


void ProblemBase::ProcessNonlinear(){

    if(IterationStrategy::GAUSSIAN_NEWTON_METHOD ==
        problem_config_.nonlinear_strategy){
            ProcessGaussionNewton();
    }
    else if(IterationStrategy::LEVENBERGER_MARQUARDT_METHOD ==
        problem_config_.nonlinear_strategy){
            ProcessLevenbergMarquardt();
    }
    else if(IterationStrategy::DOGLEG_METHOD ==
        problem_config_.nonlinear_strategy){
            ProcessDogLeg();
    }
    else{
        assert(false&&"nonlinear strategy is not defined");
    }
}

// --<nonlinear methods 1>
void ProblemBase::ProcessGaussionNewton(){

}

// --<nonlinear methods 2>
void ProblemBase::ProcessLevenbergMarquardt(){
    
    TicToc t_solve;

    // params to control the trust region
    double currentLambda_;
    double currentChi_;
    double stopThresholdLM_;    // LM 迭代退出阈值条件
    double ni_;                 //控制 Lambda 缩放大小

    //---1.1 define ComputeLambdaInitLM
    std::function<void(void)> ComputeLambdaInitLM  = [&](){
        ni_ = 2.;
        currentLambda_ = -1.;
        currentChi_ = 0.0;

        for (auto edge: edges_) {
            currentChi_ += edge.second->RobustChi2();
        }
        if (err_prior_.rows() > 0)
            currentChi_ += err_prior_.squaredNorm();
        currentChi_ *= 0.5;

        stopThresholdLM_ = 1e-10 * currentChi_;          // 迭代条件为 误差下降 1e-6 倍

        double maxDiagonal = 0;
        ulong size = Hessian_.cols();
        assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
        for (ulong i = 0; i < size; ++i) {
            maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
        }

        maxDiagonal = std::min(5e10, maxDiagonal);
        double tau = 1e-5;  // 1e-5
        currentLambda_ = tau * maxDiagonal;
    };
    
    //---2. define "AddLambdaToHessian"
    std::function<void(MatXX& H)> AddLambdaToHessian = [&](MatXX& H){
        for (size_t i = 0; i < Hessian_.cols(); ++i) {
            H(i, i) += currentLambda_;
        }
    };

    //---3. define IsGoodStepInLM
    std::function<bool(void)> IsGoodStepInLM = [&]() -> bool{
        double scale = 0;
        scale = 0.5* delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
        scale += 1e-6;    // make sure it's non-zero :)

        // recompute residuals after update state
        double tempChi = 0.0;
        for (auto edge: edges_) {
            edge.second->ComputeResidual();
            tempChi += edge.second->RobustChi2();
        }
        if (err_prior_.size() > 0)
            tempChi += err_prior_.squaredNorm();
        tempChi *= 0.5;          // 1/2 * err^2

        double rho = (currentChi_ - tempChi) / scale;
        if (rho > 0 && isfinite(tempChi))   // last step was good, 误差在下降
        {
            double alpha = 1. - pow((2 * rho - 1), 3);
            alpha = std::min(alpha, 2. / 3.);
            double scaleFactor = (std::max)(1. / 3., alpha);
            currentLambda_ *= scaleFactor;
            ni_ = 2;
            currentChi_ = tempChi;
            return true;
        } else {
            currentLambda_ *= ni_;
            ni_ *= 2;
            return false;
        }
    };

    //---4. define RollbackStates
    std::function <void(void)> RollbackStates = [&]() {
            // update vertex
        for (auto vertex: verticies_) {
            vertex.second->RollBackParameters();
        }

        // Roll back prior_
        if (err_prior_.rows() > 0) {
            b_prior_ = b_prior_backup_;
            err_prior_ = err_prior_backup_;
        }
    };

    // ---5. now the LM process
    //---5.1 execute ComputeLambdaInitLM
    ComputeLambdaInitLM();
    bool stop = false;
    int iter = 0;
    double last_chi_ = 1e20;
    while (!stop && (iter < problem_config_.iteration_num)) {
        std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_ << std::endl;
        bool oneStepSuccess = false;
        int false_cnt = 0;
        while (!oneStepSuccess && false_cnt < 10){  // 不断尝试 Lambda, 直到成功迭代一步
            MatXX H_copy = Hessian_;
            AddLambdaToHessian(H_copy);
            ExecuteLinearSolverFunc_(H_copy, b_, delta_x_);
            UpdateStates();
            oneStepSuccess = IsGoodStepInLM();
            if (oneStepSuccess) {
                MakeHessian();
                false_cnt = 0;
            }
            else{
               RollbackStates(); 
               false_cnt = 0;
            }
        }
        iter++;
        if(last_chi_ - currentChi_ < 1e-5){
            std::cout << "sqrt(currentChi_) <= stopThresholdLM_" << std::endl;
            stop = true;
        }
        last_chi_ = currentChi_;
    }
    std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
    std::cout << "   makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;
    t_hessian_cost_ = 0.;
    
}

// --<nonlinear methods 3>
void ProblemBase::ProcessDogLeg(){

}

//--------------------Prior related funcs------------------
void ProblemBase::ResizePoseHessiansWhenAddingPose(shared_ptr<Vertex> v) {

    int size = H_prior_.rows() + v->LocalDimension();
    H_prior_.conservativeResize(size, size);
    b_prior_.conservativeResize(size);

    b_prior_.tail(v->LocalDimension()).setZero();
    H_prior_.rightCols(v->LocalDimension()).setZero();
    H_prior_.bottomRows(v->LocalDimension()).setZero();
}

void ProblemBase::ExtendHessiansPriorSize(int dim)
{
    int size = H_prior_.rows() + dim;
    H_prior_.conservativeResize(size, size);
    b_prior_.conservativeResize(size);

    b_prior_.tail(dim).setZero();
    H_prior_.rightCols(dim).setZero();
    H_prior_.bottomRows(dim).setZero();
}

}// namespace