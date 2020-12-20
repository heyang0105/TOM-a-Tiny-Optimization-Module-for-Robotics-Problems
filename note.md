2020/12/21
plan:
    return bool of nonlinear types

2020/12/17
bug: --DONE
    size_t Vertex::global_vertex_id_ = 0;   
    size_t Edge::global_edge_id_ = 0;
    多线程，每个线程有一个OPTIM的时候

    problemBase Provide it, change the way each edge and vertex init

plan: 
    problem_dense --DONE
    modify pnp to test the new problem --DONE

2020/12/16
Design:
1. Remove all Prior mat related problem into the spec-problem
2. MakeHessian() --DONE 
    the prior part needs to be splited as a additional func 
3. linearsolver, put arg in the Solve ->DONE
4. UpdateStates() , prior --DONE 
    should override it in the spec problem
5. Q: put prior in base or specific problem??? --DONE
    -- part in base, since it is embed in the nonlinear procudure
    -- part in spec, prior is mostly related to the pose vertex
6. using the combination of lambda function to do the nonlinaer process

2020/12/12
1. plan: inv depth edge ->DONE< TOBE TESTED>
2. plan & Q: if direct method can be set in this lib?
    yes, refer it to the slambook 
3. plan: go through problem.cc
    -a. problem_base --DONE
    -b. problem_sparse_VO
    -c. problem_dense_general  --DONE
    -d. linear_solver  ->DONE< TOBE TESTED>


2020/12/11
1. plan: add check using the vertex type_name_ 
add: EdgeMonoReprojectionPoseOnly --DONE

Q: How to jacobian the transpose of a Rotation --DONE
    using the property of a skewsymetric mat

2020/12/10
1. plan: add pangolin to show the BA process
2. plan: add stereo edge
3. plan: test stereo BA

Q:
1. How to define pose T_wc or T_cw ->DONE
    T_cw to project
2. How to define right or left optim, and difference -> DONE

2020/12/09
1. 建立core
2. 建立test
3. plan: 添加 poseVertex pointXYZVertex -> DONE
4. plan:  added pointXYZProjectionEdge -> DONE
5. plan: 
        1. complete the test of the MONO BA -> REMOVE
        2. test using slambook 2 chapter 7, PNP
            -> BUG: lambda is too huge in 2nd iteration
            -DONE, 
            # schur trick should not be used in DENSE PROBELM

