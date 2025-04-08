

//构造左侧系数矩阵
void construct_Matrix(auto& halfedge_mesh, std::vector<double> &cot, Eigen::SparseMatrix<float> &A, 
    Eigen::SparseLU<Eigen::SparseMatrix<float>> &solver, int& fixed_idx_0, int& fixed_idx_1, double& distance);
// 用于计算余切，其中idx是顶点，idx1,idxj是两条边上的顶点
void calculate_cot(std::vector<double>& cot, auto& halfedge_mesh_3D);
//将3D网格的坐标映射到二维存在uv_result中
void set_x_uv(auto& halfedge_mesh_3D, auto& halfedge_mesh_2D, std::vector<Eigen::Matrix2f>& uv_result, 
    std::vector<Eigen::Matrix2f>& x_result, std::vector<Eigen::Vector2f>& vec_x);
//基于2D和3D网格构造Lt
void Lt_construct(auto &halfedge_mesh_3D, std::vector<double>& cotangent, std::vector<Eigen::Matrix2f>& x_result, 
    std::vector<Eigen::Matrix2f>& uv_result, std::vector<Eigen::MatrixXf> &Lts);
//坐标更新
void update(auto& halfedge_mesh_2D , pxr::VtArray<pxr::GfVec2f> &uv_result);
//构造右侧系数矩阵
void x_cof_construct(auto& halfedge_mesh, std::vector<Eigen::Matrix2f>& x_result,std::vector<Eigen::Matrix2f>& uv_result,
    std::vector<Eigen::Vector2f>& vec_x,std::vector<double> cotangent, const std::vector<Eigen::MatrixXf>& Lts, 
    Eigen::MatrixXd& B, int fixed_idx_0, int fixed_idx_1, double distance) ;
//求解
void solve(Eigen::MatrixXd &B, auto &halfedge_mesh_2D, Eigen::SparseLU<Eigen::SparseMatrix<double>> &solver);



void construct_Matrix(auto& halfedge_mesh, std::vector<double>& cot, Eigen::SparseMatrix<float> &A, 
    Eigen::SparseLU<Eigen::SparseMatrix<float>> &solver, int& fixed_idx_0, int& fixed_idx_1,double& distance){
    int num_v = halfedge_mesh->n_vertices();
    A.resize(num_v, num_v);
    A.setZero();
    std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
    auto edge = *(halfedge_mesh->edges_begin());

    auto vec = halfedge_mesh->point(edge.v0()) - halfedge_mesh->point(edge.v1());
    distance = vec.norm();
    fixed_idx_0 = edge.v0().idx();
    fixed_idx_1 = edge.v1().idx();

    for(auto v : halfedge_mesh->vertices()){
        int v_idx = v.idx();
        if (v_idx == fixed_idx_0){
            triplet_list.push_back(Eigen::Triplet<double>(v_idx, v_idx, 1));
            continue;
        }
        if (v_idx == fixed_idx_1){
            triplet_list.push_back(Eigen::Triplet<double>(v_idx, v_idx, 1));
            continue;
        }
        double cof = 0;
        for(const auto& halfedge_handle : v.outgoing_halfedges()){
            int n_v_idx = halfedge_handle.to().idx();
            double cot_weight = 0;
            if(!halfedge_handle.is_boundary()){
                cot_weight += cot[halfedge_handle.idx()];
            }
            auto opp = halfedge_handle.opp();
            if (!opp.is_boundary()) { 
                cot_weight += cot[opp.idx()];
            }        
            cof += cot_weight;
            triplet_list.push_back(Eigen::Triplet<double>(v_idx, n_v_idx, -1 * cot_weight));
        }
        triplet_list.push_back(Eigen::Triplet<double>(v_idx, v_idx, cof));
    }
    A.setFromTriplets(triplet_list.begin(), triplet_list.end());
    /*for(int i = 0 ; i < A.rows(); i++){
        for(int j = 0; j < A.cols(); j++)
            printf("%lf\n", A.coeff(i, j));
    }*/
    solver.compute(A);
}

void Lt_construct(auto &halfedge_mesh, std::vector<double>& cotangent, std::vector<Eigen::Matrix2f>& x_result, 
    std::vector<Eigen::Matrix2f>& uv_result, std::vector<Eigen::Matrix2f> &Lts){
    int face_num = halfedge_mesh->n_faces();
    Lts.clear();
    Lts.resize(halfedge_mesh->n_faces());
    Eigen::Matrix2f S_t;
    Eigen::Vector2f U_t;
    Eigen::Vector2f X_t;
/*
       for (auto face = halfedge_mesh->faces_begin(); face != halfedge_mesh->faces_end(); face++) {
        S_t.setZero();
        int idx = face->idx();

        auto edge = face->halfedge();
        U_t(0) = 0 - uv_result[idx](0,0);
        U_t(1) = 0 - uv_result[idx](1,0);
        X_t(0) = 0 - x_result[idx](0,0);
        X_t(1) = 0 - x_result[idx](1,0);
        //printf("U:%lf %lf\n", U_t(0), U_t(1));
        //printf("X:%lf %lf\n", X_t(0), X_t(1));
        if(!edge.is_boundary())
            S_t += (cotangent[edge.idx()] * U_t) * (X_t.transpose());
        edge = edge.next();

        U_t(0) = uv_result[idx](0,0) - uv_result[idx](0,1);
        U_t(1) = uv_result[idx](1,0) - uv_result[idx](1,1);
        X_t(0) = x_result[idx](0,0) - x_result[idx](0,1);
        X_t(1) = x_result[idx](1,0) - x_result[idx](1,1);
        //printf("U:%lf %lf\n", U_t(0), U_t(1));
        //printf("X:%lf %lf\n", X_t(0), X_t(1));
        if(!edge.is_boundary())
            S_t += (cotangent[edge.idx()] * U_t) * (X_t.transpose());
        edge = edge.next();

        U_t(0) = uv_result[idx](0,1) - 0;
        U_t(1) = uv_result[idx](1,1) - 0;
        X_t(0) = x_result[idx](0,1) - 0;
        X_t(1) = x_result[idx](1,1) - 0;
        //printf("U:%lf %lf\n", U_t(0), U_t(1));
        //printf("X:%lf %lf\n", X_t(0), X_t(1));
        if(!edge.is_boundary())
            S_t += (cotangent[edge.idx()] * U_t) * (X_t.transpose());
        edge = edge.next();
        //printf("S:%lf %lf %lf %lf\n", S_t(0, 0), S_t(0, 1), S_t(1, 0), S_t(1, 1));

        Eigen::JacobiSVD<Eigen::Matrix2f> svd(S_t, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2f U = svd.matrixU();
        Eigen::Matrix2f V = svd.matrixV();
        if((U*(V.transpose())).determinant() < 0){
            U(0, 1) *= -1;
            U(1, 1) *= -1;
        }
        Eigen::Matrix2f L1 =  U*(V.transpose());
        //printf("%lf %lf %lf %lf\n", L(0, 0), L(0, 1), L(1, 0), L(1, 1));
        Lts[idx] = L1;
        


    }*/
    for (const auto& face_handle : halfedge_mesh->faces()) {

        // Get the points' IDs in two meshes
        int idx = face_handle.idx();

        // Figure out L by SVD
        Eigen::MatrixXf J = uv_result[idx] * x_result[idx].inverse();
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix2f U = svd.matrixU();
        Eigen::Matrix2f V = svd.matrixV();
        Lts[idx] = U * V.transpose();
    }
    
}

void calculate_cot(std::vector<double>& cot, auto& halfedge_mesh_3D){
    for (const auto& edge : halfedge_mesh_3D->halfedges()) {
        if (!edge.is_boundary()) {
            auto point0 = edge.from();
            auto point1 = edge.to();
            auto point2 = edge.next().to();
            auto vec1 = halfedge_mesh_3D->point(point0) - halfedge_mesh_3D->point(point2);
            auto vec2 = halfedge_mesh_3D->point(point1) - halfedge_mesh_3D->point(point2);
            double cos = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
            cot[edge.idx()] = cos / sqrt(1 - pow(cos, 2));
        }
    }
}

void x_cof_construct(auto& halfedge_mesh, std::vector<Eigen::Matrix2f>& x_result,std::vector<Eigen::Matrix2f>& uv_result,
    std::vector<Eigen::Vector2f>& vec_x, std::vector<double> cot, const std::vector<Eigen::Matrix2f>& Lts, Eigen::MatrixXf& B,
    int fixed_idx_0, int fixed_idx_1, double distance) {
    int v_num = halfedge_mesh->n_vertices();
    B.resize(v_num, 2);
    B.setZero();
    for (auto v : halfedge_mesh->vertices()) {
        int v_idx = v.idx();
        if(v_idx == fixed_idx_0){
            B(fixed_idx_0, 0) = 0;
            B(fixed_idx_0, 1) = 0;
            continue;
        }
        if(v_idx == fixed_idx_1){
            B(fixed_idx_1, 0) = distance;
            B(fixed_idx_1, 1) = 0;
            continue;
        }

        Eigen::Vector2f term;
        term.setZero();
        for (const auto& halfedge_handle : v.outgoing_halfedges()) { 

            if (!halfedge_handle.is_boundary()) {
                term += cot[halfedge_handle.idx()] * Lts[halfedge_handle.face().idx()] * vec_x[halfedge_handle.idx()];
            }

            auto opp = halfedge_handle.opp();
            if (!opp.is_boundary()) {
                term -= cot[opp.idx()] * Lts[opp.face().idx()] * vec_x[opp.idx()];
            }
        }
        B(v_idx, 0) = term(0);
        B(v_idx, 1) = term(1);
    }
}

void solve(Eigen::MatrixXf &B, auto &halfedge_mesh_2D, Eigen::SparseLU<Eigen::SparseMatrix<float>> &solver){
    Eigen::MatrixXf U = solver.solve(B);

    for(auto v : halfedge_mesh_2D->vertices()){
        int i = v.idx();
        auto& position = halfedge_mesh_2D->point(v);
        position[0] = U(i, 0);
        position[1] = U(i, 1);
        position[2] = 0;
    } 
}

void update(auto& halfedge_mesh ,  std::vector<Eigen::Matrix2f> &uv_result){
    for (auto& face : halfedge_mesh->faces()) {
        int idx = face.idx();
        auto edge = face.halfedge();
        auto point0 = edge.from();
        auto point1 = edge.to();
        auto point2 = edge.next().to();
        const auto& vec1_u = halfedge_mesh->point(point1) - halfedge_mesh->point(point0);
        const auto& vec2_u = halfedge_mesh->point(point2) - halfedge_mesh->point(point0);
        uv_result[idx] << vec1_u[0], vec2_u[0], vec1_u[1], vec2_u[1];
    }   
}

void set_x_uv(auto& halfedge_mesh_3D, auto& halfedge_mesh_2D, std::vector<Eigen::Matrix2f>& uv_result, 
    std::vector<Eigen::Matrix2f>& x_result, std::vector<Eigen::Vector2f>& vec_x){
    //将3D网格的坐标映射到二维存在x_result中
    //将2D网格的坐标映存到uv_result
    for (auto& face : halfedge_mesh_3D->faces()) {
        int idx = face.idx();
        auto edge = face.halfedge();
        auto point0 = edge.from();
        auto point1 = edge.to();
        auto point2 = edge.next().to();

        const auto& vec1_x = halfedge_mesh_3D->point(point1) - halfedge_mesh_3D->point(point0);
        const auto& vec2_x = halfedge_mesh_3D->point(point2) - halfedge_mesh_3D->point(point0);
        float length1 = vec1_x.norm();
        float length2 = vec2_x.norm();
        float cos = vec1_x.dot(vec2_x) / (vec1_x.norm() * vec2_x.norm());
        float theta = acosf(cos);
        vec_x[edge.idx()] = { -length1, 0.0f };
        vec_x[edge.next().idx()] = { length1 - length2 * cos, -length2 * sin(theta) };
        vec_x[edge.next().next().idx()] = { length2 * cos, length2 * sin(theta) };
        const auto& vec1_u = halfedge_mesh_2D->point(point1) - halfedge_mesh_2D->point(point0);
        const auto& vec2_u = halfedge_mesh_2D->point(point2) - halfedge_mesh_2D->point(point0);
        x_result[idx] << length1, length2 * cos, 0, length2 * sin(theta);
        uv_result[idx] << vec1_u[0], vec2_u[0], vec1_u[1], vec2_u[1];
    }
}