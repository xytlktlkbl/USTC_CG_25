#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h"
#include <cmath>
#include <time.h>
#include <Eigen/Sparse>
#include "../utils/min_surf.h"
#define Pi 3.1416
#include <utility>

    /*
    ** @brief HW4_TutteParameterization
    **
    ** This file presents the basic framework of a "node", which processes inputs
    ** received from the left and outputs specific variables for downstream nodes to
    ** use.
    ** - In the first function, node_declare, you can set up the node's input and
    ** output variables.
    ** - The second function, node_exec is the execution part of the node, where we
    ** need to implement the node's functionality.
    ** Your task is to fill in the required logic at the specified locations
    ** within this template, especially in node_exec.
    */

NODE_DEF_OPEN_SCOPE
NODE_DECLARATION_FUNCTION(min_surf_uniform)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");

    /*
    ** NOTE: You can add more inputs or outputs if necessary. For example, in
    *some cases,
    ** additional information (e.g. other mesh geometry, other parameters) is
    *required to perform
    ** the computation.
    **
    ** Be sure that the input/outputs do not share the same name. You can add
    *one geometry as
    **
    **                b.add_input<Geometry>("Input");
    **
    ** Or maybe you need a value buffer like:
    **
    **                b.add_input<float1Buffer>("Weights");
    */

    // Output-1: Minimal surface with fixed boundary
    b.add_output<Geometry>("Uniform");
}

NODE_EXECUTION_FUNCTION(min_surf_uniform)
{
    // Get the input from params
    auto input = params.get_input<Geometry>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
        return false;
    }

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);

    /* ---------------- [HW4_TODO] TASK 1: Minimal Surface --------------------
    ** In this task, you are required to generate a 'minimal surface' mesh with
    ** the boundary of the input mesh as its boundary.
    **
    ** Specifically, the positions of the boundary vertices of the input mesh
    ** should be fixed. By solving a global Laplace equation on the mesh,
    ** recalculate the coordinates of the vertices inside the mesh to achieve
    ** the minimal surface configuration.
    **
    ** (Recall the Poisson equation with Dirichlet Boundary Condition in HW3)
    */

    /*
    ** Algorithm Pseudocode for Minimal Surface Calculation
    ** ------------------------------------------------------------------------
    ** 1. Initialize mesh with input boundary conditions.
    **    - For each boundary vertex, fix its position.
    **    - For internal vertices, initialize with initial guess if necessary.
    **    
    ** 2. Construct Laplacian matrix for the mesh.
    **    - Compute weights for each edge based on the chosen weighting scheme
    **      (e.g., uniform weights for simplicity).
    **    - Assemble the global Laplacian matrix.
    **
    ** 3. Solve the Laplace equation for interior vertices.
    **    - Apply Dirichlet boundary conditions for boundary vertices.
    **    - Solve the linear system (Laplacian * X = 0) to find new positions
    **      for internal vertices.
    **
    ** 4. Update mesh geometry with new vertex positions.
    **    - Ensure the mesh respects the minimal surface configuration.
    **
    ** Note: This pseudocode outlines the general steps for calculating a
    ** minimal surface mesh given fixed boundary conditions using the Laplace
    ** equation. The specific implementation details may vary based on the mesh
    ** representation and numerical methods used.
    **
    */
   //uniform
   Eigen::SparseMatrix<double> A; // 矩阵A
   Eigen::SparseLU<Eigen::SparseMatrix<double>> solver; // 求解器
   int num_vertices =  halfedge_mesh->n_vertices();
   A.resize(num_vertices, num_vertices);
   Eigen::MatrixXd B(num_vertices, 3); //除边界点外均为0
   std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
   for(const auto& vertex_handle : halfedge_mesh->vertices()){
       int index = vertex_handle.idx();
       auto& position = halfedge_mesh->point(vertex_handle);
       if(vertex_handle.is_boundary()){
           triplet_list.push_back(Eigen::Triplet<double>(index, index, 1));
           B(index, 0) = position[0];
           B(index, 1) = position[1];
           B(index, 2) = position[2];
       }
       else{
           std::vector<int> neibour_index;
           for(const auto& halfedge_handle : vertex_handle.outgoing_halfedges()){
               const auto& v1 = halfedge_handle.to();
               neibour_index.push_back(v1.idx());
           }
           double size = neibour_index.size();
           for(int i = 0; i < size; i++){
               triplet_list.push_back(Eigen::Triplet<double>(index, neibour_index[i], -1/size));
           }
           triplet_list.push_back(Eigen::Triplet<double>(index, index, 1));
           B(index, 0) = 0;
           B(index, 1) = 0;
           B(index, 2) = 0;
       }
   }
    A.setFromTriplets(triplet_list.begin(), triplet_list.end());
    solver.compute(A); // 分解矩阵 A
    Eigen::MatrixXd r = solver.solve(B);
    for(auto& vertex_handle : halfedge_mesh->vertices()){
        int index = vertex_handle.idx();
        auto& position = halfedge_mesh->point(vertex_handle);
        position[0] = r(index, 0);
        position[1] = r(index, 1);
        position[2] = r(index, 2);
    }
    triplet_list.clear();
    auto geometry = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Uniform", std::move(*geometry));
    return true;
}


    /* ----------------------------- Postprocess ------------------------------
    ** Convert the minimal surface mesh from the halfedge structure back to
    ** Geometry format as the node's output.
    */
   NODE_DECLARATION_FUNCTION(min_surf_Cotangent)
{
    b.add_input<Geometry>("Input");
    b.add_input<Geometry>("reference_mesh");
    b.add_output<Geometry>("output");
}


NODE_EXECUTION_FUNCTION(min_surf_Cotangent)
{
    // Get the input from params
    auto input = params.get_input<Geometry>("Input");
    auto origin = params.get_input<Geometry>("reference_mesh");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>() || !origin.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
        return false;
    }

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);
    auto origin_mesh = operand_to_openmesh(&origin);

    Eigen::SparseMatrix<double> A2; // 矩阵A
    int num_vertices =  halfedge_mesh->n_vertices();
    std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
    A2.resize(num_vertices, num_vertices);
    Eigen::MatrixXd B2(num_vertices, 3); //除边界点外均为0
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver; // 求解器
    for(const auto& vertex_handle : origin_mesh->vertices()){
        int index = vertex_handle.idx();
        auto& position = halfedge_mesh->point(vertex_handle);
        if(vertex_handle.is_boundary()){
           triplet_list.push_back(Eigen::Triplet<double>(index, index, 1));
           B2(index, 0) = position[0];
           B2(index, 1) = position[1];
           B2(index, 2) = position[2];
        }
        else{
            std::vector<int> neibour_index;
            std::vector<double> weight;
            double sum = 0;
            for(const auto& halfedge_handle : vertex_handle.edges_ccw()){
                const auto& v0 = halfedge_handle.v0();
                const auto& v1 = halfedge_handle.v1();
                if(v0.idx() != vertex_handle.idx())
                    neibour_index.push_back(v0.idx());
                else
                    neibour_index.push_back(v1.idx());
            }
            int size = neibour_index.size();
            for(int i = 0; i < size; i++){
                auto vex = origin_mesh->vertex_handle(vertex_handle.idx());
                auto current = origin_mesh->vertex_handle(neibour_index[i]);
                auto next = origin_mesh->vertex_handle(neibour_index[(i + 1) % size]);
                auto prev = origin_mesh->vertex_handle(neibour_index[(i + size - 1) % size]);
                auto vec1 = origin_mesh->point(current) - origin_mesh->point(prev);
                auto vec2 = origin_mesh->point(vex) - origin_mesh->point(prev);
                double beta = acosf(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
                vec1 = origin_mesh->point(current) - origin_mesh->point(next);
                vec2 = origin_mesh->point(vex) - origin_mesh->point(next);
                double alpha = acosf(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
                double weight_this = 1 / tan(alpha) + 1 / tan(beta);
                sum += weight_this;
                weight.push_back(weight_this);
            }
            for(int i = 0; i < size; i++){
                triplet_list.push_back(Eigen::Triplet<double>(index, neibour_index[i], -1 * weight[i]/sum));
            }
            triplet_list.push_back(Eigen::Triplet<double>(index, index, 1));
            B2(index, 0) = 0;
            B2(index, 1) = 0;
            B2(index, 2) = 0;
       }
   }
    A2.setFromTriplets(triplet_list.begin(), triplet_list.end());
    solver.compute(A2); // 分解矩阵 A
    Eigen::MatrixXd r2 = solver.solve(B2);
    for(auto& vertex_handle : halfedge_mesh->vertices()){
        int index = vertex_handle.idx();
        auto& position = halfedge_mesh->point(vertex_handle);
        position[0] = r2(index, 0);
        position[1] = r2(index, 1);
        position[2] = r2(index, 2);
    }
    /* ----------------------------- Postprocess ------------------------------
    ** Convert the minimal surface mesh from the halfedge structure back to
    ** Geometry format as the node's output.
    */
   auto geometry = openmesh_to_operand(halfedge_mesh.get());

   // Set the output of the nodes
   params.set_output("output", std::move(*geometry));
   return true;

}

NODE_DECLARATION_FUNCTION(min_surf_shape_preserving)
{
    b.add_input<Geometry>("Input");
    b.add_input<Geometry>("reference_mesh");
    b.add_output<Geometry>("output");
}

NODE_EXECUTION_FUNCTION(min_surf_shape_preserving)
{
    auto input = params.get_input<Geometry>("Input");
    auto origin = params.get_input<Geometry>("reference_mesh");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>() || !origin.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
        return false;
    }
    auto halfedge_mesh = operand_to_openmesh(&input);
    auto origin_mesh = operand_to_openmesh(&origin);
    Eigen::SparseMatrix<double> A2; // 矩阵A
    int num_vertices =  halfedge_mesh->n_vertices();
    std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
    A2.resize(num_vertices, num_vertices);
    Eigen::MatrixXd B2(num_vertices, 3); //除边界点外均为0
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver; // 求解器

    for(const auto& vertex_handle : origin_mesh->vertices()){
        int index = vertex_handle.idx();
        auto& position = halfedge_mesh->point(vertex_handle);
        if(vertex_handle.is_boundary()){
           triplet_list.push_back(Eigen::Triplet<double>(index, index, 1));
           B2(index, 0) = position[0];
           B2(index, 1) = position[1];
           B2(index, 2) = position[2];
        }
        else{
            std::vector<int> neibour_index;
            for(const auto& halfedge_handle : vertex_handle.edges_ccw()){
                const auto& v0 = halfedge_handle.v0();
                const auto& v1 = halfedge_handle.v1();
                if(v0.idx() != vertex_handle.idx())
                    neibour_index.push_back(v0.idx());
                else
                    neibour_index.push_back(v1.idx());
            }
            int size = neibour_index.size();
            std::vector<double> weight;
            weight_shape_preserving(neibour_index, origin_mesh, index, weight);
            for(int i = 0; i < size; i++){
                triplet_list.push_back(Eigen::Triplet<double>(index, neibour_index[i], -1 * weight[i]));
            }
            triplet_list.push_back(Eigen::Triplet<double>(index, index, 1));
            B2(index, 0) = 0;
            B2(index, 1) = 0;
            B2(index, 2) = 0;
        }
    }


    A2.setFromTriplets(triplet_list.begin(), triplet_list.end());
    solver.compute(A2); // 分解矩阵 A
    Eigen::MatrixXd r2 = solver.solve(B2);
    for(auto& vertex_handle : halfedge_mesh->vertices()){
        int index = vertex_handle.idx();
        auto& position = halfedge_mesh->point(vertex_handle);
        position[0] = r2(index, 0);
        position[1] = r2(index, 1);
        position[2] = r2(index, 2);
    }
    /* ----------------------------- Postprocess ------------------------------
    ** Convert the minimal surface mesh from the halfedge structure back to
    ** Geometry format as the node's output.
    */
   auto geometry = openmesh_to_operand(halfedge_mesh.get());

   // Set the output of the nodes
   params.set_output("output", std::move(*geometry));
   return true;
}

NODE_DECLARATION_UI(min_surf);
NODE_DEF_CLOSE_SCOPE
