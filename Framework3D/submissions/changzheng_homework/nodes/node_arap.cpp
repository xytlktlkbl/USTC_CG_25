#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h"
#include <cmath>
#include <time.h>
#include <Eigen/Dense>
#include <Eigen/SparseLU>
#include <Eigen/Sparse>
#include "../utils/hw5.h"
#include <vector>

#define Iterate_time 5

/*
** @brief HW5_ARAP_Parameterization
**
** This file presents the basic framework of a "node", which processes inputs
** received from the left and outputs specific variables for downstream nodes to
** use.
**
** - In the first function, node_declare, you can set up the node's input and
** output variables.
**
** - The second function, node_exec is the execution part of the node, where we
** need to implement the node's functionality.
**
** - The third function generates the node's registration information, which
** eventually allows placing this node in the GUI interface.
**
** Your task is to fill in the required logic at the specified locations
** within this template, especially in node_exec.
*/

NODE_DEF_OPEN_SCOPE
NODE_DECLARATION_FUNCTION(arap)
{
    // Input-1: Original 3D mesh with boundary
    // Maybe you need to add another input for initialization?
    b.add_input<Geometry>("3DInput");
    b.add_input<Geometry>("2DInitial");
    /*
    ** NOTE: You can add more inputs or outputs if necessary. For example, in
    ** some cases, additional information (e.g. other mesh geometry, other
    ** parameters) is required to perform the computation.
    **
    ** Be sure that the input/outputs do not share the same name. You can add
    ** one geometry as
    **
    **                b.add_input<Geometry>("Input");
    **
    ** Or maybe you need a value buffer like:
    **
    **                b.add_input<float1Buffer>("Weights");
    */

    // Output-1: The UV coordinate of the mesh, provided by ARAP algorithm
    b.add_output<Geometry>("ARAP");
}

NODE_EXECUTION_FUNCTION(arap)
{
 // Get the input from params
 auto input = params.get_input<Geometry>("3DInput");
 auto initial = params.get_input<Geometry>("2DInitial");


 // Avoid processing the node when there is no input
 if (!input.get_component<MeshComponent>() || !initial.get_component<MeshComponent>()) {
     throw std::runtime_error("Need Geometry Input.");
 }


   auto halfedge_mesh = operand_to_openmesh(&input);
   auto parm = operand_to_openmesh(&initial);

   //存储变化矩阵Lt的vector
   std::vector<Eigen::Matrix2f> Lts(halfedge_mesh->n_faces());

   //将原始3D网格展平的结果
   std::vector<Eigen::Matrix2f> x_flat(halfedge_mesh->n_faces());

   //目标2D网格
   std::vector<Eigen::Matrix2f> uv_flat(halfedge_mesh->n_faces());

   // 将边向量存储
   std::vector<Eigen::Vector2f> vec_x(halfedge_mesh->n_halfedges());
   //初始化x, uv
   set_x_uv(halfedge_mesh, parm, uv_flat, x_flat, vec_x);

   //计算cot，以备后续使用
   std::vector<double> cot(halfedge_mesh->n_halfedges(), 0.0f);
   calculate_cot(cot, halfedge_mesh);

   int fixed_idx_0, fixed_idx_1;
   double distance;


   //系数矩阵A
   Eigen::SparseMatrix<float> A(halfedge_mesh->n_vertices(), halfedge_mesh->n_vertices());
   Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
   //初始化系数矩阵并且预分解
   construct_Matrix(halfedge_mesh, cot, A, solver, fixed_idx_0, fixed_idx_1, distance);

   for (int t = 0; t < 1; ++t) {

       // Local phase
        Lt_construct(halfedge_mesh, cot, x_flat, uv_flat, Lts);

       // Global phase
       Eigen::MatrixXf B(halfedge_mesh->n_vertices(), 2);
       B.setZero();
       x_cof_construct(halfedge_mesh, x_flat, uv_flat, vec_x, cot, Lts, B, fixed_idx_0, fixed_idx_1, distance);
       solve(B, parm, solver);
       update(parm, uv_flat);
   }

   auto geometry = openmesh_to_operand(parm.get());

 // Set the output of the nodes
 params.set_output("ARAP", std::move(*geometry));
 return true;
}

NODE_DECLARATION_UI(arap);
NODE_DEF_CLOSE_SCOPE
