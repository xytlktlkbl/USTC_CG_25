#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h"
#include <cmath>
#include <time.h>
#include <Eigen/Sparse>

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
NODE_DECLARATION_FUNCTION(min_surf)
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
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(min_surf)
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

    /* ----------------------------- Postprocess ------------------------------
    ** Convert the minimal surface mesh from the halfedge structure back to
    ** Geometry format as the node's output.
    */
    auto geometry = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*geometry));
    return true;
}

NODE_DECLARATION_UI(min_surf);
NODE_DEF_CLOSE_SCOPE
