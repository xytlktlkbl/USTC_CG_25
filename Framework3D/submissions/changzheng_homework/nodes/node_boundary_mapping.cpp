#include "GCore/Components/MeshOperand.h"
#include "geom_node_base.h"
#include "GCore/util_openmesh_bind.h"
#include <Eigen/Sparse>

    /*
    ** @brief HW4_TutteParameterization
    **
    ** This file contains two nodes whose primary function is to map the boundary of
    *a mesh to a plain
    ** convex closed curve (circle of square), setting the stage for subsequent
    *Laplacian equation
    ** solution and mesh parameterization tasks.
    **
    ** Key to this node's implementation is the adept manipulation of half-edge data
    *structures
    ** to identify and modify the boundary of the mesh.
    **
    ** Task Overview:
    ** - The two execution functions (node_map_boundary_to_square_exec,
    ** node_map_boundary_to_circle_exec) require an update to accurately map the
    *mesh boundary to a and
    ** circles. This entails identifying the boundary edges, evenly distributing
    *boundary vertices along
    ** the square's perimeter, and ensuring the internal vertices' positions remain
    *unchanged.
    ** - A focus on half-edge data structures to efficiently traverse and modify
    *mesh boundaries.
    */

#define Pi 3.1416
NODE_DEF_OPEN_SCOPE

    /*
    ** HW4_TODO: Node to map the mesh boundary to a circle.
    */

NODE_DECLARATION_FUNCTION(boundary_mapping)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");
    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the
    // interior vertices remains the same
    b.add_output<Geometry>("circle_boundary");
    b.add_output<Geometry>("square_boundary");
}

NODE_EXECUTION_FUNCTION(boundary_mapping)
{
    // Get the input from params
    auto input = params.get_input<Geometry>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Boundary Mapping: Need Geometry Input.");
    }
    //throw std::runtime_error("Not implemented");

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);

    /* ----------- [HW4_TODO] TASK 2.1: Boundary Mapping (to circle)
    *------------
    ** In this task, you are required to map the boundary of the mesh to a
    *circle
    ** shape while ensuring the internal vertices remain unaffected. This step
    *is
    ** crucial for setting up the mesh for subsequent parameterization tasks.
    **
    ** Algorithm Pseudocode for Boundary Mapping to Circle
    ** ------------------------------------------------------------------------
    ** 1. Identify the boundary loop(s) of the mesh using the half-edge
    *structure.
    **
    ** 2. Calculate the total length of the boundary loop to determine the
    *spacing
    **    between vertices when mapped to a square.
    **
    ** 3. Sequentially assign each boundary vertex a new position along the
    *square's
    **    perimeter, maintaining the calculated spacing to ensure proper
    *distribution.
    **
    ** 4. Keep the interior vertices' positions unchanged during this process.
    **
    ** Note: How to distribute the points on the circle?
    **
    ** Note: It would be better to normalize the boundary to a unit circle in
    *[0,1]x[0,1] for
    ** texture mapping.
    */
   std::vector<OpenMesh::SmartHalfedgeHandle> boundary;
   std::vector<double> length;
   std::vector<double> theta;
   double sum = 0;
   OpenMesh::SmartVertexHandle start;
   OpenMesh::SmartEdgeHandle first_edge;
    for(auto& vertex_handle : halfedge_mesh->all_vertices()){
        if(vertex_handle.is_boundary()){
            start = vertex_handle;
            break;
        }
    }
    auto end = start;

    do{
        for(auto edge : end.outgoing_halfedges()){
            if(edge.is_boundary()){
                boundary.push_back(edge);
                end = edge.to();
            }
        }

    }while(start != end);

    for(const auto& edge : boundary){
        const auto& vec = halfedge_mesh->point(edge.to()) - halfedge_mesh->point(edge.from());
        length.push_back(vec.norm());
        sum += vec.norm();
    }

    for(int i = 0; i < length.size(); i++){
        theta.push_back(length[i] / sum * 2 * Pi);
    }
    double theta_sum = 0;
    int boundary_size = boundary.size();
    for(int i = 0; i < boundary_size; i++){
        auto current_point = boundary[i].from();
        if(i == 0){
            auto& position = halfedge_mesh->point(current_point);
            position[0] = 1;
            position[1] = 0.5;
            position[2] = 0;
        }
        else{
            theta_sum += theta[i-1];
            double new_x = cos(theta_sum) * 0.5 + 0.5;
            double new_y = sin(theta_sum) * 0.5 + 0.5;
            auto& position = halfedge_mesh->point(current_point);
            position[0] = new_x;
            position[1] = new_y;
            position[2] = 0;
        }
    }

    /* ----------------------------- Postprocess ------------------------------
    ** Convert the result mesh from the halfedge structure back to Geometry
    *format as the node's
    ** output.
    */
    auto geometry = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("circle_boundary", std::move(*geometry));

    //square
    std::vector<double> mod;
    std::vector<int> div;
    sum = 0;
    for(int i = 0; i < length.size(); i++){
        mod.push_back(sum);
        sum += length[i];
    }
    double length_square = sum / 4;
    for(int i = 0; i < mod.size(); i++){
        div.push_back(int(mod[i]/ length_square));
        mod[i] = mod[i] - div[i] * length_square;
    }

    for(int i = 0; i < boundary_size; i++){
        auto current_point = boundary[i].from();
            double new_x = 0;
            double new_y = 0;
            if(div[i] == 0 && div[i+1] != 1){
                new_x = mod[i] / length_square;
                new_y = 0;
            }
            else if(div[i] == 0 && div[i+1] == 1){
                if(length_square - mod[i] < mod[i+1]){
                    new_x = 1;
                    new_y = 0;
                }
                else{
                    new_x = mod[i] / length_square;
                    new_y = 0;
                    auto& position = halfedge_mesh->point(current_point);
                    position[0] = new_x;
                    position[1] = new_y;
                    position[2] = 0;
                    auto current_point = boundary[++i].from();
                    auto& position1 = halfedge_mesh->point(current_point);
                    position1[0] = 1;
                    position1[1] = 0;
                    position1[2] = 0;
                    continue;
                }
            }
            else if(div[i] == 1 && div[i+1] != 2){
                new_x = 1;
                new_y = mod[i] / length_square;
            }
            else if(div[i] == 1 && div[i+1] == 2){
                if(length_square - mod[i] < mod[i+1]){
                    new_x = 1;
                    new_y = 1;
                }
                else{
                    new_x = 1;
                    new_y = mod[i] / length_square;
                    auto& position = halfedge_mesh->point(current_point);
                    position[0] = new_x;
                    position[1] = new_y;
                    position[2] = 0;
                    auto current_point = boundary[++i].from();
                    auto& position1 = halfedge_mesh->point(current_point);
                    position1[0] = 1;
                    position1[1] = 1;
                    position1[2] = 0;
                    continue;
                }
            }

            else if(div[i] == 2 && div[i+1] != 3){
                new_x = 1 - mod[i] / length_square;
                new_y = 1;
            }
            else if(div[i] == 2 && div[i+1] == 3){
                if(length_square - mod[i] < mod[i+1]){
                    new_x = 0;
                    new_y = 1;
                }
                else{
                    new_x = 1 - mod[i] / length_square;
                    new_y = 1;
                    auto& position = halfedge_mesh->point(current_point);
                    position[0] = new_x;
                    position[1] = new_y;
                    position[2] = 0;
                    auto current_point = boundary[++i].from();
                    auto & position1 = halfedge_mesh->point(current_point);
                    position1[0] = 0;
                    position1[1] = 1;
                    position1[2] = 0;
                    continue;
                }
            }
            else if(div[i] == 3){
                new_x = 0;
                new_y = 1 - mod[i] / length_square;
            }
            auto& position = halfedge_mesh->point(current_point);
            position[0] = new_x;
            position[1] = new_y;
            position[2] = 0;
    }
        /* ----------------------------- Postprocess ------------------------------
    ** Convert the result mesh from the halfedge structure back to Geometry
    *format as the node's
    ** output.
    */
   geometry = openmesh_to_operand(halfedge_mesh.get());

   // Set the output of the nodes
   params.set_output("square_boundary", std::move(*geometry));
   return true;
}




NODE_DECLARATION_UI(boundary_mapping);
NODE_DEF_CLOSE_SCOPE
