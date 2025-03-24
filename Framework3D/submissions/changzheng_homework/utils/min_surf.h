#pragma once

#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h"
#include <cmath>
#include <time.h>
#include <Eigen/Dense>
#include "../utils/min_surf.h"
#define Pi 3.1416
#include <utility>

void weight_shape_preserving(const std::vector<int>& neibour_index, auto origin_mesh, int index,
    std::vector<double>& weight){
    using Point = std::pair<double, double>;
    std::vector<double> length;
    std::vector<double> theta;
    int size = neibour_index.size();
    weight.resize(size, 0.0);
    if(size <= 2){
        for(int i = 0; i < size; i++){
            weight.push_back(1.0 / size);
        }
        return;
    }
    double theta_sum = 0;
    for(int i = 0 ; i < size; i++){
        auto vex = origin_mesh->vertex_handle(index);
        auto current = origin_mesh->vertex_handle(neibour_index[i]);
        auto next = origin_mesh->vertex_handle(neibour_index[(i + 1) % size]);
        auto vec1 = origin_mesh->point(current) - origin_mesh->point(vex);
        auto vec2 = origin_mesh->point(next) - origin_mesh->point(vex);
        length.push_back(vec1.norm());
        double theta_this = acosf(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
        theta.push_back(theta_this);
        theta_sum += theta_this;
    }
    for(auto theta_this : theta){
        theta_this = theta_this / theta_sum * 2 * Pi;
    }
    theta_sum = 0;
    std::vector<Point> points;
    for(int i = 0; i < size; i++){
        double x = length[i] * cos(theta_sum);
        double y = length[i] * sin(theta_sum);
        points.push_back(std::make_pair(x, y));
        theta_sum += theta[i];
    }
    for(int i = 0; i < size; i++){
        for(int j = 0; j < size; j++){
            if(j == i)
                continue;
            double x_1 = points[j].first;
            double y_1 = points[j].second;
            double x_2 = points[(j+1) % size].first;
            double y_2 = points[(j+1) % size].second;
            double x = points[i].first;
            double y = points[i].second;
            if(((y / x * x_1) - y_1) * ((y / x * x_2) - y_2) > 0)
                continue;
            
            Eigen::MatrixXd A(3, 3); 
            Eigen::MatrixXd B(3, 1); 
            A(0,0) = x_1, A(1,0) = y_1, A(2,0) = 1;
            A(0,1) = x_2, A(1,1) = y_2, A(2,1) = 1;
            A(0,2) = x, A(1, 2) = y, A(2, 2) = 1;
            B(0,0) = B(1,0) = 0;
            B(2,0) = 1;
            Eigen::MatrixXd result_x = A.lu().solve(B);
            if(result_x(0,0) <= 0 || result_x(1,0)<=0 || result_x(2,0) < 0)
                continue;
            weight[j] += result_x(0,0) / size;
            weight[(j+1) % size] += result_x(1,0) / size;
            weight[i] += result_x(2,0) / size;
        }
    }
}