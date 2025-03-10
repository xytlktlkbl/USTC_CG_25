#include "RBF_warper.h"
#include <Eigen/Dense>
#include <cmath>

#define INF 1e5
#define EPSILON 1e-6

namespace USTC_CG{
    double RBFWarper::get_distance(Warper::point& p1, Warper::point& p2){
        return pow(pow(p1.start_x-p2.start_x, 2) + pow(p1.start_y-p2.start_y, 2), 0.5);
    }

    void RBFWarper::initial(){
        ini = true;
        g.clear();
        r.clear();
        if(selected_point.size() == 1){
            A.x11 = A.x22 = 1;
            g.push_back([](double d){ return pow(d * d + 10, 0.5);});
            struct Vector21 alpha_i = {(selected_point[0].end_x - selected_point[0].start_x)/g[0](0), 
                (selected_point[0].end_y - selected_point[0].start_y)/g[0](0)};
            alpha.push_back(alpha_i);
            return;
        }
        else{
            A.x11 = A.x22 = 1;
            // B默认初始化为0
            double r_i;
            for(auto p1: selected_point){
                r_i = INF;
                for(auto p2 : selected_point){
                    if(get_distance(p1, p2) < r_i && get_distance(p1, p2) > EPSILON)
                        r_i = get_distance(p1, p2);
                }
                r.push_back(r_i);
            }
            for(const double& r_i : r){
                g.push_back([r_i](double d){ return pow(d * d + r_i * r_i, 0.5);});
            }
            int n = selected_point.size();
            Eigen::MatrixXf A(n,n);
            Eigen::MatrixXf B(n,2);
            for(int i = 0; i < n; i++){
                for(int j = 0; j < n; j++){
                    A(i, j) = g[j](get_distance(selected_point[i], selected_point[j]));
                }
            }
            for(int i = 0; i < n; i++){
                B(i, 0) = selected_point[i].end_x - selected_point[i].start_x;
                B(i, 1) = selected_point[i].end_y - selected_point[i].start_y;
            }
            Eigen::MatrixXf Alpha(n,2);
            Alpha = A.householderQr().solve(B);
            for(int i = 0; i < n; i++){
                alpha.push_back({Alpha(i,0), Alpha(i, 1)});
            }
        }
    }

    void RBFWarper::warp(){
        if(!ini)
            initial();
        current_point.end_x = current_point.end_y = 0;
        for(int i = 0; i < selected_point.size(); i++){
            current_point.end_x += alpha[i].x11 * g[i](get_distance(current_point, selected_point[i]));
            current_point.end_y += alpha[i].x21 * g[i](get_distance(current_point, selected_point[i]));
        }
        current_point.end_x += current_point.start_x;
        current_point.end_y += current_point.start_y;
    }
}