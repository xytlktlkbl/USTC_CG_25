#include "IDW_warper.h"
#include <Eigen/Dense>
#include <cmath>

#define EPSILON 1e-6
#define INF 1e6

namespace USTC_CG{

    void IDWWarper::initial(){
        ini = true;
        sum = 0;
        weight.clear();
        if(selected_point.size() == 1){
            Matrix X = {1, 0 ,1 ,0};
            T.push_back(X);
            return;
        }
        for(int i = 0; i < selected_point.size(); i++){
            Matrix A, B;
            sigma.clear();
            calculate_sigma(selected_point[i]);
            for(int j = 0; j < selected_point.size(); j++){
                if(i == j)
                    continue;
                double d_px = selected_point[j].start_x - selected_point[i].start_x;
                double d_py = selected_point[j].start_y - selected_point[i].start_y;
                double d_qx = selected_point[j].end_x - selected_point[i].end_x;
                double d_qy = selected_point[j].end_y - selected_point[i].end_y;
                A.x11 += sigma[j] * (pow(d_px, 2));
                A.x12 += sigma[j] * d_px * d_py;
                A.x21 += sigma[j] * d_px * d_py;
                A.x22 += sigma[j] * (pow(d_py, 2));
                B.x11 += sigma[j] * d_qx * d_px;
                B.x12 += sigma[j] * d_qx * d_py;
                B.x21 += sigma[j] * d_qy * d_px;
                B.x22 += sigma[j] * d_qy * d_py;
            }
            Eigen::Matrix2d MA, MB;
            MA<<A.x11, A.x12, 
            A.x21, A.x22;
            MB<<B.x11, B.x12, 
            B.x21, B.x22;
            Eigen::Matrix2d X = MA.transpose().householderQr().solve(MB.transpose()).transpose();
            std::vector<double> T_i(X.data(), X.data() + X.size());
            Matrix MX = {X(0,0), X(0, 1), X(1,0), X(1,1)};
            T.push_back(MX);
        }

    }

    void IDWWarper::calculate_weight(point &q){

        double weight_i;
        for(auto p : selected_point){
            {
                weight_i = calculate_two_point_sigma(p, q)/sum;
                weight.push_back(weight_i);
            }
        }
    }

    void IDWWarper::warp(){
        if(!ini)
            IDWWarper::initial(); // initial, calculate T_i

        sum = 0;
        sigma.clear();
        weight.clear();
        current_point.end_x = current_point.end_y = 0;
        calculate_sigma(current_point);
        for(auto sigma_i : sigma){
            sum += sigma_i;
        }
        calculate_weight(current_point);

        for(int i = 0; i < selected_point.size(); i++){
            current_point.end_x += weight[i] * (selected_point[i].end_x + 1*(current_point.start_x - selected_point[i].start_x) +0*(current_point.start_y - selected_point[i].start_y));
            current_point.end_y += weight[i] * (selected_point[i].end_y + 0*(current_point.start_x - selected_point[i].start_x) +1*(current_point.start_y - selected_point[i].start_y));
        }
        if(fabs(current_point.end_x-EPSILON) + fabs(current_point.end_y - EPSILON) <= 5 * EPSILON){
            current_point.end_x = current_point.start_x;
            current_point.end_y = current_point.start_y;
        }

    }

    void IDWWarper::calculate_sigma(point &p){
        for(auto pi : selected_point){
            sigma.push_back(calculate_two_point_sigma(pi, p));
        }
    }

    double IDWWarper::calculate_two_point_sigma(point &p, point &q){
        double distance = sqrt(std::pow((p.start_x-q.start_x),2) + std::pow((p.start_y - q.start_y),2));
            if (distance <= EPSILON)
                return INF;
            else
                return pow(distance, -1*mu);
    }
}