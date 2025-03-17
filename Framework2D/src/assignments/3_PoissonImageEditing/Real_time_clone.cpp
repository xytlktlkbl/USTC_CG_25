#include "Real_time_clone.h"
#include <Eigen/Sparse>

namespace USTC_CG{
    RealtimeClone::RealtimeClone(){}
    void RealtimeClone::set(std::shared_ptr<Image> src, std::shared_ptr<Image>tar, std::shared_ptr<Image> selected_mask, 
    int mouse_x, int mouse_y, int src_x, int src_y){
        src_img_ = src;
        tar_img_ = tar;
        src_selected_mask_ = selected_mask;
        mouse_x_ = mouse_x;
        mouse_y_ = mouse_y;
        src_image_x_ = src_x;
        src_image_y_ = src_y;
        width = src_selected_mask_->width();
        height = src_selected_mask_->height();
        size = src_img_->channels();
        for(int x = 0; x < width; ++x){
            for(int y = 0; y < height; ++y){
                if(is_selected(x, y)){
                    selected_point.push_back(std::make_pair(x, y));
                    if(x < width_start)
                        width_start = x;
                    if(y < height_start)
                        height_start = y;
                }
            }
        }
        size_point = selected_point.size();
        A.resize(size_point, size_point);
        std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
        for(int i = 0; i < size_point; ++i){
            triplet_list.push_back(Eigen::Triplet<double>(i, i, 4.0));
            auto it = find(selected_point.begin(), selected_point.end(), std::make_pair(selected_point[i].first - 1, selected_point[i].second));
            if(it != selected_point.end()){
                triplet_list.push_back(Eigen::Triplet<double>(i, it-selected_point.begin(), -1.0));
            }
            it = find(selected_point.begin(), selected_point.end(), std::make_pair(selected_point[i].first + 1, selected_point[i].second));
            if(it != selected_point.end()){
                triplet_list.push_back(Eigen::Triplet<double>(i, it-selected_point.begin(), -1.0));
            }
            it = find(selected_point.begin(), selected_point.end(), std::make_pair(selected_point[i].first, selected_point[i].second - 1));
            if(it != selected_point.end()){
                triplet_list.push_back(Eigen::Triplet<double>(i, it-selected_point.begin(), -1.0));
            }
            it = find(selected_point.begin(), selected_point.end(), std::make_pair(selected_point[i].first, selected_point[i].second + 1));
            if(it != selected_point.end()){
                triplet_list.push_back(Eigen::Triplet<double>(i, it-selected_point.begin(), -1.0));
            }
        }
        A.setFromTriplets(triplet_list.begin(), triplet_list.end());
        solver.compute(A); // 分解矩阵 A
        is_initial = true;
    }

    bool RealtimeClone::is_selected(int x, int y){
        if(src_selected_mask_->get_pixel(x, y)[0] > 0 && x < tar_img_->width() && y < tar_img_->height())
            return true;
        return false;
    }

    void RealtimeClone::solve(int mouse_x, int mouse_y){
        mouse_x_ = mouse_x;
        mouse_y_ = mouse_y;
        Eigen::MatrixXd B(size_point, size);
            for (int p = 0 ; p < size_point; ++p)
            {
                std::vector<double> bVal(size);
                int tar_x = mouse_x_ + selected_point[p].first - width_start;
                int tar_y = mouse_y_ + selected_point[p].second - height_start;
                    for(int i = 0 ; i < size; i++){
                        bVal[i] = 4.0 * src_img_->get_pixel(selected_point[p].first, selected_point[p].second)[i] 
                        - src_img_->get_pixel(selected_point[p].first, selected_point[p].second - 1)[i] 
                        - src_img_->get_pixel(selected_point[p].first, selected_point[p].second + 1)[i] 
                        - src_img_->get_pixel(selected_point[p].first + 1, selected_point[p].second)[i]
                        - src_img_->get_pixel(selected_point[p].first - 1, selected_point[p].second)[i];
                    }
                //处理邻居
                if(!is_selected(selected_point[p].first, selected_point[p].second - 1)){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x, tar_y-1)[i];   
                    }
                }

                if(!is_selected(selected_point[p].first, selected_point[p].second + 1)){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x, tar_y + 1)[i];   
                    }
                }

                if(!is_selected(selected_point[p].first - 1, selected_point[p].second)){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x, tar_y + 1)[i];   
                    }
                }

                if(!is_selected(selected_point[p].first + 1, selected_point[p].second) ){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x + 1, tar_y )[i];   
                    }
                }

                for(int i = 0; i < size; ++i){
                    B(p, i) = bVal[i];
                }
            }
        Eigen::MatrixXd r = solver.solve(B); // 求解方程组 Ar = B
        for(int p = 0; p < size_point; p++){
            int tar_x = mouse_x_ + selected_point[p].first - width_start;
            int tar_y = mouse_y_ + selected_point[p].second - height_start;
            if(0 <= tar_x && tar_x < tar_img_->width() && 0 <= tar_y && tar_y < tar_img_->height()){
                std::vector<unsigned char> pixel;
                for(int i = 0; i < size; i++){
                    if( r(p, i) >= 0 &&  r(p, i) <= 255){
                        pixel.push_back( r(p, i));
                    }
                    else if( r(p, i) < 0){
                        pixel.push_back(0);
                    }
                    else{
                        pixel.push_back(255);
                    }
                }
                tar_img_->set_pixel(tar_x, tar_y, pixel);
            }
        }
 
        
    }

    std::vector<double> RealtimeClone::get_gradient(int src_x, int src_y){
        int tar_x = mouse_x_ + src_x - width_start;
        int tar_y = mouse_y_ + src_y - height_start;
        std::vector<double> mix_gradient;
        for(int i = 0; i < size; i++){
            double gradient_g = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
            - src_img_->get_pixel(src_x, src_y - 1)[i] 
            - src_img_->get_pixel(src_x, src_y + 1)[i] 
            - src_img_->get_pixel(src_x + 1, src_y)[i]
            - src_img_->get_pixel(src_x - 1, src_y)[i];
            double gradient_f = 4.0 * tar_img_->get_pixel(tar_x, tar_y)[i] 
            - tar_img_->get_pixel(tar_x, tar_y - 1)[i] 
            - tar_img_->get_pixel(tar_x, tar_y + 1)[i] 
            - tar_img_->get_pixel(tar_x + 1, tar_y)[i]
            - tar_img_->get_pixel(tar_x - 1, tar_y)[i];
            if(fabs(gradient_f) > fabs(gradient_g))
                mix_gradient.push_back(gradient_f);
            else
                mix_gradient.push_back(gradient_g);
        }
        return mix_gradient;
}

    void RealtimeClone::solve_mix_gradient(int mouse_x, int mouse_y){
        mouse_x_ = mouse_x;
        mouse_y_ = mouse_y;
        Eigen::MatrixXd B(size_point, size);
            for (int p = 0 ; p < size_point; ++p)
            {
                std::vector<double> bVal;
                int tar_x = mouse_x_ + selected_point[p].first - width_start;
                int tar_y = mouse_y_ + selected_point[p].second - height_start;
                bVal = get_gradient(selected_point[p].first, selected_point[p].second);
                //处理邻居
                if(!is_selected(selected_point[p].first, selected_point[p].second - 1) ){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x, tar_y-1)[i];   
                    }
                }

                if(!is_selected(selected_point[p].first, selected_point[p].second + 1) ){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x, tar_y + 1)[i];   
                    }
                }

                if(!is_selected(selected_point[p].first - 1, selected_point[p].second)){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x, tar_y + 1)[i];   
                    }
                }

                if(!is_selected(selected_point[p].first + 1, selected_point[p].second) ){
                    for(int i = 0 ; i < size; i++){
                        bVal[i] += tar_img_->get_pixel(tar_x + 1, tar_y )[i];   
                    }
                }
                for(int i = 0; i < size; ++i){
                    B(p, i) = bVal[i];
                }
            }
        Eigen::MatrixXd r = solver.solve(B); // 求解方程组 Ar = B

        for(int p = 0; p < size_point; p++){
            int tar_x = mouse_x_ + selected_point[p].first - width_start;
            int tar_y = mouse_y_ + selected_point[p].second - height_start;
            if(0 <= tar_x && tar_x < tar_img_->width() && 0 <= tar_y && tar_y < tar_img_->height()){
                std::vector<unsigned char> pixel;
                for(int i = 0; i < size; i++){
                    if( r(p, i) >= 0 &&  r(p, i) <= 255){
                        pixel.push_back( r(p, i));
                    }
                    else if( r(p, i) < 0){
                        pixel.push_back(0);
                    }
                    else{
                        pixel.push_back(255);
                    }
                }
                tar_img_->set_pixel(tar_x, tar_y, pixel);
            }
        }
 
        
    }

}
