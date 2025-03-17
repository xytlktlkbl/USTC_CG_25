#include "Seamless_clone.h"
#include <Eigen/Sparse>

namespace USTC_CG{
    SeamlessClone::SeamlessClone(std::shared_ptr<Image> src, std::shared_ptr<Image>tar, std::shared_ptr<Image> selected_mask, 
    int mouse_x, int mouse_y, int src_x, int src_y){
        src_img_ = src;
        tar_img_ = tar;
        src_selected_mask_ = selected_mask;
        mouse_x_ = mouse_x;
        mouse_y_ = mouse_y;
        src_image_x_ = src_x;
        src_image_y_ = src_y;
    }
    void SeamlessClone::solve(){
        int width = src_selected_mask_->width();
        int height = src_selected_mask_->height();
        int width_start = -1;
        int height_start = -1;
        int width_selected = -1;
        int height_selected = -1;
        for(int x = 0; x < width; ++x){
            for(int y = 0; y < height; ++y){
                if(src_selected_mask_->get_pixel(x,y)[0] > 0){
                    width_start = x;
                    height_start = y;
                    break;
                }
            }
            if(width_start != -1)
                break;
        }
        if(width_start == -1)
            return;
        for(int x = width_start; x < width; ++x){
            if(src_selected_mask_->get_pixel(x, height_start)[0] <= 0 && mouse_x_ + x - width_start < tar_img_->width() -1){
                width_selected = x - width_start;
                break;
            }
        }
        for(int y = height_start; y < height; ++y){
            if(src_selected_mask_->get_pixel(width_start, y)[0] <= 0 && mouse_y_ + y - height_start < tar_img_->height() -1){
                height_selected = y - height_start;
                break;
            }
        }
        if(width_selected == -1)
            width_selected = width - width_start;
        if(height_selected == -1)
            height_selected = height - height_start;
        int size = src_img_->channels();
        Eigen::SparseMatrix<double> A(height_selected * width_selected, height_selected * width_selected);
        Eigen::MatrixXd B(height_selected * width_selected, size);
        std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
        for (int x = width_start; x < width_start + width_selected; ++x)
        {
            for (int y = height_start; y < height_start + height_selected; ++y)
            {
                int idx = (y - height_start) * width_selected + x - width_start;
                int tar_x = static_cast<int>(mouse_x_) + x - width_start;
                int tar_y = static_cast<int>(mouse_y_) + y - height_start;
                int src_x = x;
                int src_y = y;
                if(x != width_start && x != width_start + width_selected -1 && y != height_start && y != height_start + height_selected - 1)
                {
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx - width_selected, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx + width_selected, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx- 1, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx + 1, -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x + 1, src_y)[i]
                        - src_img_->get_pixel(src_x - 1, src_y)[i];
                    }
                }
                //处理边
                else if(x == width_start && y != height_start && y != height_start + height_selected - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y + 1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x-1, tar_y)[i];   
                    }
                }
                else if(x == width_start + width_selected -1 && y != height_start && y != height_start + height_selected - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x + 1, tar_y)[i];   
                    }
                }
                else if(x != width_start && x != width_start + width_selected -1 && y == height_start){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x, tar_y - 1)[i];   
                    }
                }
                else if(x != width_start && x != width_start + width_selected -1 && y == height_start + height_selected - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x, tar_y + 1)[i];   
                    }
                }
                //处理顶点
                else if(x == width_start && y == height_start ){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                    std::vector<unsigned char> aq = tar_img_->get_pixel(tar_x - 1, tar_y);
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x, tar_y - 1)[i] + tar_img_->get_pixel(tar_x - 1, tar_y)[i];   
                    }
                }
                else if(x == width_start + width_selected -1 && y == height_start ){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x, tar_y - 1)[i] + tar_img_->get_pixel(tar_x + 1, tar_y)[i];  
                    } 
                }
                else if(x == width_start && y == height_start + height_selected - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x, tar_y + 1)[i] + tar_img_->get_pixel(tar_x - 1, tar_y)[i];
                    }
                }
                else if(x == width_start + width_selected -1 && y == height_start + height_selected - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                    for(int i = 0 ; i < size; i++){
                        B(idx, i) = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
                        - src_img_->get_pixel(src_x, src_y-1)[i] 
                        - src_img_->get_pixel(src_x, src_y+1)[i] 
                        - src_img_->get_pixel(src_x+1, src_y)[i] 
                        - src_img_->get_pixel(src_x-1, src_y)[i]
                        + tar_img_->get_pixel(tar_x, tar_y + 1)[i] + tar_img_->get_pixel(tar_x + 1, tar_y)[i];   
                    }
                }
            }
        }
        A.setFromTriplets(triplet_list.begin(), triplet_list.end());
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> solver;
        solver.setMaxIterations(5000); // 可选：设置最大迭代次数
        solver.setTolerance(1e-7);    // 可选：设置收敛容限
        solver.compute(A); // 分解矩阵 A
        Eigen::MatrixXd r = solver.solve(B); // 求解方程组 Ar = B
        for(int x = width_start; x < width_start + width_selected; ++x){
            for(int y = height_start; y < height_start + height_selected; ++y){
                int tar_x = static_cast<int>(mouse_x_) + x - width_start;
                int tar_y = static_cast<int>(mouse_y_) + y - height_start;
            if (0 <= tar_x && tar_x < tar_img_->width() && 0 <= tar_y && tar_y < tar_img_->height() && src_selected_mask_->get_pixel(x, y)[0] > 0)
            {
                std::vector<unsigned char> pixel;
                int index  =(y-height_start) * width_selected + x - width_start;
                for(int i = 0; i < size; i++){
                    if( r(index, i) >= 0 &&  r(index, i) <= 255){
                        pixel.push_back( r(index, i));
                    }
                    else if( r(index, i) < 0){
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

double SeamlessClone::get_gradient(int src_x, int src_y, int i, int tar_x, int tar_y){
    double gradient_g = 4.0 * src_img_->get_pixel(src_x, src_y)[i] 
    - src_img_->get_pixel(src_x, src_y-1)[i] 
    - src_img_->get_pixel(src_x, src_y+1)[i] 
    - src_img_->get_pixel(src_x + 1, src_y)[i]
    - src_img_->get_pixel(src_x - 1, src_y)[i];
    double gradient_f = 4.0 * tar_img_->get_pixel(tar_x, tar_y)[i] 
    - tar_img_->get_pixel(tar_x, tar_y-1)[i] 
    - tar_img_->get_pixel(tar_x, tar_y+1)[i] 
    - tar_img_->get_pixel(tar_x + 1, tar_y)[i]
    - tar_img_->get_pixel(tar_x - 1, tar_y)[i];
    if(fabs(gradient_f) > fabs(gradient_g))
    {
        printf("%d %d:%d\n", src_x, src_y, int(gradient_f));
        return gradient_f;
    }
    printf("%d %d:%d\n", src_x, src_y, int(gradient_g));
    return gradient_g;
}


void SeamlessClone::solve_mix_gradient(){
    int width = src_selected_mask_->width();
    int height = src_selected_mask_->height();
    int width_start = -1;
    int height_start = -1;
    int width_selected = -1;
    int height_selected = -1;
    for(int x = 0; x < width; ++x){
        for(int y = 0; y < height; ++y){
            if(src_selected_mask_->get_pixel(x,y)[0] > 0){
                width_start = x;
                height_start = y;
                break;
            }
        }
        if(width_start != -1)
            break;
    }
    if(width_start == -1)
        return;
    for(int x = width_start; x < width; ++x){
        if(src_selected_mask_->get_pixel(x, height_start)[0] <= 0){
            width_selected = x - width_start;
            break;
        }
    }
    for(int y = height_start; y < height; ++y){
        if(src_selected_mask_->get_pixel(width_start, y)[0] <= 0){
            height_selected = y - height_start;
            break;
        }
    }
    if(width_selected == -1)
        width_selected = width - width_start;
    if(height_selected == -1)
        height_selected = height - height_start;
    int size = src_img_->channels();
    Eigen::SparseMatrix<double> A(height_selected * width_selected, height_selected * width_selected);
    Eigen::MatrixXd B(height_selected * width_selected, size);
    std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
    for (int x = width_start; x < width_start + width_selected; ++x)
    {
        for (int y = height_start; y < height_start + height_selected; ++y)
        {
            int idx = (y - height_start) * width_selected + x - width_start;
            int tar_x = static_cast<int>(mouse_x_) + x - width_start;
            int tar_y = static_cast<int>(mouse_y_) + y - height_start;
            int src_x = x;
            int src_y = y;
            if(x != width_start && x != width_start + width_selected -1 && y != height_start && y != height_start + height_selected - 1)
            {
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx - width_selected, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx + width_selected, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx- 1, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx + 1, -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y);
                }
            }
            //处理边
            else if(x == width_start && y != height_start && y != height_start + height_selected - 1){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y) 
                    + tar_img_->get_pixel(tar_x-1, tar_y)[i];   
                }
            }
            else if(x == width_start + width_selected -1 && y != height_start && y != height_start + height_selected - 1){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y)
                    + tar_img_->get_pixel(tar_x + 1, tar_y)[i];   
                }
            }
            else if(x != width_start && x != width_start + width_selected -1 && y == height_start){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y)
                    + tar_img_->get_pixel(tar_x, tar_y - 1)[i];   
                }
            }
            else if(x != width_start && x != width_start + width_selected -1 && y == height_start + height_selected - 1){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y)
                    + tar_img_->get_pixel(tar_x, tar_y + 1)[i];   
                }
            }
            //处理顶点
            else if(x == width_start && y == height_start ){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                std::vector<unsigned char> aq = tar_img_->get_pixel(tar_x - 1, tar_y);
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y)
                    + tar_img_->get_pixel(tar_x, tar_y - 1)[i] + tar_img_->get_pixel(tar_x - 1, tar_y)[i];   
                }
            }
            else if(x == width_start + width_selected -1 && y == height_start ){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y)
                    + tar_img_->get_pixel(tar_x, tar_y - 1)[i] + tar_img_->get_pixel(tar_x + 1, tar_y)[i];  
                } 
            }
            else if(x == width_start && y == height_start + height_selected - 1){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x + 1 - width_start), -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y)
                    + tar_img_->get_pixel(tar_x, tar_y + 1)[i] + tar_img_->get_pixel(tar_x - 1, tar_y)[i];
                }
            }
            else if(x == width_start + width_selected -1 && y == height_start + height_selected - 1){
                triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1 - height_start) * width_selected + x - width_start, -1.0));
                triplet_list.push_back(Eigen::Triplet<double>(idx, (y - height_start) * width_selected + (x - 1 - width_start), -1.0));
                for(int i = 0 ; i < size; i++){
                    B(idx, i) = get_gradient(src_x, src_y, i, tar_x, tar_y)
                    + tar_img_->get_pixel(tar_x, tar_y + 1)[i] + tar_img_->get_pixel(tar_x + 1, tar_y)[i];   
                }
            }
        }
    }
    A.setFromTriplets(triplet_list.begin(), triplet_list.end());
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> solver;
    solver.setMaxIterations(5000);
    solver.setTolerance(1e-7);
    solver.compute(A); // 分解矩阵 A
    Eigen::MatrixXd r = solver.solve(B); // 求解方程组 Ar = B
    for(int i = 0; i < width_selected * height_selected; i++){
        for(int j = 0; j < size; j ++)
        printf("%d, %d:  %d\n", i, j, int(r(i,j)));
    }
    for(int x = width_start; x < width_start + width_selected; ++x){
        for(int y = height_start; y < height_start + height_selected; ++y){
            int tar_x = static_cast<int>(mouse_x_) + x - width_start;
            int tar_y = static_cast<int>(mouse_y_) + y - height_start;
        if (0 <= tar_x && tar_x < tar_img_->width() && 0 <= tar_y && tar_y < tar_img_->height() && src_selected_mask_->get_pixel(x, y)[0] > 0)
        {
            std::vector<unsigned char> pixel;
            int index  =(y-height_start) * width_selected + x - width_start;
            for(int i = 0; i < size; i++){
                if( r(index, i) >= 0 &&  r(index, i) <= 255){
                    pixel.push_back( r(index, i));
                }
                else if( r(index, i) < 0){
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
}