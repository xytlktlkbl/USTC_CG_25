#include "Seamless_clone.h"
#include <Eigen/Sparse>

namespace USTC_CG{
    SeamlessClone::SeamlessClone(std::shared_ptr<Image> src, std::shared_ptr<Image>tar, std::shared_ptr<Image> selected_mask, 
    int offset_x, int offset_y, int mouse_x, int mouse_y, int src_x, int src_y){
        src_img_ = src;
        tar_img_ = tar;
        src_selected_mask_ = selected_mask;
        offset_x_ = offset_x;
        offset_y_ = offset_y;
        mouse_x_ = mouse_x;
        mouse_y_ = mouse_y;
        src_image_x_ = src_x;
        src_image_y_ = src_y;
    }
    void SeamlessClone::solve(){
        int width = src_selected_mask_->width();
        int height = src_selected_mask_->height();
        unsigned char size = src_img_->channels();
        Eigen::SparseMatrix<double> A(width * height, width * height);
        Eigen::MatrixXf B(width * height, size);
        std::vector<Eigen::Triplet<double>> triplet_list; // 使用三元组 vector 来存储非零元素
        for (int x = 0; x < width; ++x)
        {
            for (int y = 0; y < height; ++y)
            {
                int idx = y * width + x;
                if(x != 0 && x != width-1 && y != 0 && y != height - 1)
                {
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x - 1), -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x + 1), -1.0));
                    for(int i = 0 ; )
                    B(idx, 0) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0];
                }
                //处理边
                if(x == 0 && y != 0 && y != height - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x + 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x-1, y)[0];   
                }
                if(x == width-1 && y != 0 && y != height - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x - 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x + 1, y)[0];   
                }
                if(y == 0 && x != 0 && x != width - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x + 1), -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x - 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x, y - 1)[0];   
                }
                if(y == height - 1 && x != 0 && x != width - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x + 1), -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x - 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x, y + 1)[0];   
                }
                //处理顶点
                if(y == 0 && x == 0){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x + 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x, y - 1)[0] + tar_img_->get_pixel(x - 1, y)[0];   
                }
                if(y == 0 && x == width-1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y + 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x - 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x, y - 1)[0] + tar_img_->get_pixel(x + 1, y)[0];   
                }
                if(y == height-1 && x == 0){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x + 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x, y + 1)[0] + tar_img_->get_pixel(x - 1, y)[0];   
                }
                if(y == height-1 && x == width - 1){
                    triplet_list.push_back(Eigen::Triplet<double>(idx, idx, 4.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, (y - 1) * width + x, -1.0));
                    triplet_list.push_back(Eigen::Triplet<double>(idx, y * width + (x - 1), -1.0));
                    B(idx) = 4.0 * src_img_->get_pixel(x, y)[0] - src_img_->get_pixel(x, y-1)[0] 
                    - src_img_->get_pixel(x, y+1)[0] - src_img_->get_pixel(x+1, y)[0] - src_img_->get_pixel(x-1, y)[0]
                    + tar_img_->get_pixel(x, y + 1)[0] + tar_img_->get_pixel(x + 1, y)[0];   
                }
            }
        }
        A.setFromTriplets(triplet_list.begin(), triplet_list.end());
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver; // 创建 SimplicialLDLT 求解器
        solver.compute(A); // 分解矩阵 A
        Eigen::VectorXd r = solver.solve(B); // 求解方程组 Ar = B
        int tar_x =
                    static_cast<int>(mouse_x_) + x -
                    static_cast<int>(src_image_x_);
                int tar_y =
                    static_cast<int>(mouse_y_) + y -
                    static_cast<int>(src_image_y_);
                if (0 <= tar_x && tar_x < tar_img_->width() && 0 <= tar_y &&
                    tar_y < tar_img_->height() && src_selected_mask_->get_pixel(x, y)[0] > 0)
                {
                    tar_img_->set_pixel(
                        tar_x,
                        tar_y,
                        src_img_->get_pixel(x, y));
                }
    }

}