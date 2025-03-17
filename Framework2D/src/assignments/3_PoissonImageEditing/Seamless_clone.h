#pragma once

#include "common/image_widget.h"
namespace USTC_CG{

class SeamlessClone
{
    public:
    SeamlessClone(std::shared_ptr<Image>, std::shared_ptr<Image>, std::shared_ptr<Image>, 
    int, int, int, int);
    void solve(); // 给外部调用的接口，求解 Poisson 方程组，返回一个 Seamless Clone 的结果图像（和背景图像一样大，替换了选中区域）        //...
    void solve_mix_gradient(); // 和solve類似，但是B使用混合梯度
    private:
    double get_gradient(int x, int y, int i, int, int);
    // 注意使用指针，避免额外的复制操作
    std::shared_ptr<Image> src_img_; // 源图像
    std::shared_ptr<Image> tar_img_; // 背景图像
    std::shared_ptr<Image> src_selected_mask_; // 选中区域（矩形情形可以无视）
    int offset_x_, offset_y_;        // 矩形区域在背景图像中的位置（例如，左上角的坐标）
    int mouse_x_, mouse_y_; // 鼠标的位置
    int src_image_x_, src_image_y_; // 原图形的位置
        //...
};

}
