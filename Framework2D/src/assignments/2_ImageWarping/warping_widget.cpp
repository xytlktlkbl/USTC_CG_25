#include "warping_widget.h"
#include "warper/warper.h"
#include "warper/IDW_warper.h"
#include "warper/RBF_warper.h"
#include "warper/Dlib_warper.h"

#include <cmath>
#include <iostream>

namespace USTC_CG
{
using uchar = unsigned char;

WarpingWidget::WarpingWidget(const std::string& label, const std::string& filename)
    : ImageWidget(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void WarpingWidget::draw()
{
    // Draw the image
    ImageWidget::draw();
    // Draw the canvas
    if (flag_enable_selecting_points_)
        select_points();
}

void WarpingWidget::invert()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            data_->set_pixel(
                i,
                j,
                { static_cast<uchar>(255 - color[0]),
                  static_cast<uchar>(255 - color[1]),
                  static_cast<uchar>(255 - color[2]) });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}
void WarpingWidget::mirror(bool is_horizontal, bool is_vertical)
{
    Image image_tmp(*data_);
    int width = data_->width();
    int height = data_->height();

    if (is_horizontal)
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i,
                        j,
                        image_tmp.get_pixel(width - 1 - i, height - 1 - j));
                }
            }
        }
        else
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(width - 1 - i, j));
                }
            }
        }
    }
    else
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(i, height - 1 - j));
                }
            }
        }
    }

    // After change the image, we should reload the image data to the renderer
    update();
}
void WarpingWidget::gray_scale()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            uchar gray_value = (color[0] + color[1] + color[2]) / 3;
            data_->set_pixel(i, j, { gray_value, gray_value, gray_value });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}
void WarpingWidget::warping()
{
    // HW2_TODO: You should implement your own warping function that interpolate
    // the selected points.
    // Please design a class for such warping operations, utilizing the
    // encapsulation, inheritance, and polymorphism features of C++. 

    // Create a new image to store the result
    Image warped_image(*data_);
    // Initialize the color of result image
    for (int y = 0; y < data_->height(); ++y)
    {
        for (int x = 0; x < data_->width(); ++x)
        {
            warped_image.set_pixel(x, y, { 0, 0, 0 });
        }
    }

    switch (warping_type_)
    {
        case kDefault: break;
        case kFisheye:
        {
            // Example: (simplified) "fish-eye" warping
            // For each (x, y) from the input image, the "fish-eye" warping
            // transfer it to (x', y') in the new image: Note: For this
            // transformation ("fish-eye" warping), one can also calculate the
            // inverse (x', y') -> (x, y) to fill in the "gaps".
            for (int y = 0; y < data_->height(); ++y)
            {
                for (int x = 0; x < data_->width(); ++x)
                {
                    // Apply warping function to (x, y), and we can get (x', y')
                    auto [new_x, new_y] =
                        fisheye_warping(x, y, data_->width(), data_->height());
                    // Copy the color from the original image to the result
                    // image
                    if (new_x >= 0 && new_x < data_->width() && new_y >= 0 &&
                        new_y < data_->height())
                    {
                        std::vector<unsigned char> pixel =
                            data_->get_pixel(x, y);
                        warped_image.set_pixel(new_x, new_y, pixel);
                    }
                }
            }
            break;
        }
        case kIDW:
        {
            // HW2_TODO: Implement the IDW warping
            // use selected points start_points_, end_points_ to construct the map
            int result_x, result_y, size = data_->get_pixel(0, 0).size();
            IDWWarper idw(data_->width(), data_->height());
            for(int i = 0; i < start_points_.size(); i++){
                idw.add_point(start_points_[i].x, start_points_[i].y, end_points_[i].x, end_points_[i].y);
            }
            for (int y = 0; y < data_->height(); ++y){
                for(int x = 0; x < data_->width(); ++x){
                    idw.current_point.start_x = x, idw.current_point.start_y = y;
                    idw.warp();
                    result_x = idw.current_point.end_x, result_y = idw.current_point.end_y;
                    if(result_x >= 0 &&result_x < data_->width() && result_y >= 0 
                        && result_y < data_->height()){
                            std::vector<unsigned char> pixel = data_->get_pixel(x, y);
                            warped_image.set_pixel(result_x, result_y, pixel);
                            idw.add_result(result_x, result_y);
                        }
                }
            }
            for (int y = 0; y < data_->height(); ++y){
                for(int x = 0; x < data_->width(); ++x){
                    if(!(*idw.flag)[x][y]){
                        std::vector<unsigned char> pixel(size);
                        idw.fill_blank(x,y);
                        float* v = new float[2];
                        for(int i = 0; i < idw.neibour_number; i++){
                            idw.index.get_item(idw.closed_item[i], v);
                            for(int j = 0; j < size; j++){
                                pixel[j] += data_->get_pixel(v[0], v[1])[j] / idw.neibour_number;
                            }
                        }
                        warped_image.set_pixel(x, y, pixel);
                    }
                }
            }
            break;
        }
        case kRBF:
        {
            // HW2_TODO: Implement the RBF warping
            // use selected points start_points_, end_points_ to construct the map
            int result_x, result_y, size = data_->get_pixel(0, 0).size();
            RBFWarper rbf(data_->width(), data_->height());
            for(int i = 0; i < start_points_.size(); i++){
                rbf.add_point(start_points_[i].x, start_points_[i].y, end_points_[i].x, end_points_[i].y);
            }
            for (int y = 0; y < data_->height(); ++y){
                for(int x = 0; x < data_->width(); ++x){
                    rbf.current_point.start_x = x, rbf.current_point.start_y = y;
                    rbf.warp();
                    result_x = rbf.current_point.end_x, result_y = rbf.current_point.end_y;
                    if(result_x >= 0 &&result_x < data_->width() && result_y >= 0 
                        && result_y < data_->height()){
                            std::vector<unsigned char> pixel = data_->get_pixel(x, y);
                            warped_image.set_pixel(result_x, result_y, pixel);
                            rbf.add_result(result_x, result_y);
                        }
            }
            }
            for (int y = 0; y < data_->height(); ++y){
                for(int x = 0; x < data_->width(); ++x){
                    if(!(*rbf.flag)[x][y]){
                        std::vector<unsigned char> pixel(size);
                        rbf.fill_blank(x,y);
                        float* v = new float[2];
                        for(int i = 0; i < rbf.neibour_number; i++){
                            rbf.index.get_item(rbf.closed_item[i], v);
                            for(int j = 0; j < size; j++){
                                pixel[j] += data_->get_pixel(v[0], v[1])[j] / rbf.neibour_number;
                            }
                        }
                        warped_image.set_pixel(x, y, pixel);
                    }
                }
            }
            break;
        }
        case kDlib:
        {
            int result_x, result_y, size = data_->get_pixel(0, 0).size();
            DlibWarper dlib(data_->width(), data_->height());
            for(int i = 0; i < start_points_.size(); i++){
                dlib.add_point(start_points_[i].x, start_points_[i].y, end_points_[i].x, end_points_[i].y);
            }
            for (int y = 0; y < data_->height(); ++y){
                for(int x = 0; x < data_->width(); ++x){
                    dlib.current_point.start_x = x, dlib.current_point.start_y = y;
                    dlib.warp();
                    result_x = dlib.current_point.end_x, result_y = dlib.current_point.end_y;
                    if(result_x >= 0 &&result_x < data_->width() && result_y >= 0 
                        && result_y < data_->height()){
                            std::vector<unsigned char> pixel = data_->get_pixel(x, y);
                            warped_image.set_pixel(result_x, result_y, pixel);
                            dlib.add_result(result_x, result_y);
                        }
            }
        }
        for (int y = 0; y < data_->height(); ++y){
            for(int x = 0; x < data_->width(); ++x){
                if(!(*dlib.flag)[x][y]){
                    std::vector<unsigned char> pixel(size);
                    dlib.fill_blank(x,y);
                    float* v = new float[2];
                    for(int i = 0; i < dlib.neibour_number; i++){
                        dlib.index.get_item(dlib.closed_item[i], v);
                        for(int j = 0; j < size; j++){
                            pixel[j] += data_->get_pixel(v[0], v[1])[j] / dlib.neibour_number;
                        }
                    }
                    warped_image.set_pixel(x, y, pixel);
                }
            }
        }
            break;
        }
        
        default: break;
    }

    *data_ = std::move(warped_image);
    update();
}
void WarpingWidget::restore()
{
    *data_ = *back_up_;
    update();
}
void WarpingWidget::set_default()
{
    warping_type_ = kDefault;
}
void WarpingWidget::set_fisheye()
{
    warping_type_ = kFisheye;
}
void WarpingWidget::set_IDW()
{
    warping_type_ = kIDW;
}
void WarpingWidget::set_RBF()
{
    warping_type_ = kRBF;
}
void WarpingWidget::set_Dlib()
{
    warping_type_ = kDlib;
}
void WarpingWidget::enable_selecting(bool flag)
{
    flag_enable_selecting_points_ = flag;
}
void WarpingWidget::select_points()
{
    /// Invisible button over the canvas to capture mouse interactions.
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    // Record the current status of the invisible button
    bool is_hovered_ = ImGui::IsItemHovered();
    // Selections
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        draw_status_ = true;
        start_ = end_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
    }
    if (draw_status_)
    {
        end_ = ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            start_points_.push_back(start_);
            end_points_.push_back(end_);
            draw_status_ = false;
        }
    }
    // Visualization
    auto draw_list = ImGui::GetWindowDrawList();
    for (size_t i = 0; i < start_points_.size(); ++i)
    {
        ImVec2 s(
            start_points_[i].x + position_.x, start_points_[i].y + position_.y);
        ImVec2 e(
            end_points_[i].x + position_.x, end_points_[i].y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
        draw_list->AddCircleFilled(e, 4.0f, IM_COL32(0, 255, 0, 255));
    }
    if (draw_status_)
    {
        ImVec2 s(start_.x + position_.x, start_.y + position_.y);
        ImVec2 e(end_.x + position_.x, end_.y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
    }
}
void WarpingWidget::init_selections()
{
    start_points_.clear();
    end_points_.clear();
}

std::pair<int, int>
WarpingWidget::fisheye_warping(int x, int y, int width, int height)
{
    float center_x = width / 2.0f;
    float center_y = height / 2.0f;
    float dx = x - center_x;
    float dy = y - center_y;
    float distance = std::sqrt(dx * dx + dy * dy);

    // Simple non-linear transformation r -> r' = f(r)
    float new_distance = std::sqrt(distance) * 10;

    if (distance == 0)
    {
        return { static_cast<int>(center_x), static_cast<int>(center_y) };
    }
    // (x', y')
    float ratio = new_distance / distance;
    int new_x = static_cast<int>(center_x + dx * ratio);
    int new_y = static_cast<int>(center_y + dy * ratio);

    return { new_x, new_y };
}
}  // namespace USTC_CG