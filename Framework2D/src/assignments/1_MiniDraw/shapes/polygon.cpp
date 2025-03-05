#include "polygon.h"

#include <imgui.h>

namespace USTC_CG
{
// Draw the line using ImGui

void Polygon::draw(const Config& config) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    size_t size = x_list_.size();
    for(size_t i = 0; i < size - 1; i++){
        draw_line(x_list_[i], y_list_[i], x_list_[i+1], y_list_[i+1], config);
    }
    if(is_closed)
        draw_line(x_list_[0], y_list_[0], x_list_[size-1], y_list_[size -1], config);
}

void Polygon::close_polygon(){
    is_closed = true;
}

void Polygon::update(float x, float y)
{
    x_list_.pop_back();
    x_list_.push_back(x);
    y_list_.pop_back(); 
    y_list_.push_back(y);
}

void Polygon::draw_line(float start_point_x, float start_point_y, float end_point_x, float end_point_y, const Config& config) const{
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddLine(
        ImVec2(config.bias[0] + start_point_x, config.bias[1] + start_point_y),
        ImVec2(config.bias[0] + end_point_x, config.bias[1] + end_point_y),
        IM_COL32(config.line_color[0], config.line_color[1], config.line_color[2], config.line_color[3]), config.line_thickness);
    }

void Polygon::add_control_point(float x, float y ){
    Config config;
    x_list_.push_back(x);
    y_list_.push_back(y);
    size_t size = x_list_.size();
    draw_line(x_list_[size-2], y_list_[size-2], x_list_[size-1], y_list_[size-1], config);
}
}  // namespace USTC_CG
