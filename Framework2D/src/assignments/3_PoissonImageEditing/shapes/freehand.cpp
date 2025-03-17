#include "freehand.h"

#include <imgui.h>
#include <algorithm>

namespace USTC_CG
{
// Draw the line using ImGui

void Freehand::draw(const Config& config) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    size_t size = x_list_.size();
    for(size_t i = 0; i < size - 1; i++){
        draw_line(x_list_[i], y_list_[i], x_list_[i+1], y_list_[i+1], config);
    }
    if(is_closed)
        draw_line(x_list_[0], y_list_[0], x_list_[size-1], y_list_[size -1], config);
}


void Freehand::update(float x, float y)
{
    x_list_.pop_back();
    x_list_.push_back(x);
    y_list_.pop_back(); 
    y_list_.push_back(y);
}

std::vector<std::pair<int, int>> Freehand::get_interior_pixels()
{
    std::vector<std::pair<int, int>> result;
    std::vector<Edge> AEL;
    Freehand::build_ET_table();
    int current_y = min_y;
    while(current_y <= max_y || AEL.size() != 0 && AEL.size() != 1){
        for(auto & e :ET[current_y]){
            AEL.push_back(e);
        }
        ET.erase(current_y);

        for(auto e =  AEL.begin();e!=AEL.end();){
            if(e->y_max <= current_y){
                e = AEL.erase(e);
            }
            else
                ++e;
        }

        std::sort(AEL.begin(), AEL.end(),
        [](const Edge& a, const Edge& b) { return a.x_min < b.x_min;});

        for(int i = 0; i < AEL.size(); i+= 2){
            if(AEL.size() == 1)
                break;
            int x_start = AEL[i].x_min;
            int x_end = AEL[i+1].x_min;
            for(int j = std::min(x_start, x_end); j <= std::max(x_start, x_end); j++){
                result.push_back(std::make_pair(j, current_y));
            }
        }

        for(auto& e:AEL){
            e.x_min += e.dx; 
        }
        current_y++;
    }
    printf("%d", result.size());
    return result;
}

void Freehand::draw_line(float start_point_x, float start_point_y, float end_point_x, float end_point_y, const Config& config) const{
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddLine(
        ImVec2(config.bias[0] + start_point_x, config.bias[1] + start_point_y),
        ImVec2(config.bias[0] + end_point_x, config.bias[1] + end_point_y),
        IM_COL32(config.line_color[0], config.line_color[1], config.line_color[2], config.line_color[3]), config.line_thickness);
    }

void Freehand::add_control_point(float x, float y ){
    Config config;
    x_list_.push_back(x);
    y_list_.push_back(y);
    size_t size = x_list_.size();
    draw_line(x_list_[size-2], y_list_[size-2], x_list_[size-1], y_list_[size-1], config);
}

void Freehand::build_ET_table(){
    min_y = y_list_[0];
    max_y = y_list_[0];
    for(int i = 0; i < y_list_.size(); i++){
        int y_min = y_list_[i], y_max = y_list_[(i + 1) % y_list_.size()];
        float x_min = x_list_[i], x_max = y_list_[(i + 1) % y_list_.size()];
        if(y_min == y_max)
            continue;
        if(y_min > y_max){
            std::swap(y_min, y_max);
            std::swap(x_min, x_max);
        }
        float dx = (x_min - x_max) / (y_min - y_max);
        y_max --;
        ET[y_min].emplace_back(y_max, x_min, dx);
        min_y = std::min(min_y, y_min);
        max_y = std::max(max_y, y_max);
    }
}
}  // namespace USTC_CG
