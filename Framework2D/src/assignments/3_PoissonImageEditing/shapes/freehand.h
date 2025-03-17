#pragma once

#include "shape.h"
#include <vector>
#include <map>

namespace USTC_CG
{
class Freehand : public Shape
{
   public:
    Freehand() = default;

    // Constructor to initialize a Poligon with start and end coordinates
    Freehand(std::vector<float> x_list, std::vector<float> y_list)
        :x_list_(x_list), y_list_(y_list)
    {
    }

    virtual ~Freehand() = default;

    // Overrides draw function to implement line-specific drawing logic
    void draw(const Config& config) const override;
    std::vector<std::pair<int, int>> get_interior_pixels() override;

    // Overrides Shape's update function to adjust the end point during
    // interaction
    void update(float x, float y) override;

    void add_control_point(float x, float y) override;

    void draw_line(float start_point_x,
        float start_point_y,
        float end_point_x,
        float end_point_y, const Config& config) const;

   private:
    std::vector<float> x_list_, y_list_;
    bool is_closed = false;
    struct Edge{
        int y_max;
        float x_min;
        float dx;
    };
    std::map<int, std::vector<Edge>> ET; // 边表 (y_min为键)
    int min_y, max_y;            // 多边形y范围
    void build_ET_table();
};
}  // namespace USTC_CG