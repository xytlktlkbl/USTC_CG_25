#pragma once

#include "shape.h"
#include <vector>
namespace USTC_CG
{
class Polygon : public Shape
{
   public:
    Polygon() = default;

    // Constructor to initialize a Poligon with start and end coordinates
    Polygon(std::vector<float> x_list, std::vector<float> y_list)
        :x_list_(x_list), y_list_(y_list)
    {
    }

    virtual ~Polygon() = default;

    // Overrides draw function to implement line-specific drawing logic
    void draw(const Config& config) const override;

    // Overrides Shape's update function to adjust the end point during
    // interaction
    void update(float x, float y) override;

    void add_control_point(float x, float y) override;

    void close_polygon() override;

    void draw_line(float start_point_x,
        float start_point_y,
        float end_point_x,
        float end_point_y, const Config& config) const;

   private:
    std::vector<float> x_list_, y_list_;
    bool is_closed = false;
};
}  // namespace USTC_CG