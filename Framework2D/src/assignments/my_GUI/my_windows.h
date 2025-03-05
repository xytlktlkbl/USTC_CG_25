#pragma once

#include "common/window.h"

namespace USTC_CG
{
class MiniDraw : public Window
{
   public:
    explicit MiniDraw(const std::string& window_name);
    ~MiniDraw();

    void draw();    // 重载 Window 的绘制函数 draw()

   private:
    bool flag_show_canvas_common = true;        // Priavte attributes
};
}  // namespace USTC_CG