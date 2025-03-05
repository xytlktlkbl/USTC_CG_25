#include "my_windows.h"

namespace USTC_CG{ 

void MiniDraw::draw()
{
    // flag_show_canvas_common 为bool类型，记录窗口的打开状态

    if (ImGui::Begin("Canvas", &flag_show_canvas_common))
    {
        ImGui::Text("This is a canvas window");
    }
    ImGui::End();
}
}