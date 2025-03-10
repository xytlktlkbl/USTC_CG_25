#include "warper.h"

namespace USTC_CG{
    Warper::Warper(int width, int height)
        :flag(new std::vector<std::vector<bool>>(height, std::vector<bool>(width, false)))
    {
    }

    void Warper::add_point(int start_x, int start_y, int end_x, int end_y){
        point p = {start_x, start_y, end_x, end_y};
        selected_point.push_back(p);
    }

    void Warper::add_result(int x, int y){
        std::vector<float> vec(dim);
        vec[0] = x, vec[1] = y;
        index.add_item(result_point++, vec.data());
        if (x < 0 || x >= flag->size() || y < 0 || y >= (*flag)[0].size()) {
            return;
        }
        (*flag)[x][y] = true;
    }

    void Warper::fill_blank(int x, int y){
        closed_item.clear();
        distances.clear();
        if(!first_ini){
            index.build(10);
            first_ini = true;
        }
        float *vec = new float[2];
        vec[0] = x, vec[1] = y; 
        index.get_nns_by_vector(vec, neibour_number, -1, &closed_item, &distances);
    }
};
