// HW2_TODO: Please implement the abstract class Warper
// 1. The Warper class should abstract the **mathematical mapping** involved in
// the warping problem, **independent of image**.
// 2. The Warper class should have a virtual function warp(...) to be called in
// our image warping application.
//    - You should design the inputs and outputs of warp(...) according to the
//    mathematical abstraction discussed in class.
//    - Generally, the warping map should map one input point to another place.
// 3. Subclasses of Warper, IDWWarper and RBFWarper, should implement the
// warp(...) function to perform the actual warping.
#include <vector>
#include "annoylib.h"
#include "kissrandom.h"
#include <memory>
#pragma once

namespace USTC_CG
{
class Warper
{
   public:
   Warper(int width, int height);
    virtual ~Warper() = default;

    // HW2_TODO: A virtual function warp(...)
    virtual void warp() = 0;
    // HW2_TODO: other functions or variables if you need

    void add_point(int start_x, int start_y, int end_x, int end_y);
    void add_result(int x, int y);
    void fill_blank(int x, int y);

    public:
    struct point
    {
        int start_x = 0;  //start position of x
        int start_y = 0;  //start position of y
        int end_x = 0;
        int end_y = 0;
    };
    std::vector<point> selected_point;     //a vector to store selected information
    point current_point;    //to store information of current point
    
    std::unique_ptr<std::vector<std::vector<bool>>> flag;
    int dim = 2;
    int result_point = 0;
    int neibour_number = 2;
    Annoy::AnnoyIndex<int, float, Annoy::Euclidean, Annoy::Kiss32Random, Annoy::AnnoyIndexSingleThreadedBuildPolicy> index{dim};
    std::vector<int> closed_item{4};
    std::vector<float> distances;
    bool first_ini = false;
};
}  // namespace USTC_CG