// HW2_TODO: Implement the RBFWarper class
#pragma once

#include "warper.h"

#include <vector>
#include <functional>


namespace USTC_CG
{
class RBFWarper : public Warper
{
   public:
    using Warper::Warper;
    virtual ~RBFWarper() = default;
    // HW2_TODO: Implement the warp(...) function with IDW interpolation
    void warp();

    // HW2_TODO: other functions or variables if you need

    double get_distance(point &p, point &q);
    void initial(); //calculate alpha_i

    public:

    struct Matrix
    {
        double x11 = 0;
        double x12 = 0;
        double x21 = 0;
        double x22 = 0;
    }A; //A

    struct Vector21
    {
        double x11 = 0;
        double x21 = 0;
    }B; // B
    

    std::vector<double> r;  //to store r_i
    std::vector<Vector21> alpha; // to store alpha_i 
    std::vector<std::function<double(double)>> g; //to store g_i

    double mu = 2;  //to store \mu
    double sum = 0; //to store the sum of \sigma_i
    bool ini = false; // flag to initial
};
}  // namespace USTC_CG