// HW2_TODO: Implement the IDWWarper class
#pragma once

#include "warper.h"
#include <vector>


namespace USTC_CG
{
class IDWWarper : public Warper
{
   public:
    using Warper::Warper;
    virtual ~IDWWarper() = default;
    // HW2_TODO: Implement the warp(...) function with IDW interpolation
    void warp();

    // HW2_TODO: other functions or variables if you need

    void calculate_weight(point &p); //calculate w based on p, and store the result in weight

    void calculate_sigma(point &p); //calculate \sigma based on p, and store the result in sigma
    double calculate_two_point_sigma(point &p, point &q);
    void initial(); //calculate T_i

    public:

    struct Matrix
    {
        double x11 = 0;
        double x12 = 0;
        double x21 = 0;
        double x22 = 0;
    };

    std::vector<double> sigma;  //to store \sigma_i
    std::vector<double> weight; //to store \weight_i
    std::vector<Matrix> T; // to store T_i 

    double mu = 2;  //to store \mu
    double sum = 0; //to store the sum of \sigma_i
    bool ini = false; // flag to initial
};
}  // namespace USTC_CG