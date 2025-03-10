#pragma once

#include "warper.h"
#include <dlib/statistics.h>
#include <dlib/dnn.h>
#include <iostream>
#include <vector>

namespace USTC_CG
{
class DlibWarper : public Warper
{
    public:
    using Warper::Warper;
    void warp();
    void initial();
    void train();

    public:
    dlib::matrix<double> src{2, 1};
    dlib::matrix<float> dst{2, 1};
    std::vector<dlib::matrix<double>> src_vec_input, src_vec;
    std::vector<dlib::matrix<float>> dst_vec_input, dst_vec;
    dlib::matrix<float> output_means, output_stds;
    dlib::vector_normalizer<dlib::matrix<double>> input_normalizer;
    dlib::vector_normalizer<dlib::matrix<float>> output_normalizer;
    using network_type = dlib::loss_mean_squared_multioutput< // Loss function: mean squared error
    dlib::fc<2, // Full connection layer with 2 output neurons
    dlib::relu<dlib::fc<64, // Full connection layer with 64 neurons, followed by ReLU activation
    dlib::relu<dlib::fc<64, // Full connection layer with 64 neurons, followed by ReLU activation
    dlib::input<dlib::matrix<double>> // Input layer
    >>>>>>;
    dlib::adam solver{0.002, 0.9, 0.999};
    // Initialize the trainer
    network_type net;
    dlib::dnn_trainer<network_type, dlib::adam> trainer{net, solver};
    bool ini = false;
};
}