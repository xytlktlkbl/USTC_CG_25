#include "Dlib_warper.h"

namespace USTC_CG{

    void DlibWarper::initial(){
        for(int i = 0; i < selected_point.size(); i++){
            src(0) = selected_point[i].start_x;
            src(1) = selected_point[i].start_y;
            dst(0) = selected_point[i].end_x,
            dst(1) = selected_point[i].end_y;
            src_vec_input.push_back(src);
            src_vec.push_back(src);
            dst_vec_input.push_back(dst);
            dst_vec.push_back(dst);
        }
    }

    void DlibWarper::train(){
        input_normalizer.train(src_vec);
        for (auto& x : src_vec) 
            x = input_normalizer(x);
        output_normalizer.train(dst_vec);
        output_means = output_normalizer.means();
        output_stds = output_normalizer.std_devs();
        for (auto& x : dst_vec) 
            x = output_normalizer(x);
        trainer.set_learning_rate(0.001);
        trainer.set_min_learning_rate(1e-6);
        trainer.set_mini_batch_size(128);
        trainer.set_learning_rate_shrink_factor(0.1);
        trainer.set_iterations_without_progress_threshold(500);
        trainer.be_verbose();

        trainer.train(src_vec, dst_vec);

    }

    void DlibWarper::warp(){
        if(!ini){
            initial();
            train();
            ini = true;
        }
        dlib::matrix<double> input(2, 1);
        input(0, 0) = current_point.start_x;
        input(1, 0) = current_point.start_y;
        input = input_normalizer(input);
        dlib::matrix<float> test_out = net(input);
        current_point.end_x = test_out(0, 0) / output_stds(0, 0) + output_means(0, 0);
        current_point.end_y = test_out(1, 0) / output_stds(1, 0) + output_means(1, 0);
    }
    
}