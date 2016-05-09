#ifndef ACTIVATION_LAYERS_H
#define ACTIVATION_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers/layers.h"

/**
 * @brief
 *
 * In general, activation / Neuron layers are element-wise operators, taking 
 * one bottom blob and producing one top blob of the same size. 
 */
namespace Latte {

class ReLU;

/**
 * @brief Rectified Linear Unit 
 *
 * Given an input value x, The ReLU layer computes the output as x if x > 0 
 * and negative_slope * x if x <= 0. When the negative slope parameter is 
 * not set, it is equivalent to the standard ReLU function of taking
 * max(x, 0). 
 */
class ReLU : public Layer {
	  float negative_slope = 0.f;

  public:
    ReLU(std::string layer_name, 
         const Layer *prev, 
         const caffe::ReLUParameter *param);

    void SetOutputDim(const Layer *prev) {
      set_output_dim(prev->get_width(),
                     prev->get_height(), 
                     prev->get_channels(), 
                     prev->get_batchsize());
    }
};

} /* namespace latte */

#endif /* ACTIVATION_LAYERS_H */
