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
      int input_width = prev->get_width();
      int input_height = prev->get_height();
      int input_channels = prev->get_channels();
      int batch_size = prev->get_batchsize();

      /* Output dimension */
      int output_width    = input_width;
      int output_height   = input_height;
      int output_channels = input_channels;

      /* Set output dimension */
      set_output_dim(output_width, output_height, output_channels, batch_size);
    }
};

} /* namespace latte */

#endif /* ACTIVATION_LAYERS_H */
