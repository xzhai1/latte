#ifndef COMMON_LAYERS_H
#define COMMON_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers/layers.h"

namespace Latte {

class Crop;
class Dropout;
class Split;
class Silence;

/**
 * @brief Crop layer
 * TODO  spatial crop
 */
class Crop : public Layer {
  int offset_i;
  int offset_j;
  public:
    Crop(std::string name, 
         Layer *prev,
         const caffe::CropParameter *param, 
         int input_width, int input_height);

    void SetOutputDim(const Layer *prev) {
      /* Output dimension */
      int output_width    = prev->get_width() - 2*offset_i;
      int output_height   = prev->get_height()- 2*offset_j;
      int output_channels = prev->get_channels();
      int batch_size      = prev->get_batchsize();

      /* Set output dimension */
      set_output_dim(output_width, output_height, output_channels, batch_size);
    }
};

/**
 * @brief Dropout layer
 * TODO explain what this does
 */
class Dropout : public Layer {
    float dropout_ratio = 0.5f;
  public:
    Dropout(std::string layer_name, const caffe::DropoutParameter *param);
};

/**
 * @brief NOOP
 */
class Split : public Layer {
  public:
    Split(std::string name);
};

/**
 * @brief Silence layer
 * TODO NOPS
 */
class Silence : public Layer {
  public:
    Silence(std::string layer_name);
};

} /* namespace latte */

#endif /* COMMON_LAYERS_H */
