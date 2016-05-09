#ifndef COMMON_LAYERS_H
#define COMMON_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers/layers.h"

namespace Latte {

/**
 * @brief Data layer is just a dummy layer to hold the image that comes in so
 * we can bootstrap the whole net
 */
class Data : public Layer {
  public:
    Data(std::string name,
         int width,
         int height,
         int channels,
         int num,
         Halide::ImageParam img)
      :Layer(name, DATA, img) {
        set_output_dim(width, height, channels, num);
        storage(i, j, k, l) = img(i, j, k, l);
    }
};

/**
 * @brief Crop layer restores the result output deconv layer back to the same
 * dimension as the input image
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


} /* namespace latte */

#endif /* COMMON_LAYERS_H */
