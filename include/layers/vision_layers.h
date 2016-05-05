#ifndef VISION_LAYERS_H
#define VISION_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers/layers.h"

namespace Latte {

class Convolution;
class Pooling;
class Deconvolution;

/**
 * @brief Convolution Layer reduces the area under the kernel by doing the dot
 * product.
 */
class Convolution : public Layer {
  int pad    = 0;
  int stride = 1;
  int num_output, kernel_size;
  Halide::Image<float> kernel, bias;

  public:
    Convolution(std::string layer_name, 
                Layer *prev,
                const caffe::ConvolutionParameter *param, 
                const caffe::BlobProto *weights, 
                const caffe::BlobProto *bias_blob);

    void SetOutputDim(const Layer *prev) {
      /* Input dimension */
      int input_width    = prev->get_width();
      int input_height   = prev->get_height();
      int batch_size     = prev->get_batchsize();

      /* Output dimension */
      int output_width    = (input_width  - kernel_size + 2*pad)/stride + 1;
      int output_height   = (input_height - kernel_size + 2*pad)/stride + 1;
      int output_channels = num_output;

      /* Set output dimension */
      set_output_dim(output_width, output_height, output_channels, batch_size);
    }
};

/**
 * @brief Pooling layer reduces the image over a kernel using a function. 
 * In this case, max is taken over the kernel with zero padding.
 */
class Pooling : public Layer {
  int stride = 1;
  int kernel_size;

  public:
    Pooling(std::string layer_name, Layer *prev,
        const caffe::PoolingParameter *param);
    
    void SetOutputDim(const Layer *prev) {
      /* Input dimension */
      int input_width    = prev->get_width();
      int input_height   = prev->get_height();
      int input_channels = prev->get_channels();
      int batch_size     = prev->get_batchsize();

      /* Output dimension */
      int output_width    = (input_width  - kernel_size)/stride + 1;
      int output_height   = (input_height - kernel_size)/stride + 1;
      int output_channels = input_channels;

      /* Set output dimension */
      set_output_dim(output_width, output_height, output_channels, batch_size);
    }
};

/**
 * @brief Deconvolution layer
 * TODO explain
 */
class Deconvolution : public Layer {
  int stride = 1;
  int kernel_size;
  int num_output;
  Halide::Image<float> bias, kernel;

  public:
    Deconvolution(std::string layer_name, 
                  Layer *prev,
                  const caffe::ConvolutionParameter *param,
                  const caffe::BlobProto *kernel_blob, 
                  const caffe::BlobProto *bias_blob);

    void SetOutputDim(const Layer *prev) {
      /* Input dimension */
      int input_width    = prev->get_width();
      int input_height   = prev->get_height();
      int input_channels = prev->get_channels();
      int batch_size     = prev->get_batchsize();

      /* Output dimension */
      int output_width    = kernel_size + (input_width  - 1)*stride;
      int output_height   = kernel_size + (input_height - 1)*stride;
      int output_channels = num_output;

      /* Set output dimension */
      set_output_dim(output_width, output_height, output_channels, batch_size);
    }
};

} /* namespace latte */

#endif /* VISION_LAYERS_H */
