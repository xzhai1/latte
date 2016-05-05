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
};

/**
 * @brief Deconvolution layer
 * TODO explain
 */
class Deconvolution : public Layer {
  int stride = 1;
  int kernel_size;
  int num_output;
  Halide::Image<float> bias;
  Halide::Image<float> kernel;

  public:
    Deconvolution(std::string layer_name, 
                  Layer *prev,
                  const caffe::ConvolutionParameter *param,
                  const caffe::BlobProto *kernel_blob, 
                  const caffe::BlobProto *bias_blob);
};

} /* namespace latte */

#endif /* VISION_LAYERS_H */
