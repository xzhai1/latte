#ifndef VISION_LAYERS_H
#define VISION_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers.h"

namespace Latte {

class Convolution;
class Pooling;
class Deconvolution;
class Crop;

/**
 * @brief Convolution Layer
 */
class Convolution : public Layer {
  /* Default values */
  int  pad = 0;
  int  stride = 1;
  Halide::Image<float> bias = Halide::Image<float>(1, 1, num_output);

  /* Filled in values */
  int  num_output, kernel_size;
  Halide::Image<float> kernel;

  public:
    /**
     * @brief Convolution Constructor for convolution layer.
     *
     * Parse the relevant information from protobuf parameter and construct the
     * convolution kernel and bias. The weights and bias data is a single 
     * array stored in row major order. 
     *
     * @param layer_name Name given in the model
     * @param param      Parsed ConvolutionParameter
     * @param weights    Weight BlobProto
     * @param bias_blob  Bias BlobProto
     */
    Convolution(std::string layer_name, 
                const caffe::ConvolutionParameter *param, 
                const caffe::BlobProto *weights, 
                const caffe::BlobProto *bias_blob);

    /**
     * @brief convolve Perform convolution on image
     *
     * @param input Input image
     *
     * @return post convolution result
     */
    Halide::Image<float> convolve(Halide::Image<float> input);
};

/**
 * @brief Pooling layer
 *
 * TODO Brief explain what this does
 * The model's default is Max pool with zero padding
 */
class Pooling : public Layer {
    int kernel_size = 2;
    int stride      = 1;

  public:
    /**
     * @brief Pooling 
     *
     * @param layer_name Name given in the model
     * @param param      Parsed PoolingParameter from the caffemodel
     */
    Pooling(std::string layer_name, const caffe::PoolingParameter *param);

    /**
     * @brief pool Perform max pooling over kernel for each channel
     *
     * @param input Input from previous stage
     *
     * @return Input for next stage
     */
    Halide::Image<float> pool(Halide::Image<float> input);
};

#if 0
/**
 * @brief Deconvolution layer
 * TODO explain
 */
class Deconvolution : public Layer {
  int kernel_size;
  int stride;
  int num_output;
  Halide::Image<float> bias;
  Halide::Image<float> kernel;

  public:
    Deconvolution(std::string layer_name, 
                  const caffe::ConvolutionParameter *param,
                  const caffe::BlobProto *kernel_blob, 
                  const caffe::BlobProto *bias_blob);
    Halide::Image<float> deconvolve(Halide::Image<float> input);
};
#endif

} /* namespace latte */

#endif /* VISION_LAYERS_H */
