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
 * @brief Convolution Layer
 */
class Convolution : public Layer {
  /* Default values */
  int  pad    = 0;
  int  stride = 1;

  /* Filled in values */
  int num_output, kernel_size;
  Halide::Image<float> kernel, bias;

  Halide::Func kernel_func, bias_func;

  public:
    /**
     * @brief Convolution Constructor for convolution layer.
     *
     * Parse the relevant information from protobuf parameter and construct 
     * the convolution kernel and bias. The weights and bias data is a single 
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
     * @brief SerialConv performs plain vanilla convolution with for loops.
     *
     * This serves as a correctness check for the Halide version
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> SerialConv(Halide::Image<float> input);

    /**
     * @brief run Perform convolution on image
     *
     * @param input Input image
     *
     * @return post convolution result
     */
    Halide::Func run(Halide::Func input, int input_width, int input_height, int input_channels, int input_num);
};

/**
 * @brief Pooling layer reduces the image over a kernel using a function. In
 * this case, max is taken over the kernel with zero padding.
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
     * @brief SerialPool performs plain vanilla pooling with for loops.
     *
     * This serves as a correctness check for the Halide version
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> SerialPool(Halide::Image<float> input);

    /**
     * @brief pool Perform max pooling over kernel for each channel
     *
     * @param input Input from previous stage
     *
     * @return Input for next stage
     */
    Halide::Func run(Halide::Func input, int input_width, int input_height, int input_channels, int input_num);
};

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
    Halide::Image<float> run(Halide::Image<float> input);
    Halide::Func run(Halide::Func input, int input_width, int input_height, int input_channels, int input_num);
};

} /* namespace latte */

#endif /* VISION_LAYERS_H */
