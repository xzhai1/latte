#ifndef VISION_LAYERS_H
#define VISION_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers.h"

namespace Latte {

class Convolution;
class ReLU;
class Pooling;
class Dropout;
class Deconvolution;
class Crop;
class Split;
class SoftmaxWithLoss;
class Softmax;
class Silence;

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
 * @brief Rectified Linear Unit 
 * TODO copy explaination from caffe
 */
class ReLU : public Layer {
	  float negative_slope = 0.f;

  public:
    /**
     * @brief ReLU 
     *
     * @param layer_name Name given in the model
     * @param param      Parsed ReLUParameter from the caffemodel
     */
    ReLU(std::string layer_name, const caffe::ReLUParameter *param);

    /**
     * @brief relu Performs linear rectification on the input
     *
     * @param input Input from previous stage
     *
     * @return Input for next stage
     */
    Halide::Image<float> rectify(Halide::Image<float> input);
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

/**
 * @brief Dropout layer
 * TODO explain what this does
 */
class Dropout : public Layer {
    float dropout_ratio = 0.5f;

  public:
    /**
     * @brief Dropout
     *
     * @param layer_name Name given in the model
     * @param param      Parsed DropoutParameter from the caffemodel
     */
    Dropout(std::string layer_name, const caffe::DropoutParameter *param);

    /**
     * @brief dropout 
     *
     * @param input Input from previous stage
     *
     * @return Input to next stage
     */
    Halide::Image<float> dropout(Halide::Image<float> input);
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

/**
 * @brief Crop layer
 * TODO  spatial crop
 */
class Crop : public Layer {
	int offset_x;
	int offset_y;

  public:
    /**
     * @brief Crop 
     *
     * @param layer_name Name given in the model
     * @param param      Parsed CropParameter from the caffemodel
     */
    Crop(std::string name, const caffe::CropParameter *param);

    /**
     * @brief crop Performs cropping in x and y dimesion on both sides
     *
     * @param input Input from previous stage
     *
     * @return Input to next stage
     */
    Halide::Image<float> crop(Halide::Image<float> input);
};

/**
 * @brief NOOP
 */
class Split : public Layer {
  public:
    /**
     * @brief Split 
     *
     * @param name
     */
    Split(std::string name);

    /**
     * @brief split 
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> split(Halide::Image<float> input);
};

/**
 * @brief 
 */
class SoftmaxWithLoss : public Layer {
  public:
    /**
     * @brief SoftmaxWithLoss 
     *
     * @param name
     */
    SoftmaxWithLoss(std::string layer_name);
    
    /**
     * @brief softmaxwithloss 
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> softmaxwithloss(Halide::Image<float> input);
};

/**
 * @brief Softmax layer
 * TODO explain 
 */
class Softmax : public Layer {
  public:
    /**
     * @brief Softmax 
     *
     * @param name
     */
    Softmax(std::string name);

    /**
     * @brief softmax 
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> softmax(Halide::Image<float> input);
};

/**
 * @brief Silence layer
 * TODO NOPS
 */
class Silence : public Layer {
  public:
    /**
     * @brief Silence 
     *
     * @param layer_name
     */
    Silence(std::string layer_name);

    /**
     * @brief silence NOOP
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> silence(Halide::Image<float> input);
};

} /* namespace latte */

#endif /* VISION_LAYERS_H */





