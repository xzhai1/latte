#ifndef COMMON_LAYERS_H
#define COMMON_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers.h"

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
    Halide::Image<float> run(Halide::Image<float> input);
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
    Halide::Image<float> run(Halide::Image<float> input);
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
    Halide::Image<float> run(Halide::Image<float> input);
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
    Halide::Image<float> run(Halide::Image<float> input);
};

} /* namespace latte */

#endif /* COMMON_LAYERS_H */
