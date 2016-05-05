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
    Crop(std::string name, const caffe::CropParameter *param);
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
