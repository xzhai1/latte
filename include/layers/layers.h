#ifndef LAYERS_H
#define LAYERS_H

#include <string>

namespace Latte {

const std::string CONVOLUTION     = "Convolution";
const std::string POOLING         = "Pooling";
const std::string DECONVOLUTION   = "Deconvolution";

const std::string ReLU            = "ReLU";
 
const std::string CROP            = "Crop";
const std::string DROPOUT         = "Dropout";
const std::string SPLIT           = "Split";
const std::string SILENCE         = "Silence";

const std::string SOFTMAXWITHLOSS = "SoftmaxWithLoss";
const std::string SOFTMAX         = "Softmax";

//const int FILTERS_PER_ROW = 8;

class Layer {
  public:
    std::string name;
    Layer() {};
    ~Layer() {};
};

} /* namespace Latte */

#endif /* LAYERS_H */
