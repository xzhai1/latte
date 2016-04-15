#ifndef LAYERS_H
#define LAYERS_H

#include <string>

namespace Latte {

const std::string CONVOLUTION = "Convolution";
const std::string POOLING     = "Pooling";

const int FILTERS_PER_ROW = 8;

class Layer {
  public:
    std::string name;
    Layer() {};
    ~Layer() {};
};

} /* namespace Latte */

#endif /* LAYERS_H */
