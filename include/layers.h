#ifndef LAYERS_H
#define LAYERS_H

#include <string>

namespace Latte {

const std::string CONVOLUTION = "Convolution";

class Layer {
  std::string name;
  public:
    Layer() {};
    ~Layer() {};
};

} /* namespace Latte */

#endif /* LAYERS_H */
