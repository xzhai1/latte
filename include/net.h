#ifndef NET_H
#define NET_H

#include "caffe.pb.h"
#include "Halide.h"

#include "layers.h"

namespace Latte {

class Net {
    Layer *head = NULL;

  public:
    Net(caffe::NetParameter *net_model);
    ~Net() {}

    /**
     * @brief print_net Prints the network for debugging
     */
    void print_net();

    /**
     * @brief run Triggers the cascade of operations that
     * produce the final image
     */
    Halide::Image<float> run(Halide::Image<float> input);
};

} /* namespace Latte */

#endif /* NET_H */
