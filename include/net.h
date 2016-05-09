#ifndef NET_H
#define NET_H

#include "caffe.pb.h"
#include "Halide.h"

#include "layers/layers.h"

namespace Latte {

class Net {
    Layer *head = NULL;
    Layer *tail = NULL;

  public:
    int input_width;
    int input_height;
    Net(caffe::NetParameter *net_model,
        Halide::Image<float> img);
    ~Net() {}

    /**
     * @brief print_net Prints the network for debugging
     */
    void PrintNet();

    /**
     * @brief run Triggers the cascade of operations that
     * produce the final image
     */
    Halide::Image<float> Run(Halide::Image<float> input, int iterations);
};

} /* namespace Latte */

#endif /* NET_H */
