#ifndef NET_H
#define NET_H

#include "caffe.pb.h"
#include "Halide.h"

#include "layers/layers.h"

namespace Latte {

//class Data;

class Net {
    Layer *head = NULL;
    Layer *tail = NULL;
    Layer *data = NULL;

  public:
    Net(caffe::NetParameter *net_model,
        Halide::Image<float> tmp_img);
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
