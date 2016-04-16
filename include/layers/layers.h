#ifndef LAYERS_H
#define LAYERS_H

#include <string>

#include "Halide.h"
#include "caffe.pb.h"

namespace Latte {

const std::string CONVOLUTION     = "Convolution";
const std::string POOLING         = "Pooling";
const std::string DECONVOLUTION   = "Deconvolution";
const std::string RELU            = "ReLU";
 
const std::string CROP            = "Crop";
const std::string DROPOUT         = "Dropout";
const std::string SPLIT           = "Split";
const std::string SILENCE         = "Silence";

const std::string SOFTMAXWITHLOSS = "SoftmaxWithLoss";
const std::string SOFTMAX         = "Softmax";

class Layer {
  private:
  	std::string name;
  	std::string type;
  	Layer *next;

  protected:
  	void set_name(std::string layer_name) {name = layer_name;}
  	void set_type(std::string layer_type) {type = layer_type;}

  public:
    Layer() {name = ""; type = ""; next = NULL;}
    ~Layer() {}

    std::string get_name() {return name;}
    std::string get_type() {return type;}

    Layer *get_next() {return next;}
    void set_next(Layer *next_layer) {next = next_layer;}

    /**
     * @brief run The genertic function that all inherited classes have to
     * implement to run its specific operations
     *
     * When we call run on the first layer of the net, it will
     * run its computation, be it convolution or pooling, then pass its output
     * to the layer pointed to next and call its run, creating a cascade of
     * operations that will eventually result in a final image being saved.
     *
     * @param input
     *
     * @return 
     */
    virtual Halide::Image<float> run(Halide::Image<float> input) {
      return input;
    }
};

} /* namespace Latte */

#endif /* LAYERS_H */
