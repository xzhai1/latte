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
  	/* mutators */
  	void set_name(std::string layer_name);
  	void set_type(std::string layer_type);

  public:
    /* constructor */
    Layer();
    /* deconstructor */
    ~Layer();
    /* accessors */
    std::string get_name();
    std::string get_type();
    Layer *get_next();
    /* mutator */
    void set_next(Layer *next_layer);
    /* virtual functions */
    virtual Halide::Image<float> run(Halide::Image<float> input);
};

} /* namespace Latte */

#endif /* LAYERS_H */
