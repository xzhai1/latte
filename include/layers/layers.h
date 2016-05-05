#ifndef LAYERS_H
#define LAYERS_H

#include <string>

#include "Halide.h"
#include "caffe.pb.h"

namespace Latte {

const std::string DATA            = "Data";

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
    int width; 
    int height; 
    int channels;
    int num;
  	Layer *next;

  protected:
  	void set_name(std::string layer_name) {name = layer_name;}
  	void set_type(std::string layer_type) {type = layer_type;}
    void set_width    (int w) {width = w;}
    void set_height   (int h) {height = h;}
    void set_channels (int c) {channels = c;}
    void set_num      (int n) {num = n;}

  public:
    Halide::Func storage;
    Halide::Var i, j, k, l;

    Layer(std::string name,
          std::string type) 
      : name(name), type(type), next(NULL) {}
    ~Layer() {}

    /* Accessors do not modify the member variables */
    std::string get_name() const {return name;}
    std::string get_type() const {return type;}
    int get_width()     const {return width;}
    int get_height()    const {return height;}
    int get_channels()  const {return channels;}
    int get_num()       const {return num;}

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
     
    //virtual Halide::Image<float> run(Halide::Image<float> input) {
    //  return input;
    //}
    
    virtual Halide::Func run(Halide::Func input, 
      int input_width, int input_height, int input_channels, int input_num) {
      return input;
    }
};

} /* namespace Latte */

#endif /* LAYERS_H */
