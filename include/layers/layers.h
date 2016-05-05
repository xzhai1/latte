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
  protected:
  	void set_name(std::string layer_name) {name = layer_name;}
  	void set_type(std::string layer_type) {type = layer_type;}
    void set_width    (int w) {width = w;}
    void set_height   (int h) {height = h;}
    void set_channels (int c) {channels = c;}
    void set_output_dim(int w, int h, int c, int n) {
      width    = w;
      height   = h;
      channels = c;
      batchsize = n;
    }

  public:
    /* For other layers */
    Layer(std::string name,
          std::string type) 
      : name(name), type(type), next(NULL) {}

    /* For first data layer */
    Layer(std::string name,
          std::string type,
          Halide::ImageParam img) 
      : name(name), type(type), next(NULL) {}

    ~Layer() {}

    Halide::Var i, j, k, l;
    Halide::Func storage;

    /* Accessors do not modify the member variables */
    std::string get_name() const {return name;}
    std::string get_type() const {return type;}
    int get_width()     const {return width;}
    int get_height()    const {return height;}
    int get_channels()  const {return channels;}
    int get_batchsize() const {return batchsize;}

    Layer *get_next() {return next;}
    void set_next(Layer *next_layer) {next = next_layer;}

  private:
  	std::string name;
  	std::string type;
    
    /* Layer outout dimensions */
    int width; 
    int height; 
    int channels; 
    int batchsize;      /* Batch size */
  	Layer *next;
};

} /* namespace Latte */

#endif /* LAYERS_H */
