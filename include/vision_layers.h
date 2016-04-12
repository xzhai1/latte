#ifndef VISION_LAYERS_H
#define VISION_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers.h"

namespace Latte {

class Convolution;
//class Pooling;

class Convolution : public Layer {
  int num_output, pad = 1, kernel_size;
  Halide::Image<float> kernel;
  public:
    Convolution(std::string name, const caffe::ConvolutionParameter *param, 
                const caffe::BlobProto *blob);
    Halide::Image<float> convolve(Halide::Image<float> input);
};

/*
class Pooling : public Layers {

};
*/

} /* namespace latte */

#endif /* VISION_LAYERS_H */
