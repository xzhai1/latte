#ifndef VISION_LAYERS_H
#define VISION_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers.h"

namespace Latte {

class Convolution;
class ReLU;
class Pooling; // implement the default max-pooling
class Dropout;
class Deconvolution;
class Crop;
class Split;
class SoftmaxWithLoss;
class Softmax;
class Silence;

class Convolution : public Layer {
  int num_output, pad = 0, kernel_size;
  Halide::Image<float> kernel;
public:
  /* TODO need to also give it bias. So best give it all the blobs */
  Convolution(std::string name, const caffe::ConvolutionParameter *param, 
              const caffe::BlobProto *blob);
  Halide::Image<float> convolve(Halide::Image<float> input);
};


class ReLU : public Layers {
	int num_output, float negative_slope;
public:
	ReLU(std:string name, const caffe::ReLUParameter *param, 
			 float negative_slope = 0.f);
	Halide::Image<float> relu(Halide::Image<float> input);
};



// class Pooling : public Layers {

// };



// class Dropout : public Layers {

// };


// class Deconvolution : public Layers {

// };

// class Crop : public Layers {

// };

// class Split : public Layers {

// };

// class SoftmaxWithLoss : public Layers {

// };

// class Softmax : public Layers {

// };

// class Silence : public Layers {

// };

} /* namespace latte */

#endif /* VISION_LAYERS_H */
