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
	int kernel_size;
  int stride;
  int pad;
  int num_output;
  Halide::Image<float> bias;
  Halide::Image<float> kernel;
public:
  /* TODO need to also give it bias. So best give it all the blobs */
  Convolution(std::string name, const caffe::ConvolutionParameter *param, 
              const caffe::BlobProto *kernel_blob, , const caffe::BlobProto *bias_blob);
  Halide::Image<float> convolve(Halide::Image<float> input);
};


class ReLU : public Layer {
	float negative_slope;
public:
	ReLU(std::string name, const caffe::ReLUParameter *param);
	Halide::Image<float> relu(Halide::Image<float> input);
};


// It is not common to have zero paddings
class Pooling : public Layer {
	int kernel_size;
	int stride;
public:
	Pooling(std::string name, const caffe::PoolingParameter *param);
	Halide::Image<float> pool(Halide::Image<float> input);
};



// class Dropout : public Layer {

// };


// class Deconvolution : public Layer {

// };

// class Crop : public Layer {

// };

// class Split : public Layer {

// };

// class SoftmaxWithLoss : public Layer {

// };

// class Softmax : public Layer {

// };

// class Silence : public Layer {

// };

} /* namespace latte */

#endif /* VISION_LAYERS_H */





