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
              const caffe::BlobProto *kernel_blob, const caffe::BlobProto *bias_blob);
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

class Dropout : public Layer {
	float dropout_ratio;
public:
	// WARNING: Only training phase needs dropout
	Dropout(std::string name, const caffe::DropoutParameter *param);
	Halide::Image<float> dropout(Halide::Image<float> input);
};

class Deconvolution : public Layer {
	int kernel_size;
  int stride;
  int num_output;
  Halide::Image<float> bias;
  Halide::Image<float> kernel;
public:
	Deconvolution(std:string name, const caffe::ConvolutionParameter *param,
								const caffe::BlobProto *kernel_blob, const caffe::BlobProto *bias_blob);
	Halide::Image<float> deconvolve(Halide::Image<float> input);
};

class Crop : public Layer {
	int offset_x;
	int offset_y;
public:
	// WARNING: Assume spatial crop
	Crop(std::string name, const caffe::CropParameter *param);
	Halide::Image<float> crop(Halide::Image<float> input);
};

class Split : public Layer {
	// WARNING: Only implementing feedforward, no ops
public:
	Split(std::string name);
	Halide::Image<float> split(Halide::Image<float> input);
};

class SoftmaxWithLoss : public Layer {
public:
	SoftmaxWithLoss(std::string name);
	Halide::Image<float> softmaxwithloss(Halide::Image<float> input);
};

class Softmax : public Layer {
public:
	Softmax(std::string name);
	Halide::Image<float> softmax(Halide::Image<float> input);
};

class Silence : public Layer {
	// WARNING: Only implementing feedforward, no ops
public:
	Silence(std::string name);
	Halide::Image<float> silence(Halide::Image<float> input);
};

} /* namespace latte */

#endif /* VISION_LAYERS_H */





