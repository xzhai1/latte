#include <iostream>
#include <algorithm>

#include "Halide.h"
#include "caffe.pb.h"

#include "vision_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

/* Definitions for class Convolution */
// WARNING: assume kernel is square and padding is symmetric
Convolution::Convolution(string name, const ConvolutionParameter *param, 
                         const BlobProto *kernel_blob, const BlobProto *bias_blob) {
  /* Set name and parameters */
  name = name;
  kernel_size = param->kernel_size(0);
  stride = param->stride(0);
  num_output = param->num_output();
  if (param->pad_size()) {
    pad = param->pad(0);
  }

  if (param->has_bias_term()) {
    bias = LoadKernelFromBlob(bias_blob, 1, num_output);
  }
  /* Fill in the kernel */
  kernel = LoadKernelFromBlob(kernel_blob, kernel_size, num_output);
}

Image<float> Convolution::convolve(Image<float> input) {
  Func convolution("convolution");
  Var x("x"), y("y"), z("z");  
  int width = (input.width() - kernel_size + 2 * pad) / stride + 1;
  int height = (input.height() - kernel_size + 2 * pad) / stride + 1;
  int channels = input.channels();

  Func clamped = BoundaryConditions::constant_exterior(input, 0.f);
  // Func clamped = BoundaryConditions::repeat_edge(input);
  RDom r(start, kernel_size, start, kernel_size, 0, channels);
  convolution(x, y, z) = sum(
      kernel(r.x, r.y, r.z + z*channels) * clamped(x * stride - pad+ r.x, 
                                                   y * stride - pad + r.y, r.z));

  /* TODO: define schedule */
  Image<float> output = convolution.realize(width, height, num_output);
  return output;
}

/* Definitions for class ReLU */
ReLU::ReLU(string name, const ReLUParameter *param) {
  name = name;
  if (param->has_negative_slope()) {
    negative_slope = param->negative_slope();
  } else {
    negative_slope = 0.f;
  }
}

/* Definitions for class Pooling */
Image<float> ReLU::relu(Image<float> input) {
  Func rectified("rectified");
  Var x("x"), y("y"), z("z");
  int width = input.width();
  int height = input.height();
  int channels = input.channels();

  rectified(x, y, z) = Halide::max(0, input(x, y, z)) + negative_slope *
                       Halide::min(0, input(x, y, z));

  Image<float> output = rectified.realize(width, height, channels);
  return output;
}


/* Definitions for class Dropout */
// WARNING: assume kernel is square and no padding
Pooling::Pooling(string name, const PoolingParameter *param) {
  /* Set name and parameters */
  // Set name
  name = name;
  // Set kernel size
  if (param->has_kernel_size()) {
    kernel_size = param->kernel_size();
  } else {
    kernel_size = 2; // [default = 2]
  }
  // Set stride
  if (param->has_stride()) {
    stride = param->stride();
  } else {
    stride = 1; // [default = 1]
  }
}

Image<float> Pooling::pool(Image<float> input) {
  Func pooled("pooled");
  Var x("x"), y("y"), z("z");
  int width = (input.width() - kernel_size) / stride + 1;
  int height = (input.height() - kernel_size) / stride + 1;
  int channels = input.channels();

  RDom r(0, kernel_size, 0, kernel_size);
  pooled(x, y, z) = Halide::maximum(input(x * stride + r.x, y * stride + r.y, z));

  /* TODO: Scheduling */
  Image<float> output = pooled.realize(width, height, channels);
}

/* Definitions for class Dropout */

/* Definitions for class Deconvolution */


/* Definitions for class Crop */


/* Definitions for class Split */


/* Definitions for class SoftmaxWithLoss */


/* Definitions for class Softmax */


/* Definitions for class Silence */

} /* namespace latte */
