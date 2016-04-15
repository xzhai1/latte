/*
 * Currently implementing feedforward
 *
 */

#include <iostream>
#include <algorithm>
#include <stdlib.h>

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
  if (param->stride_size()) stride = param->stride(0);
  else stride = 1;
  num_output = param->num_output();
  if (param->pad_size()) {
    pad = param->pad(0);
  } else {
    pad = 0;
  }
  /* Fill in the bias */
  if (param->has_bias_term()) {
    bias = LoadKernelFromBlob(bias_blob, 1, num_output);
  } else {
    bias = 0.f;
  }
  /* Fill in the kernel */
  kernel = LoadKernelFromBlob(kernel_blob, kernel_size, num_output);
}

Image<float> Convolution::convolve(Image<float> input) {
  Func convolution("convolution");
  Var x("x"), y("y"), z("z");  
  int width     = (input.width() - kernel_size + 2 * pad) / stride + 1;
  int height    = (input.height() - kernel_size + 2 * pad) / stride + 1;
  int channels  = input.channels();

  Func clamped = BoundaryConditions::constant_exterior(input, 0.f);
  RDom r(0, kernel_size, 0, kernel_size, 0, channels);
  convolution(x, y, z) = sum(
      kernel(r.x, r.y, r.z + z*channels) * 
      clamped(x * stride - pad + r.x, y * stride - pad + r.y, r.z)) + 
      bias(r.z);

  /* TODO: define schedule */
  Image<float> output = convolution.realize(width, height, num_output);
  return output;
}

/* Definitions for class ReLU */
ReLU::ReLU(string name, const ReLUParameter *param) {
  /* Set name and parameter */
  name = name;
  if (param->has_negative_slope()) {
    negative_slope = param->negative_slope();
  } else {
    negative_slope = 0.f;
  }
}

Image<float> ReLU::relu(Image<float> input) {
  Func rectified("rectified");
  Var x("x"), y("y"), z("z");
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  rectified(x, y, z) = Halide::max(0, input(x, y, z)) + negative_slope *
                       Halide::min(0, input(x, y, z));

  /* TODO: define schedule */
  Image<float> output = rectified.realize(width, height, channels);
  return output;
}

/* Definitions for class Pooling */
// WARNING: assume kernel is square and no padding
Pooling::Pooling(string name, const PoolingParameter *param) {
  /* Set name and parameters */
  name = name;
  if (param->has_kernel_size()) {
    kernel_size = param->kernel_size();
  } else {
    kernel_size = 2; // [default = 2]
  }
  if (param->has_stride()) {
    stride = param->stride();
  } else {
    stride = 1; // [default = 1]
  }
}

Image<float> Pooling::pool(Image<float> input) {
  Func pooled("pooled");
  Var x("x"), y("y"), z("z");
  int width     = (input.width() - kernel_size) / stride + 1;
  int height    = (input.height() - kernel_size) / stride + 1;
  int channels  = input.channels();

  RDom r(0, kernel_size, 0, kernel_size);
  pooled(x, y, z) = Halide::maximum(input(x * stride + r.x, y * stride + r.y, z));

  /* TODO: define schedule */
  Image<float> output = pooled.realize(width, height, channels);
  return output;
}

/* Definitions for class Dropout */
// WARNING: Only training phase needs dropout
Dropout::Dropout(string name, const DropoutParameter *param) {
  /* Set name and parameter */
  name = name;
  if (param->has_dropout_ratio()) {
    dropout = param->dropout_ratio();
  } else {
    dropout = 0.5f;
  }
}

Image<float> dropout(Image<float> input) {
 
  Func dropped("dropped");
  Var x("x"), y("y"), z("z");
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  // /* Train */
  // dropped(x, y, z) = input(x, y, z) * ((float)rand() / RAND_MAX < dropout_ratio ? 0 : 1);
  // /* TODO: define schedule */
  // Image<float> output = dropped.realize(width, height, channels);

  /* Test */
  dropped(x, y, z) = input(x, y, z);

  /* TODO: define schedule */
  Image<float> output = dropped.realize(width, height, channels);
  return output;
}


/* Definitions for class Deconvolution */
Deconvolution::Deconvolution(string name, const ConvolutionParameter *param,
                             const BlobProto *kernel_blob, const BlobProto *bias_blob) {
  /* Set name and parameters */
  name = name;
  kernel_size = param->kernel_size(0);
  if (param->stride_size()) stride = param->stride(0);
  else stride = 1;
  num_output = param->num_output();
  /* Fill in the bias */
  if (param->has_bias_term()) {
    bias = LoadKernelFromBlob(bias_blob, 1, num_output);
  } else {
    bias = 0.f;
  }
  /* Fill in the kernel */
  kernel = LoadKernelFromBlob(kernel_blob, kernel_size, num_output);
}

// WARNING: This implementation only assumes upsampling with 2 * stride == kernel_size
Image<float> deconvolve(Image<float> input) {
  Func convolution("deconvolution");
  Var x("x"), y("y"), z("z");
  int width     = input.width() * stride;
  int height    = input.height() * stride;
  int channels  = input.channels();

  RDom r(0, kernel_size, 0, kernel_size, 0, channels);
  convolution(x, y, z) = sum(
      kernel(r.x, r.y, r.z + z*channels) * 
      clamped(x * stride - pad + r.x, y * stride - pad + r.y, r.z)) + 
      bias(r.z);

  /* TODO: define schedule */
  Image<float> output = convolution.realize(width, height, num_output);
  return output;
}

/* Definitions for class Crop */
// WARNING: Assume spatial crop
Crop::Crop(string name, const CropParameter *param) {
  /* Set name and parameters */
  name = name;
  if (param->offset_size() == 1) {
    offset_x = param->offset(0);
    offset_y = param->offset(0);
  } else {
    // axis in caffe: (N,C,H,W)
    offset_x = param->offset(1); 
    offset_y = param->offset(0);
  }
}

Image<float> Crop::crop(Image<float> input) {
  Fun cropped("cropped");
  var x("x"), y("y"), z("z");
  int width     = input.width() - 2 * offset_x;
  int height    = input.height() - 2 * offset_y;
  int channels  = input.channels();

  cropped(x, y, z) = input(x + offset_x, y + offset_y, z);

  /* TODO: define schedule */
  Image<float> output = cropped.realize(width, height, channels);
}

/* Definitions for class Split */
// WARNING: Only implementing feedforward, no ops
Split::Split(string name) {
  /* Set name */
  name = name;
}

Image<float> Split::split(Image<float> input) {
  Func splitted("splitted");
  Var x("x"), y("y"), z("z");
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  splitted(x, y, z) = input(x, y, z);

  /* TODO: define schedule */
  Image<float> output = splitted.realize(width, height, channels);
  return output;
}

/* Definitions for class SoftmaxWithLoss */
// WARNING: Not sure about the implementation
SoftmaxWithLoss::SoftmaxWithLoss(string name) {
  /* Set name and parameter */
  name = name;
  loss = 0.f;
}

Image<float> SoftmaxWithLoss::softmaxwithloss(Image<float> input) {
  Func loss("loss");
  Func prob("prob");
  Func normalizer("normalizer");
  Var x("x"), y("y"), z("z");
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  RDom r(0, width, 0, height, 0, channels);
  prob(x, y, z) = Halide::exp(x, y, z);
  normalizer(x, y, z) = Halide::sum(prob(x, y, z + r.x));
  prob(x, y, z) = prob(x, y, z)/normalizer(x, y, z);
  loss(x, y, z) = -Halide::sum(log(prob(x + r.x, y + r.y, z + r.z)));

  /* TODO: define schedule */
  Image<float> output = loss.realize(width, height, channels);
  return output;
}

/* Definitions for class Softmax */
Softmax::Softmax(string name) {
  /* Set name */
  name = name;
}

Image<float> Softmax::softmax(Image<float> input) {
  Func prob("prob");
  Func normalizer("normalizer");
  Var x("x"), y("y"), z("z");
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  RDom r(0, channels);
  prob(x, y, z) = Halide::exp(x, y, z);
  normalizer(x, y, z) = Halide::sum(prob(x, y, z + r.x));
  prob(x, y, z) = prob(x, y, z)/normalizer(x, y, z);

  /* TODO: define schedule */
  Image<float> output = prob.realize(width, height, channels);
  return output;
}

/* Definitions for class Silence */
// WARNING: Only implementing feedforward, no ops
Silence::Silence(string name) {
  /* Set name */
  name = name;
}

Image<float> Silence::silence(Image<float> input) {
  Func silenced("silenced");
  Var x("x"), y("y"), z("z");
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  silenced(x, y, z) = input(x, y, z);

  /* TODO: define schedule */
  Image<float> output = silenced.realize(width, height, channels);
  return output;
}

} /* namespace latte */
