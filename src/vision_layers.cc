/*
 * Currently implementing feedforward
 *
 */

#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "vision_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

Convolution::Convolution(string layer_name, 
                         const ConvolutionParameter *param, 
                         const BlobProto *weights, 
                         const BlobProto *bias_blob) 
{
  name = layer_name;
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->stride_size()) 
    /* stride is repeated field so we just need the first one. Same goes for
     * pad data */
    stride = param->stride(0);
  if (param->pad_size())
    pad = param->pad(0);
  if (param->has_bias_term())
    bias = LoadBiasFromBlob(bias_blob, num_output);
  kernel = LoadKernelFromBlob(weights, kernel_size, num_output);
}

Image<float>
Convolution::convolve(Image<float> input) 
{
  Func convolution;
  Var x, y, z;  
  int width     = (input.width() - kernel_size + 2 * pad) / stride + 1;
  int height    = (input.height() - kernel_size + 2 * pad) / stride + 1;
  int channels  = input.channels();

  /* Let Halide fill in the blank when we go off the reservation */
  Func clamped = BoundaryConditions::constant_exterior(input, 0.f);

  /* Reduce over kernel */
  RDom r(0, kernel_size, 0, kernel_size, 0, channels);
  convolution(x, y, z) = sum(
      kernel(r.x, r.y, r.z + z*channels) * 
      clamped(x*stride - pad + r.x, y*stride - pad + r.y, r.z));
  /* and add bias */
  convolution(x, y, z) += bias(0, 0, z);
    
  /* TODO define schedule */
  Image<float> output = convolution.realize(width, height, num_output);

  return output;
}

/*****************************************************************************
 *****************************************************************************/
ReLU::ReLU(string layer_name, const ReLUParameter *param) 
{
  name = layer_name;
  if (param->has_negative_slope())
    negative_slope = param->negative_slope();
}

Image<float> 
ReLU::rectify(Image<float> input)
{
  Func rectified;
  Var x, y, z;
  int width    = input.width();
  int height   = input.height();
  int channels = input.channels();

  /* If input is negative, we take only part of it */
  rectified(x, y, z) = max(0, input(x, y, z)) + 
                       negative_slope*min(0, input(x, y, z));

  /* TODO: define schedule */
  Image<float> output = rectified.realize(width, height, channels);
  return output;
}

/*****************************************************************************
 *****************************************************************************/
Pooling::Pooling(string layer_name, const PoolingParameter *param) 
{
  name = layer_name;
  if (param->has_kernel_size())
    kernel_size = param->kernel_size();
  if (param->has_stride())
    stride = param->stride();
}

Image<float>
Pooling::pool(Image<float> input) 
{
  Func pooled;
  Var x, y, z;
  int width    = (input.width() - kernel_size) / stride + 1;
  int height   = (input.height() - kernel_size) / stride + 1;
  int channels = input.channels();

  /* 2D reduction for each channel */
  RDom r(0, kernel_size, 0, kernel_size);
  pooled(x, y, z) = maximum(input(x*stride + r.x, y*stride + r.y, z));

  /* TODO: define schedule */
  Image<float> output = pooled.realize(width, height, channels);
  return output;
}

/*****************************************************************************
 *****************************************************************************/
Dropout::Dropout(string layer_name, const DropoutParameter *param) 
{
  name = layer_name;
  if (param->has_dropout_ratio())
    dropout_ratio = param->dropout_ratio();
}

Image<float> 
Dropout::dropout(Image<float> input) 
{
  Func dropped;
  Var x, y, z;
  int width    = input.width();
  int height   = input.height();
  int channels = input.channels();

  /* TODO only activate in training. Not used in inference */
  // /* Train */
  // dropped(x, y, z) = input(x, y, z) * ((float)rand() / RAND_MAX < dropout_ratio ? 0 : 1);
  
  dropped(x, y, z) = input(x, y, z);

  /* TODO: define schedule */
  Image<float> output = dropped.realize(width, height, channels);

  return output;
}

/*****************************************************************************
 *****************************************************************************/
#if 0
Deconvolution::Deconvolution(string layer_name, 
                             const ConvolutionParameter *param,
                             const BlobProto *kernel_blob, 
                             const BlobProto *bias_blob) 
{
  /* Set name and parameters */
  name = layer_name;
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
Image<float> 
Deconvolution::deconvolve(Image<float> input) {
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

}
#endif

/*****************************************************************************
 *****************************************************************************/
Crop::Crop(string layer_name, const CropParameter *param) 
{
  name = layer_name;
  if (param->offset_size() == 1) {
    offset_x = param->offset(0);
    offset_y = param->offset(0);
  } else {
    // axis in caffe: (N,C,H,W)
    offset_x = param->offset(1); 
    offset_y = param->offset(0);
  }
}

Image<float> 
Crop::crop(Image<float> input) 
{
  Func cropped;
  Var x, y, z;
  int width    = input.width() - 2*offset_x;
  int height   = input.height() - 2*offset_y;
  int channels = input.channels();

  cropped(x, y, z) = input(x + offset_x, y + offset_y, z);

  /* TODO: define schedule */
  Image<float> output = cropped.realize(width, height, channels);
  
  return output;
}

/*****************************************************************************
 *****************************************************************************/
Split::Split(string layer_name) {
  name = layer_name;
}

Image<float> 
Split::split(Image<float> input) 
{
  Func splitted;
  Var x, y, z;
  int width    = input.width();
  int height   = input.height();
  int channels = input.channels();

  splitted(x, y, z) = input(x, y, z);

  /* TODO: define schedule */
  Image<float> output = splitted.realize(width, height, channels);

  return output;
}

/*****************************************************************************
 *****************************************************************************/
SoftmaxWithLoss::SoftmaxWithLoss(string layer_name) 
{
  name = layer_name;
}

Image<float> 
SoftmaxWithLoss::softmaxwithloss(Image<float> input) 
{
  Func loss;
  Func prob;
  Func normalizer;
  Var x, y, z;
  int width    = input.width();
  int height   = input.height();
  int channels = input.channels();

  RDom r(0, width, 0, height, 0, channels);
  prob(x, y, z) = exp(input(x, y, z));
  normalizer(x, y, z) = Halide::sum(prob(x, y, z + r.x));
  prob(x, y, z) = prob(x, y, z)/normalizer(x, y, z);
  loss(x, y, z) = -Halide::sum(log(prob(x + r.x, y + r.y, z + r.z)));

  /* TODO: define schedule */
  Image<float> output = loss.realize(width, height, channels);
  return output;
}

/*****************************************************************************
 *****************************************************************************/
Softmax::Softmax(string layer_name) {
  name = layer_name;
}

Image<float> 
Softmax::softmax(Image<float> input) 
{
  Func prob("prob");
  Func normalizer("normalizer");
  Var x("x"), y("y"), z("z");
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  RDom r(0, channels);
  prob(x, y, z) = Halide::exp(input(x, y, z));
  normalizer(x, y, z) = Halide::sum(prob(x, y, z + r.x));
  prob(x, y, z) = prob(x, y, z)/normalizer(x, y, z);

  /* TODO: define schedule */
  Image<float> output = prob.realize(width, height, channels);
  return output;
}

/*****************************************************************************
 *****************************************************************************/
Silence::Silence(string layer_name) 
{
  name = layer_name;
}

Image<float> 
Silence::silence(Image<float> input) 
{
  Func silenced;
  Var x, y, z;
  int width    = input.width();
  int height   = input.height();
  int channels = input.channels();

  silenced(x, y, z) = input(x, y, z);

  /* TODO: define schedule */
  Image<float> output = silenced.realize(width, height, channels);
  return output;
}

} /* namespace latte */
