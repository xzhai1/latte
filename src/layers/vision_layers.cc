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

} /* namespace latte */