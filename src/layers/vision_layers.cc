#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "layers/vision_layers.h"
#include "proto2img_utils.h" 

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

Convolution::Convolution(string layer_name,
                         Layer *prev,
                         const ConvolutionParameter *param, 
                         const BlobProto *weights, 
                         const BlobProto *bias_blob)
  :Layer(layer_name, CONVOLUTION) 
{
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  /* stride is repeated field so we just need the first one.
   * Same goes for pad data */
  if (param->stride_size()) 
    stride = param->stride(0);
  if (param->pad_size())
    pad = param->pad(0);

  /* Default 0 bias */
  bias = Image<float>(1, 1, 1, num_output);
  if (bias_blob)
    bias = LoadBiasFromBlob(bias_blob, num_output);
  kernel = LoadKernelFromBlob(weights, kernel_size, num_output);

  /* Input dimension */
  int input_width = prev->get_width();
  int input_height = prev->get_height();
  int input_channels = prev->get_channels();
  //int batch_size = prev->get_num();
  int batch_size = prev->get_batchsize();

  /* Output dimension */
  int output_width     = (input_width  - kernel_size + 2 * pad) / stride + 1;
  int output_height    = (input_height - kernel_size + 2 * pad) / stride + 1;
  int output_channels  = num_output;
  //int output_num       = input_num;

  /* Set output dimension */
  set_output_dim(output_width, output_height, output_channels, batch_size);

  /* Clamped at boundary */
  Func clamped = BoundaryConditions::constant_exterior(
      prev->storage, 0.f, 0, prev->get_width(), 0, prev->get_height());

  /* Reduce over one kernel */
  RDom r(0, kernel_size, 0, kernel_size, 0, prev->get_channels());
  storage(i, j, k, l) = sum(kernel(r.x, r.y, r.z, k) * 
                        clamped(i*stride + r.x - pad, 
                                j*stride + r.y - pad, 
                                r.z, l))
                        + bias(0, 0, 0, k);
  /* Scheduling */
  storage.compute_root();
  storage.parallel(k);
  int split_num = output_height > 15 ? output_height / 15 : 8;
  int vector_size = (output_width >= 16) ? 16 : 8;

  if (output_width * 2 <= output_channels) {
    Var jo, ji;
    storage.split(j, jo, ji, split_num).parallel(jo);
    storage.vectorize(i, vector_size);
    clamped.store_at(storage, jo).compute_at(storage, ji);
  } else {
    storage.vectorize(i, vector_size);
  }
}

Pooling::Pooling(string layer_name,
                 Layer *prev,
                 const PoolingParameter *param) 
  :Layer(layer_name, POOLING) 
{
  if (param->has_kernel_size())
    kernel_size = param->kernel_size();
  if (param->has_stride())
    stride = param->stride();

  int input_width = prev->get_width();
  int input_height = prev->get_height();
  int input_channels = prev->get_channels();
  //int batch_size = prev->get_num();
  int batch_size = prev->get_batchsize();

  /* Compute output dimension */
  int output_width    = (input_width - kernel_size) / stride + 1;
  int output_height   = (input_height - kernel_size) / stride + 1;
  int output_channels = input_channels;
  //int output_num      = input_num;

  /* Set output dimension */
  set_output_dim(output_width, output_height, output_channels, batch_size);

  RDom r(0, kernel_size, 0, kernel_size);
  storage(i, j, k, l) = maximum(
      prev->storage(i*stride + r.x, j*stride + r.y, k, l));

  /* CPU parallel */
  storage.compute_root();
  storage.parallel(k);
  storage.vectorize(i, 16);
}

Deconvolution::Deconvolution(string layer_name, 
                             Layer *prev,
                             const ConvolutionParameter *param,
                             const BlobProto *kernel_blob, 
                             const BlobProto *bias_blob) 
  :Layer(layer_name, DECONVOLUTION) 
{
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  /* stride is repeated field so we just need the first one.
   * Assume no padding */
  if (param->stride_size()) 
      stride = param->stride(0);
  bias = Image<float>(1, 1, num_output);
  if (bias_blob)
    bias = LoadBiasFromBlob(bias_blob, num_output);
  kernel = LoadKernelFromBlob(kernel_blob, kernel_size, num_output);

  int input_width = prev->get_width();
  int input_height = prev->get_height();
  int input_channels = prev->get_channels();
  //int batch_size = prev->get_num();
  int batch_size = prev->get_batchsize();

  /* Output dimension */
  int output_width     = kernel_size + (input_width - 1) * stride;
  int output_height    = kernel_size + (input_height - 1) * stride;
  int output_channels  = num_output;
  //int batch_size = input_num;

  /* Set output dimension */
  set_output_dim(output_width, output_height, output_channels, batch_size);

  /* Clamped at boundary */
  Func clamped = BoundaryConditions::constant_exterior(
      prev->storage, 0.f, 0, input_width, 0, input_height);

  /* Reduce over */
  int kernel_step = kernel_size / stride;
  RDom r(0, kernel_step, 0, kernel_step, 0, input_channels);

  storage(i, j, k, l) = sum(kernel(r.x*stride + i%stride,
                                   r.y*stride + j%stride,
                                   r.z, k) *
                            prev->storage(i/stride - r.x,
                                          j/stride - r.y,
                                          r.z, l));

  /* Scheduling */
  storage.parallel(k);
  int split_num = output_height > 15 ? output_height / 15 : 8;
  int vector_size = (output_width >= 16) ? 16 : 8;
  Var jo, ji;
  storage.split(j, jo, ji, split_num).parallel(jo);
  storage.vectorize(i, vector_size);
  clamped.store_at(storage, jo).compute_at(storage, ji);
}

} /* namespace latte */
