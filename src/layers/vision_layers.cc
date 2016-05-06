#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "caffe.pb.h"

#include "layers/vision_layers.h"
#include "proto2img_utils.h" 

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

Convolution::Convolution(string layer_name,
                         Layer *prev,
                         const ConvolutionParameter *param, 
                         const BlobProto *weights, 
                         const BlobProto *bias_blob)
  :Layer(layer_name, CONVOLUTION) 
{
  /* Set member variables */
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->stride_size()) 
    stride = param->stride(0);
  if (param->pad_size())
    pad = param->pad(0);

  /* Load bias and weights */
  bias = Image<float>(1, 1, 1, num_output);
  if (bias_blob)
    bias = LoadBiasFromBlob(bias_blob, num_output);
  kernel = LoadKernelFromBlob(weights, kernel_size, num_output);

  /* Set layer output dimension */
  SetOutputDim(prev);

  /* Define algorithm */
  Func clamped = BoundaryConditions::constant_exterior(
      prev->storage, 0.f, 0, prev->get_width(), 0, prev->get_height());
  RDom r(0, kernel_size, 0, kernel_size, 0, prev->get_channels());
  storage(i, j, k, l) = 
    sum(kernel(r.x, r.y, r.z, k) * 
        clamped(i*stride + r.x - pad, j*stride + r.y - pad, r.z, l)) 
    + bias(0, 0, 0, k);

  /* Dynamic Scheduling */
  storage.compute_root();
  storage.parallel(k);
  int split_num = get_height() > 15 ? get_height() / 15 : 8;
  int vector_size = (get_width() >= 16) ? 16 : 8;
  if (get_width()*2 <= get_channels()) {
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
  /* Set member variables */
  if (param->has_kernel_size())
    kernel_size = param->kernel_size();
  if (param->has_stride())
    stride = param->stride();
  
  /* Set layer output dimension */
  SetOutputDim(prev);

  /* Define algorithm */
  RDom r(0, kernel_size, 0, kernel_size);
  storage(i, j, k, l) = 
    maximum(prev->storage(i*stride + r.x, j*stride + r.y, k, l));

  /* Schedule */
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
  /* Set member variables */
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->stride_size()) 
      stride = param->stride(0);
  bias = Image<float>(1, 1, num_output);
  if (bias_blob)
    bias = LoadBiasFromBlob(bias_blob, num_output);
  kernel = LoadKernelFromBlob(kernel_blob, kernel_size, num_output);

  /* Set layer output dimension */
  SetOutputDim(prev);

  /* Define algorithm */
  Func clamped = BoundaryConditions::constant_exterior(
      prev->storage, 0.f, 0, prev->get_width(), 0, prev->get_height());
  int kernel_step = kernel_size / stride;
  RDom r(0, kernel_step, 0, kernel_step, 0, prev->get_channels());
  storage(i, j, k, l) = 
    sum(kernel(r.x*stride + i%stride, r.y*stride + j%stride, r.z, k) * 
        prev->storage(i/stride - r.x, j/stride - r.y, r.z, l));

  /* Scheduling */
  storage.compute_root();
  storage.parallel(k);
  int split_num = get_height() > 15 ? get_height() / 15 : 8;
  int vector_size = (get_width() >= 16) ? 16 : 8;
  Var jo, ji;
  storage.split(j, jo, ji, split_num).parallel(jo);
  storage.vectorize(i, vector_size);
  clamped.store_at(storage, jo).compute_at(storage, ji);
}

} /* namespace latte */
