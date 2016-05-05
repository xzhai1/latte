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

/*************************** Convolution Layer ***************************/
Convolution::Convolution(string layer_name,
                         Layer *prev,
                         const ConvolutionParameter *param, 
                         const BlobProto *weights, 
                         const BlobProto *bias_blob)
  :Layer(layer_name, CONVOLUTION) 
{
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->stride_size()) 
    /* stride is repeated field so we just need the first one. Same goes for
     * pad data */
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
  int input_num = prev->get_num();

  /* Output dimension */
  int output_width     = (input_width  - kernel_size + 2 * pad) / stride + 1;
  int output_height    = (input_height - kernel_size + 2 * pad) / stride + 1;
  int output_channels  = num_output;
  int output_num       = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

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
  //storage.trace_store(); 
  // #if 0
  storage.compute_root();
  /* Version 2 */
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

  
  // #endif
}

Func 
Convolution::run(
    Func input, int input_width, int input_height, int input_channels, int input_num) 
{
  /* Output dimension */
  int output_width     = (input_width  - kernel_size + 2 * pad) / stride + 1;
  int output_height    = (input_height - kernel_size + 2 * pad) / stride + 1;
  int output_channels  = num_output;
  int output_num       = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  /* Clamped at boundary */
  Func clamped = BoundaryConditions::constant_exterior(
      input, 0.f, 0, input_width, 0, input_height);

  /* Reduce over one kernel */
  RDom r(0, kernel_size, 0, kernel_size, 0, input_channels);

  storage(i, j, k, l) = sum(kernel(r.x, r.y, r.z, k) * 
                     clamped(i * stride + r.x - pad, j * stride + r.y - pad, r.z, l))
                     + bias(0, 0, 0, k);
  /* and add bias */
  //storage(i, j, k, l) += bias(0, 0, 0, k);

  /* Schedule */
  /* CPU parallel */
#if 0
  storage.parallel(k);

  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, 8, 8)
             .fuse(i_outer, j_outer, tile_index)
             .parallel(tile_index);

  Var i_inner_outer, j_inner_outer, i_vectors, j_pairs;
  storage.tile(i_inner, j_inner, 
               i_inner_outer, j_inner_outer, 
               i_vectors, j_pairs, 8, 2)
             .vectorize(i_vectors)
             .unroll(j_pairs);
#endif
  /* GPU parallel */
  // storage.gpu_tile(i, j, k, 4, 4, 32);

  /* Version 2 */
  storage.parallel(k);

  int split_num = output_height > 15 ? output_height / 15 : 8;
  int vector_size = (output_width >= 16) ? 16 : 8;
  Var jo, ji;
  storage.split(j, jo, ji, split_num).parallel(jo);
  storage.vectorize(i, vector_size);
  clamped.store_at(storage, jo).compute_at(storage, ji);

  //storage.print_loop_nest();
 
  return storage;
}

/*************************** Pooling Layer ***************************/
Pooling::Pooling(
    string layer_name, 
    Layer *prev, 
    const PoolingParameter *param) 
  :Layer(layer_name, POOLING) 
{
  //set_name(layer_name);
  //set_type("Pooling");
  if (param->has_kernel_size())
    kernel_size = param->kernel_size();
  if (param->has_stride())
    stride = param->stride();

  int input_width = prev->get_width();
  int input_height = prev->get_height();
  int input_channels = prev->get_channels();
  int input_num = prev->get_num();

  /* Compute output dimension */
  int output_width    = (input_width - kernel_size) / stride + 1;
  int output_height   = (input_height - kernel_size) / stride + 1;
  int output_channels = input_channels;
  int output_num      = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  /* 2D reduction for each channel */
  RDom r(0, kernel_size, 0, kernel_size);
  storage(i, j, k, l) = maximum(
      prev->storage(i*stride + r.x, j*stride + r.y, k, l));

  /* CPU parallel */
  storage.compute_root();
  storage.parallel(k);
  storage.vectorize(i, 16);

  #if 0
  int vector_size = (output_width >= 16) ? 16 : 8;
  if (output_width * 3 <= output_channels) {
    Var ko, ki, fused1, fused2;
    storage.split(k, ko, ki, 4);
    storage.parallel(ko);
    storage.fuse(i, j, fused1).fuse(fused1, ki, fused2);
    storage.vectorize(fused2, vector_size);
  } else {
    storage.parallel(k).vectorize(i, vector_size);
  }
  #endif
#if 0
  int tile_size = kernel_size * 8;
  storage.parallel(k);

  /* CPU parallel */
  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, tile_size, tile_size)
         .fuse(i_outer, j_outer, tile_index)
         .parallel(tile_index);
  storage.vectorize(i_inner, 8);
#endif
}

Halide::Func Pooling::run(
  Halide::Func input, int input_width, int input_height, int input_channels, int input_num) 
{
  /* Compute output dimension */
  int output_width    = (input_width - kernel_size) / stride + 1;
  int output_height   = (input_height - kernel_size) / stride + 1;
  int output_channels = input_channels;
  int output_num      = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  /* 2D reduction for each channel */
  RDom r(0, kernel_size, 0, kernel_size);
  storage(i, j, k, l) = maximum(input(i * stride + r.x, j * stride + r.y, k, l));

  storage.compute_root();
  storage.parallel(k);
  storage.vectorize(i, 16);

#if 0
  storage.parallel(k);

  int split_num = output_height > 15 ? output_height / 15 : 8;
  int vector_size = (output_width >= 16) ? 16 : 8;
  Var jo, ji;
  storage.split(j, jo, ji, 32).parallel(jo);
  storage.vectorize(i, 16);
#endif

  int tile_size = kernel_size * 8;
  storage.parallel(k);
  /* CPU parallel */
  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, tile_size, tile_size)
         .fuse(i_outer, j_outer, tile_index)
         .parallel(tile_index);
  storage.vectorize(i_inner, 8);

  /* GPU parallel */
  // storage.gpu_tile(i, j, k, 4, 4, 32);
  //storage.print_loop_nest(); 
  return storage;
}

/*************************** Deconvolution Layer ***************************/
Deconvolution::Deconvolution(string layer_name, 
                             Layer *prev,
                             const ConvolutionParameter *param,
                             const BlobProto *kernel_blob, 
                             const BlobProto *bias_blob) 
  :Layer(layer_name, DECONVOLUTION) 
{

  // cout << "In constructor of deconv" << endl;
  // cout << "kernel" << endl;

  kernel_size = param->kernel_size(0);
  
  num_output = param->num_output();
  // cout << "num_output = " << num_output << endl;
  // cout << "stride" << endl;
  if (param->stride_size()) 
    /* stride is repeated field so we just need the first one.
     * Assume no padding */
    stride = param->stride(0);
  // cout << "bias" << endl;
  bias = Image<float>(1, 1, num_output);
  if (bias_blob)
    bias = LoadBiasFromBlob(bias_blob, num_output);
  // cout << "kernel" << endl;
  kernel = LoadKernelFromBlob(kernel_blob, kernel_size, num_output);

  int input_width = prev->get_width();
  int input_height = prev->get_height();
  int input_channels = prev->get_channels();
  int input_num = prev->get_num();

  /* Output dimension */
  int output_width     = kernel_size + (input_width - 1) * stride;
  int output_height    = kernel_size + (input_height - 1) * stride;
  int output_channels  = num_output;
  int output_num       = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

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

  /* CPU parallel */
  storage.parallel(k);
  int split_num = output_height > 15 ? output_height / 15 : 8;
  int vector_size = (output_width >= 16) ? 16 : 8;
  Var jo, ji;
  storage.split(j, jo, ji, split_num).parallel(jo);
  storage.vectorize(i, vector_size);
  clamped.store_at(storage, jo).compute_at(storage, ji);
}

Func Deconvolution::run(Func input, int input_width, int input_height, int input_channels, int input_num) {
  /* Output dimension */
  int output_width     = kernel_size + (input_width - 1) * stride;
  int output_height    = kernel_size + (input_height - 1) * stride;
  int output_channels  = num_output;
  int output_num       = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  /* Clamped at boundary */
  Func clamped = BoundaryConditions::constant_exterior(input, 0.f, 0, input_width, 0, input_height);

  /* Reduce over */
  int kernel_step = kernel_size / stride;
  RDom r(0, kernel_step, 0, kernel_step, 0, input_channels);

  storage(i, j, k, l) = sum(kernel(r.x * stride + i % stride,
                                   r.y * stride + j % stride,
                                   r.z, k) *
                            input (i / stride - r.x,
                                   j / stride - r.y,
                                   r.z, l));


  /* CPU parallel */
  storage.parallel(k);
  int split_num = output_height > 15 ? output_height / 15 : 8;
  int vector_size = (output_width >= 16) ? 16 : 8;
  Var jo, ji;
  storage.split(j, jo, ji, split_num).parallel(jo);
  storage.vectorize(i, vector_size);
  clamped.store_at(storage, jo).compute_at(storage, ji);

#if 0
  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, 8, 8)
             .fuse(i_outer, j_outer, tile_index)
             .parallel(tile_index);
             
  Var i_inner_outer, j_inner_outer, i_vectors, j_pairs;
  storage.tile(i_inner, j_inner, i_inner_outer, j_inner_outer, i_vectors, j_pairs, 4, 2)
             .vectorize(i_vectors)
             .unroll(j_pairs);
#endif
  // storage.gpu_tile(i, j, k, 8, 8, 8);

  return storage;
}

} /* namespace latte */
