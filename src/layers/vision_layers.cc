#include <iostream>
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
                         const ConvolutionParameter *param, 
                         const BlobProto *weights, 
                         const BlobProto *bias_blob) 
{
  set_name(layer_name);
  set_type("Convolution");
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->stride_size()) 
    /* stride is repeated field so we just need the first one. Same goes for
     * pad data */
    stride = param->stride(0);
  if (param->pad_size())
    pad = param->pad(0);

  /* Default 0 bias */
  bias = Image<float>(1, 1, num_output);
  if (bias_blob)
    bias = LoadBiasFromBlob(bias_blob, num_output);

  kernel = LoadKernelFromBlob(weights, kernel_size, num_output);
}

Image<float>
Convolution::run(Image<float> input) 
{
  Func convolution("convolution");
  Var x("x"), y("y"), z("z");  
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
  
  /*
   * Different schedulings
   */

  /* CPU parallelism */
  convolution.parallel(z);

  Var x_outer, y_outer, x_inner, y_inner, tile_index;
  convolution.tile(x, y, x_outer, y_outer, x_inner, y_inner, 8, 8)
             .fuse(x_outer, y_outer, tile_index)
             .parallel(tile_index);

  Var x_inner_outer, y_inner_outer, x_vectors, y_pairs;
  convolution.tile(x_inner, y_inner, x_inner_outer, y_inner_outer, x_vectors, y_pairs, 4, 2)
             .vectorize(x_vectors)
             .unroll(y_pairs);

  Image<float> output = convolution.realize(width, height, num_output);

  /* OpenCL */
  /*
  Image<float> output(width, height, num_output);
  convolution.gpu_tile(x, y, z, 4, 4, 32);
  Target target = get_host_target();
  target.set_feature(Target::OpenCL);
  convolution.compile_jit(target);
  convolution.realize(output);
  */


  /* CUDA */
  /*
  Image<float> output(width, height, num_output);
  convolution.gpu_tile(x, y, z, 4, 4, 32);
  Target target = get_host_target();
  target.set_feature(Target::CUDA);
  convolution.compile_jit(target);
  convolution.realize(output);
  */

  return output;
}

/*****************************************************************************
 *****************************************************************************/
Pooling::Pooling(string layer_name, const PoolingParameter *param) 
{
  set_name(layer_name);
  set_type("Pooling");
  if (param->has_kernel_size())
    kernel_size = param->kernel_size();
  if (param->has_stride())
    stride = param->stride();
}

Image<float>
Pooling::run(Image<float> input) 
{
  Func pooled;
  Var x, y, z;
  int width    = (input.width() - kernel_size) / stride + 1;
  int height   = (input.height() - kernel_size) / stride + 1;
  int channels = input.channels();

  /* 2D reduction for each channel */
  RDom r(0, kernel_size, 0, kernel_size);
  pooled(x, y, z) = maximum(input(x*stride + r.x, y*stride + r.y, z));

  /* CPU parallelism */
  /*
  pooled.parallel(z);

  Var x_outer, y_outer, x_inner, y_inner, tile_index;
  pooled.tile(x, y, x_outer, y_outer, x_inner, y_inner, 8, 8)
        .fuse(x_outer, y_outer, tile_index)
        .parallel(tile_index);

  Var x_inner_outer, y_inner_outer, x_vectors, y_pairs;
  pooled.tile(x_inner, y_inner, x_inner_outer, y_inner_outer, x_vectors, y_pairs, 4, 2)
        .vectorize(x_vectors)
        .unroll(y_pairs);
  */

  Image<float> output = pooled.realize(width, height, channels);
  return output;
}

/*****************************************************************************
 *****************************************************************************/
Deconvolution::Deconvolution(string layer_name, 
                             const ConvolutionParameter *param,
                             const BlobProto *kernel_blob, 
                             const BlobProto *bias_blob) 
{
  set_name(layer_name);
  set_type("Deconvolution");

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
}

#if 0
// We have not figured out how to use protobuf
// WARNING: This implementation assumes no padding
Image<float> 
Deconvolution::run(Image<float> input) {
  Func deconvolution("deconv");
  Expr x1L("x1L"), y1L("y1L"), x1R("x1R"), y1R("y1R");
  Var x2("x2"), y2("y2"), z("z");
  // Compute output dimension
  int width     = kernel_size + (input.width() - 1) * stride;
  int height    = kernel_size + (input.height() - 1) * stride;
  int channels  = input.channels();

  deconvolution.trace_stores();

  // Compute reduction domain
  x1L = (Halide::max(x2 - kernel_size + 1, 0) + stride - 1) / stride;
  y1L = (Halide::max(y2 - kernel_size + 1, 0) + stride - 1) / stride;
  x1R = x2 / stride;
  y1R = y2 / stride;
  cout << "create RDom r" << endl;  
  RDom r(0, x1R - x1L + 1, 0, y1R - y1L + 1, 0, channels);

  // Compute deconvolution
  cout << "create deconv" << endl;
  deconvolution(x2, y2, z) = sum(
      kernel(x2 - (x1L + r.x) * stride, y2 - (y1L + r.y) * stride, z*channels + r.z) *
      input(x1L + r.x, y1L + r.y, r.z)) + bias(r.z);

  cout << "realize" << endl;
  /* TODO: define schedule */
  Image<float> output = deconvolution.realize(width, height, num_output);
  return output;
}
#endif

Image<float>
Deconvolution::run(Image<float> input)
{
  int input_width  = input.width();
  int input_height = input.height();
  int input_depth  = input.channels();

  cout << "::: Deconv General Info [start] :::" << endl;
  cout << "input_width  = " << input_width << endl;
  cout << "input_height = " << input_height << endl;
  cout << "input_depth  = " << input_depth << endl;

  /* Compute output dimension */
  int width     = kernel_size + (input_width - 1) * stride;
  int height    = kernel_size + (input_height - 1) * stride;
  Image<float> output(width, height, num_output);

  cout << "output_width  = " << width << endl;
  cout << "output_height = " << height << endl;
  cout << "output_depth  = " << num_output << endl;

  cout << "kernel_size   = " << kernel_size << endl;
  cout << "stride        = " << stride << endl;
  cout << "::: Deconv General Info [end] :::" << endl;


  /* Recode */
  cout << "::: Compute channels [start] :::" << endl;
#pragma omp parallel for
  for (int z = 0; z < num_output; z++) {
    cout << "start computing channel " << z << flush << endl;
    for (int j = 0; j < input_height; j++) {
      for (int j_f = 0; j_f < stride; j_f++) {
        for (int i = 0; i < input_width; i++) {
          for (int i_f = 0; i_f < stride; i_f++) {
            int j_out = j * stride + j_f;
            int i_out = i * stride + i_f;
            for (int j_k = 0; j_k < kernel_size; j_k++) {
              for (int i_k = 0; i_k < kernel_size; i_k++) {
                for (int z_k = 0; z_k < input_depth; z_k++) {
                  output(i_out + i_k, j_out + j_k, z) += 
                  input(i, j, z_k) * kernel(i_k, j_k, z * num_output + z_k);
                }
              }
            }
          }
        }
      }
    }
    cout << "finish computing channel " << z << endl;
  }
  cout << "::: Compute channels [end] :::" << endl;

  // /* This version is not right */
  // /* For each input layer */
  // for (int z = 0; z < input_depth; z++) {
  //   /* Loop over all input pixels, step by stride */
  //   for (int j = 0; j < input_height; j++) {
  //     for (int i = 0; i < input_width; i++) {

  //      	int i_out = i*stride;
  //     	int j_out = j*stride;
  //      	float input_val = input(i, j, z);
        
  //       /* dot with kernel and accumulate values into output */
  //       for (int z_k = 0; z_k < num_output; z_k++) {
  //         for (int j_k = 0; j_k < kernel_size; j_k++) {
  //           for (int i_k = 0; i_k < kernel_size; i_k++) {
  //             output(i_out + i_k, j_out + j_k, z_k) += 
  //               input_val * kernel(i_k, j_k, z_k + z*num_output);
  //           } /* i_k */
  //         } /* j_k */
  //       } /* z_k */
        
       
  //     } /* j */
  //   } /* i */
  // } /* each input layer */
  
  return output;
}

} /* namespace latte */
