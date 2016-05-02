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
  bias = Image<float>(1, 1, 1, num_output);
  if (bias_blob)
    bias = LoadBiasFromBlob(bias_blob, num_output);

  kernel = LoadKernelFromBlob(weights, kernel_size, num_output);
}

/* Serial version of Convolution only supports convolution for one image */
Image<float>
Convolution::SerialConv(Image<float> input)
{
  /* Input dimension */
  int input_width     = input.extent(0);
  int input_height    = input.extent(1);
  int input_channels  = input.extent(2);
  int input_num       = input.extent(3);

  /* Output dimension */
  int output_width    = get_width();
  int output_height   = get_height();
  int output_channels = get_channels();
  int output_num      = get_num();

  Image<float> output(output_width, output_height, output_channels, output_num);

  /* Fill in the bias first */
  /* For each output in the batch */
  for (int w = 0; w < output_num; w++) {
    /* For each channel of the output/kernel */
    for (int z = 0; z < output_channels; z++) {
      float bias_val = bias(0, 0, 0, z);
      for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {
          output(x, y, z, w) = bias_val;
        }
      }
    }
  }

  /* Accumulate convolved values */
  /* For each image in batch */
  for (int w = 0; w < output_num; w++) {
    /* For each channel of the output/kernel */
    for (int z = 0; z < output_channels; z++) {
      /* slide the kernel around */
      for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {
          /* and reduce over its domain */
          float partial_sum = 0.f;
          for (int zz = 0; zz < input_channels; zz++) {
            for (int yy = 0; yy < kernel_size; yy++) {
              for (int xx = 0; xx < kernel_size; xx++) {
                int x_input = x * stride + xx - pad;
                int y_input = y * stride + yy - pad;
                float kernel_val = kernel(xx, yy, zz, z);
                float val_input;
                if (x_input < 0 || x_input > input_width - 1 ||
                    y_input < 0 || y_input > input_height -1) 
                  val_input = 0.f;
                else
                  val_input = input(x_input, y_input, zz, w);

                partial_sum += kernel_val * val_input;
              }
            }
          }

          /* Save to output position */
          output(x, y, z, w) += partial_sum;
        }
      }
    }
  }

#if 0
  /* For each kernel */
  for (int f_idx = 0; f_idx < num_output; f_idx++) {
    /* slide the kernel around */
    for (int y = 0; y < out_height; y++) {
      for (int x = 0; x < out_width; x++) {
        /* and reduce over its domain */
        float partial_sum = 0.f;
        for (int c = 0; c < img_channels; c++) {
          for (int k_y = 0; k_y < kernel_size; k_y++) {
            for (int k_x = 0; k_x < kernel_size; k_x++) {
              int x_input = x*stride + k_x - pad;
              int y_input = y*stride + k_y - pad;
              float k_val = kernel(k_x, k_y, c, f_idx);
              float val_input = 0.f;
              /* Check for out of bound */
              if (x_input < 0 || x_input > img_width - 1||
                  y_input < 0 || y_input > img_height -1)
                val_input = 0.f;
              else {
                val_input = input(x_input, y_input, c, 1);
              }
              partial_sum += k_val*val_input;
            }
          }
        }
        /* Save to output position */
        output(x, y, f_idx, 0) += partial_sum;
      }
    }
  }
#endif

  return output;
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
                     clamped(i * stride + r.x - pad, j * stride + r.y - pad, r.z, l));

  /* and add bias */
  storage(i, j, k, l) += bias(0, 0, 0, k);

  /* Schedule */
  /* CPU parallel */
  storage.parallel(k);

  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, 8, 8)
             .fuse(i_outer, j_outer, tile_index)
             .parallel(tile_index);

  Var i_inner_outer, j_inner_outer, i_vectors, j_pairs;
  storage.tile(i_inner, j_inner, 
               i_inner_outer, j_inner_outer, 
               i_vectors, j_pairs, 4, 2)
             .vectorize(i_vectors)
             .unroll(j_pairs);

  /* GPU parallel */
  //storage.gpu_tile(i, j, k, 4, 4, 32);
  
  return storage;
}

/*************************** Pooling Layer ***************************/
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
Pooling::SerialPool(Image<float> input) 
{
  /* Input dimension */
  int input_width     = input.extent(0);
  int input_height    = input.extent(1);
  int input_channels  = input.extent(2);
  int input_num       = input.extent(3);

  /* Output dimension */
  int output_width    = get_width();
  int output_height   = get_height();
  int output_channels = get_channels();
  int output_num      = get_num();

  Image<float> output(output_width, output_height, output_channels, output_num);

  /* For each output in the batch */
  for (int w = 0; w < output_num; w++) {
    /* For each channel of the output/kernel */
    for (int z = 0; z < output_channels; z++) {
      for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {
          /* and reduce over its domain */
          float max_val = 0.f;
          for (int yy = 0; yy < kernel_size; yy++) {
            for (int xx = 0; xx < kernel_size; xx++) {
              int x_input = x * stride + xx;
              int y_input = y * stride + yy;
              float val_input = input(x_input, y_input, z, w);
              if (val_input > max_val) {
                max_val = val_input;
              }
            }
          }
          /* Save to output position */
          output(x, y, z, w) = max_val;
        }
      }
    }
  }

  return output;
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
#if 0
  /* CPU parallel */
  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, 8, 8)
         .fuse(i_outer, j_outer, tile_index)
         .parallel(tile_index);
  storage.vectorize(i_inner, 8);
#endif
  
  return storage;
}

/*************************** Deconvolution Layer ***************************/
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
  /* output first layer of kernel */
  cout << "number of kernels = " << kernel.extent(3) << endl;
  for (int k = 0; k < kernel.extent(2); k++) {
    ofstream outfile("./outputs/kernel" + to_string(k) + ".txt");
    for (int j = 0; j < kernel.extent(1); j++) {
      for (int i = 0; i < kernel.extent(0); i++) {
        outfile << kernel(i, j, k, 2) << " ";
      }
      outfile << endl;
    }
    outfile.close();
  }

  int input_width    = input.extent(0);
  int input_height   = input.extent(1);
  int input_channels = input.extent(2);
  int input_num      = input.extent(3);

  cout << "::: Deconv General Info [start] :::" << endl;
  cout << "input_width    = " << input_width << endl;
  cout << "input_height   = " << input_height << endl;
  cout << "input_channels = " << input_channels << endl;
  cout << "input_num      = " << input_num << endl;

  /* Compute output dimension */
  int output_width     = kernel_size + (input_width - 1) * stride;
  int output_height    = kernel_size + (input_height - 1) * stride;
  int output_channels  = num_output;
  int output_num       = input_num;
  Image<float> output(output_width, output_height, num_output, output_num);

  cout << "output_width    = " << output_width << endl;
  cout << "output_height   = " << output_height << endl;
  cout << "output_channels = " << output_channels << endl;
  cout << "output_num      = " << output_num << endl;

  cout << "kernel_size     = " << kernel_size << endl;
  cout << "stride          = " << stride << endl;
  cout << "::: Deconv General Info [end] :::" << endl;

  cout << "output deconv kernel" << endl;

  int kernel_step = kernel_size / stride;

  for (int w = 0; w < output_num; w++) {
    #pragma omp parallel for
    for (int z = 0; z < output_channels; z++) {
      for (int y = 0; y < output_height; y++) {
        for (int x = 0; x < output_width; x++) {

          float partial_sum = 0.f;
          for (int zz = 0; zz < input_channels; zz++) {
            for (int yy = 0; yy < kernel_step; yy++) {
              for (int xx = 0; xx < kernel_step; xx++) {
                if (x / stride - xx >= 0 &&
                    x / stride - xx <  input_width &&
                    y / stride - yy >= 0 &&
                    y / stride - yy <  input_height) {
                  partial_sum += kernel(xx * stride + x % stride,
                                        yy * stride + y % stride,
                                        zz, z) *
                                 input (x / stride - xx,
                                        y / stride - yy,
                                        zz, w);
                }
              }
            }
          }
          output(x, y, z, w) = partial_sum;

        }
      }
    }
  }

  #if 0
  cout << "::: Compute channels [start] :::" << endl;
  #pragma omp parallel for
  for (int z = 0; z < num_output; z++) {
    cout << "start computing channel " << z << flush << endl;
    for (int j = 0; j < height; j++) {
      for (int i = 0; i < width; i++) {
        int x1L = (max(i - kernel_size + 1, 0) + stride - 1) / stride;
        int y1L = (max(j - kernel_size + 1, 0) + stride - 1) / stride;
        int x1R = i / stride;
        int y1R = j / stride;
        for (int y1 = y1L; y1 < y1R + 1; y1++) {
          for (int x1 = x1L; x1 < x1R + 1; x1++) {
            for (int z1 = 0; z1 < input_depth; z1++) {
              int x2 = i - x1 * stride;
              int y2 = j - y1 * stride;
              output(i, j, z) += kernel(x2, y2, z1*num_output + z) * input(x1, y1, z1);
            }
          }
        }
      }
    }
    cout << "finish computing channel " << z << flush << endl;
  }
  cout << "::: Compute channels [end] :::" << endl;
  #endif

  #if 0
  //int kernel_dim = kernel_size * kernel_size * num_output;
  float curr_sum;
  /* Recode */
  cout << "::: Compute channels [start] :::" << endl;
  for (int w = 0; w < output_num; w++) {
    #pragma omp parallel for
    for (int j = 0; j < input_height; j++) {
      cout << "j = " << j << endl;
      for (int i = 0; i < input_width; i++) {
        for (int z_k = 0; z_k < output_channels; z_k++) {
          for (int j_k = 0; j_k < kernel_size; j_k++) {
            for (int i_k = 0; i_k < kernel_size; i_k++) {
              /* Compute current sum */
              curr_sum = 0.f;
              for (int c = 0; c < input_channels; c++) {
                curr_sum += input(i, j, c, w) * 
                            kernel(i_k, j_k, c, z_k);
              }
              /* Accumulate sum */
              int col = i*stride + i_k;
              int row = j*stride + j_k;
              output(col, row, z_k, w) += curr_sum;
            }
          }
        }
      }
    }
  }
  #endif
  

  #if 0
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
                    input(i, j, z_k) * 
                    kernel(i_k, j_k, z * num_output + z_k);
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
  #endif


  return output;
}

Func Deconvolution::run(Func input, int input_width, int input_height, int input_channels, int input_num) {
  /* Compute output dimension */
  int output_width     = input_width * stride; /* Assume stride == upsampling factor */
  int output_height    = input_height * stride;
  int output_channels  = num_output;
  int output_num       = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  /* Clamped at boundary */
  Func clamped = BoundaryConditions::constant_exterior(input, 0.f, 0, input_width, 0, input_height);

  //Func clamped = BoundaryConditions::constant_exterior(input, 0.f);

  /* Reduce over kernel */
  RDom r(0, kernel_size, 0, kernel_size, 0, input_channels);
  storage(i, j, k, l) = sum(
      kernel(r.x, r.y, r.z, k) * 
      clamped(i / stride + r.x - kernel_size / 2 , j / stride + r.y - kernel_size / 2 , r.z, l));

  storage.parallel(k);

  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, 8, 8)
             .fuse(i_outer, j_outer, tile_index)
             .parallel(tile_index);
             
  Var i_inner_outer, j_inner_outer, i_vectors, j_pairs;
  storage.tile(i_inner, j_inner, i_inner_outer, j_inner_outer, i_vectors, j_pairs, 4, 2)
             .vectorize(i_vectors)
             .unroll(j_pairs);

  // storage.gpu_tile(i, j, k, 8, 8, 8);

  return storage;
}

} /* namespace latte */
