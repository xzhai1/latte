#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "layers/activation_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

ReLU::ReLU(string layer_name, Layer *prev, const ReLUParameter *param) 
  :Layer(layer_name, RELU)
{
  if (param->has_negative_slope())
    negative_slope = param->negative_slope();

  int input_width = prev->get_width();
  int input_height = prev->get_height();
  int input_channels = prev->get_channels();
  int input_num = prev->get_num();
  
  /* Output dimension */
  int output_width    = input_width;
  int output_height   = input_height;
  int output_channels = input_channels;
  int output_num      = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  storage(i, j, k, l) = max(0, prev->storage(i, j, k, l)) + 
                        negative_slope * min(0, prev->storage(i, j, k, l));

  /* CPU parallel */
  storage.compute_root();

  int vector_size = (output_width >= 16) ? 16 : 8;
  if (output_width * 2 <= output_channels) {
    Var ko, ki, fused1, fused2;
    storage.split(k, ko, ki, 4);
    storage.parallel(ko);
    storage.fuse(i, j, fused1).fuse(fused1, ki, fused2);
    storage.vectorize(fused2, vector_size);
  } else {
    storage.parallel(k).vectorize(i, vector_size);
  }

  // int vector_size = (output_width >= 16) ? 16 : 8;
  // storage.parallel(k);
  // storage.vectorize(i, vector_size);

  // storage.parallel(k);
  // Var tile_index;
  // storage.fuse(i, j, tile_index).vectorize(tile_index, 16);
}

Func ReLU::run(
  Func input, int input_width, int input_height, int input_channels, int input_num) 
{
  /* Output dimension */
  int output_width    = input_width;
  int output_height   = input_height;
  int output_channels = input_channels;
  int output_num      = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  storage(i, j, k, l) = max(0, input(i, j, k, l)) + 
                       negative_slope * min(0, input(i, j, k, l));
  /* CPU parallel */
  //storage.vectorize(i, 8);
  int vector_size = (output_width >= 16) ? 16 : 8;
  storage.parallel(k).vectorize(i, vector_size);
#if 0
  Var i_outer, j_outer, i_inner, j_inner, tile_index;
  storage.tile(i, j, i_outer, j_outer, i_inner, j_inner, 8, 8)
           .fuse(i_outer, j_outer, tile_index)
           .parallel(tile_index);

  storage.vectorize(i_inner, 8);
#endif

  /* GPU parallel */
  // storage.gpu_tile(i, j, k, 4, 4, 32);

  //storage.print_loop_nest();

  return storage;
}

/*************************** Sigmoid Layer ***************************/

/*************************** Tanh Layer ***************************/

} /* namespace latte */
