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

/*************************** ReLU Layer ***************************/
ReLU::ReLU(string layer_name, const ReLUParameter *param) 
{
  set_name(layer_name);
  set_type("ReLU");
  if (param->has_negative_slope())
    negative_slope = param->negative_slope();
}

#if 0
Image<float> 
ReLU::run(Image<float> input)
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
  /*
  rectified.parallel(z);

  Var x_outer, y_outer, x_inner, y_inner, tile_index;
  rectified.tile(x, y, x_outer, y_outer, x_inner, y_inner, 8, 8)
           .fuse(x_outer, y_outer, tile_index)
           .parallel(tile_index);

  Var x_inner_outer, y_inner_outer, x_vectors, y_pairs;
  rectified.tile(x_inner, y_inner, x_inner_outer, y_inner_outer, x_vectors, y_pairs, 4, 2)
           .vectorize(x_vectors)
           .unroll(y_pairs);
  */

  Image<float> output = rectified.realize(width, height, channels);
  return output;
}
#endif

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
#if 0
  /* CPU parallel */
  storage.parallel(z);

  Var x_outer, y_outer, x_inner, y_inner, tile_index;
  storage.tile(x, y, x_outer, y_outer, x_inner, y_inner, 8, 8)
           .fuse(x_outer, y_outer, tile_index)
           .parallel(tile_index);

  storage.vectorize(x_inner, 8);
#endif

  return storage;
}

/*************************** Sigmoid Layer ***************************/

/*************************** Tanh Layer ***************************/

} /* namespace latte */
