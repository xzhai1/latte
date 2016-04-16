#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "layers/common_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

Crop::Crop(string layer_name, const CropParameter *param)
{
  set_name(layer_name);
  set_type("Crop");
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
Crop::run(Image<float> input)
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
Dropout::Dropout(string layer_name, const DropoutParameter *param) 
{
  set_name(layer_name);
  set_type("Dropout");
  if (param->has_dropout_ratio())
    dropout_ratio = param->dropout_ratio();
}

Image<float> 
Dropout::run(Image<float> input) 
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

  /* CPU parallelism */
  /*
  dropped.parallel(z);

  Var x_outer, y_outer, x_inner, y_inner, tile_index;
  dropped.tile(x, y, x_outer, y_outer, x_inner, y_inner, 8, 8)
         .fuse(x_outer, y_outer, tile_index)
         .parallel(tile_index);

  Var x_inner_outer, y_inner_outer, x_vectors, y_pairs;
  dropped.tile(x_inner, y_inner, x_inner_outer, y_inner_outer, x_vectors, y_pairs, 4, 2)
         .vectorize(x_vectors)
         .unroll(y_pairs);
  */

  Image<float> output = dropped.realize(width, height, channels);

  return output;
}

/*****************************************************************************
 *****************************************************************************/
Split::Split(string layer_name) {
  set_name(layer_name);
  set_type("Split");
}

Image<float> 
Split::run(Image<float> input) 
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
Silence::Silence(string layer_name) 
{
  set_name(layer_name);
  set_type("Silence");
}

Image<float> 
Silence::run(Image<float> input) 
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
