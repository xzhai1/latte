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

/*************************** Crop Layer ***************************/
Crop::Crop(string layer_name, const CropParameter *param)
{
  set_name(layer_name);
  set_type("Crop");
  if (param->offset_size() == 1) {
    offset_i = param->offset(0);
    offset_j = param->offset(0);
  } else {
    // axis in caffe: (N,C,H,W)
    offset_i = param->offset(1);
    offset_j = param->offset(0);
  }
}

Func Crop::run(
  Func input, int input_width, int input_height, int input_channels, int input_num)
{
  /* Output dimension */
  int output_width    = input_width - 2 * offset_i;
  int output_height   = input_height - 2 * offset_j;
  int output_channels = input_channels;
  int output_num      = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  storage(i, j, k, l) = input(i + offset_i, j + offset_j, k, l);

  return storage;
}

/*************************** Dropout Layer ***************************/
Dropout::Dropout(string layer_name, const DropoutParameter *param) 
{
  set_name(layer_name);
  set_type("Dropout");
  if (param->has_dropout_ratio())
    dropout_ratio = param->dropout_ratio();
}

#if 0
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
#endif

Func Dropout::run(
  Func input, int input_width, int input_height, int input_channels, int input_num, bool eval) 
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

  if (eval) { /* Test */
    storage(i, j, k, l) = input(i, j, k, l);
  } else {    /* Train */
    storage(i, j, k, l) = input(i, j, k, l); /* Need random dropout */
  }

  return storage; 
}

/*************************** Split Layer ***************************/
Split::Split(string layer_name) {
  set_name(layer_name);
  set_type("Split");
}

#if 0
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
#endif

Func Split::run(
  Func input, int input_width, int input_height, int input_channels, int input_num, bool eval) 
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

  if (eval) { /* Test */
    storage(i, j, k, l) = input(i, j, k, l);
  } else {    /* Train */
    storage(i, j, k, l) = input(i, j, k, l); /* Need to split when necessary */
  }

  return storage; 
}

/*************************** Silent Layer ***************************/
Silence::Silence(string layer_name) 
{
  set_name(layer_name);
  set_type("Silence");
}

#if 0
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
#endif

Func Silence::run(
  Func input, int input_width, int input_height, int input_channels, int input_num, bool eval) 
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

  if (eval) { /* Test */
    storage(i, j, k, l) = input(i, j, k, l);
  } else {    /* Train */
    storage(i, j, k, l) = input(i, j, k, l); /* Need to do something when necessary */
  }

  return storage; 
}

} /* namespace latte */
