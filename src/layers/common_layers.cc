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
  :Layer(layer_name, CROP)
{
  if (param->offset_size() == 1) {
    offset_i = param->offset(0);
    offset_j = param->offset(0);
  } else {
    // axis in caffe: (N,C,H,W)
    offset_i = param->offset(1);
    offset_j = param->offset(0);
  }
}

Func 
Crop::run(
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

Dropout::Dropout(string layer_name, const DropoutParameter *param) 
  :Layer(layer_name, DROPOUT)
{
  if (param->has_dropout_ratio())
    dropout_ratio = param->dropout_ratio();
}

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

Split::Split(string layer_name)
  :Layer(layer_name, SPLIT) {}

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

Silence::Silence(string layer_name)
  :Layer(layer_name, SILENCE) {}

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
