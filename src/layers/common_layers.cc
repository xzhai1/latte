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

Crop::Crop(
    string layer_name,
    Layer *prev,
    const CropParameter *param, int input_width, int input_height)
  :Layer(layer_name, CROP)
{
  offset_i = (prev->get_width() - input_width)/2;
  offset_j = (prev->get_height() - input_height)/2;
  SetOutputDim(prev);
  storage(i, j, k, l) = prev->storage(i + offset_i, j + offset_j, k, l);

  /* Schedule */
  /* Halide v5 scheduling policy */
  int vector_size = (get_width() >= 16) ? 16 : 8;
  Var fused;
  storage.compute_root();
  storage.fuse(k, l, fused);
  storage.parallel(fused);
  storage.vectorize(i, vector_size);

  /* GPU parallel */
#if 0 
  storage.compute_root();
  storage.gpu_tile(i, j, k, 8, 8, 16);
#endif
}

#if 0
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
#endif

Dropout::Dropout(string layer_name, const DropoutParameter *param) 
  :Layer(layer_name, DROPOUT)
{
  if (param->has_dropout_ratio())
    dropout_ratio = param->dropout_ratio();
}

#if 0
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
#endif

Split::Split(string layer_name)
  :Layer(layer_name, SPLIT) {}

#if 0
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
#endif

Silence::Silence(string layer_name)
  :Layer(layer_name, SILENCE) {}

#if 0
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
#endif

} /* namespace latte */
