#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "common_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

Dropout::Dropout(string layer_name, const DropoutParameter *param) 
{
  name = layer_name;
  if (param->has_dropout_ratio())
    dropout_ratio = param->dropout_ratio();
}

Image<float> 
Dropout::dropout(Image<float> input) 
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

  /* TODO: define schedule */
  Image<float> output = dropped.realize(width, height, channels);

  return output;
}

/*****************************************************************************
 *****************************************************************************/
Split::Split(string layer_name) {
  name = layer_name;
}

Image<float> 
Split::split(Image<float> input) 
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
  name = layer_name;
}

Image<float> 
Silence::silence(Image<float> input) 
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
