#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "layers/loss_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

Softmax::Softmax(string layer_name)
  :Layer(layer_name, SOFTMAX)
{
  //set_name(layer_name);
  //set_type("Softmax");
}

Func Softmax::run(
  Func input, int input_width, int input_height, int input_channels, int input_num)
{
  /* Output dimension */
  int output_width     = input_width;
  int output_height    = input_height;
  int output_channels  = input_channels;
  int output_num       = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  Func normalizer;
  RDom r(0, 1, 0, 1, 0, output_channels);
  storage(i, j, k, l) = Halide::exp(input(i, j, k, l));
  normalizer(i, j, l) = Halide::sum(storage(i, j, r.z, l));
  storage(i, j, k, l) = storage(i, j, k, l)/normalizer(i, j, l);

  /* TODO: define schedule */
  return storage;
}

SoftmaxWithLoss::SoftmaxWithLoss(string layer_name) 
  :Layer(layer_name, SOFTMAXWITHLOSS)
{
  //set_name(layer_name);
  //set_type("SoftmaxWithLoss");
}

Func SoftmaxWithLoss::run(
  Func input, int input_width, int input_height, int input_channels, int input_num)
{
  /* Output dimension */
  int output_width     = input_width;
  int output_height    = input_height;
  int output_channels  = input_channels;
  int output_num       = input_num;

  /* Set output dimension */
  set_width(output_width);
  set_height(output_height);
  set_channels(output_channels);
  set_num(output_num);

  Func normalizer;
  RDom r(0, 1, 0, 1, 0, output_channels);
  storage(i, j, k, l) = Halide::exp(input(i, j, k, l));
  normalizer(i, j, l) = Halide::sum(storage(i, j, r.z, l));
  storage(i, j, k, l) = storage(i, j, k, l)/normalizer(i, j, l);

  RDom s(0, output_width, 0, output_height, 0, output_channels, 0, output_num);
  // loss = sum(-log(storage(s.x, s.y, s.z, s.w)));

  /* TODO: define schedule */
  return storage;
}

} /* namespace latte */
