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

Softmax::Softmax(string layer_name) {
  set_name(layer_name);
  set_type("Softmax");
}

Image<float> 
Softmax::run(Image<float> input) 
{
  Func prob;
  Func normalizer;
  Var x, y, z;
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  RDom r(0, channels);
  prob(x, y, z) = exp(input(x, y, z));
  normalizer(x, y, z) = sum(prob(x, y, z + r.x));
  prob(x, y, z) = prob(x, y, z)/normalizer(x, y, z);

  /* TODO: define schedule */
  Image<float> output = prob.realize(width, height, channels);
  return output;
}

/*****************************************************************************
 *****************************************************************************/

SoftmaxWithLoss::SoftmaxWithLoss(string layer_name) 
{
  set_name(layer_name);
  set_type("SoftmaxWithLoss");
}

Image<float> 
SoftmaxWithLoss::run(Image<float> input) 
{
  Func loss;
  Func prob;
  Func normalizer;
  Var x, y, z;
  int width    = input.width();
  int height   = input.height();
  int channels = input.channels();

  RDom r(0, width, 0, height, 0, channels);
  prob(x, y, z) = exp(input(x, y, z));
  normalizer(x, y, z) = sum(prob(x, y, z + r.x));
  prob(x, y, z) = prob(x, y, z)/normalizer(x, y, z);
  loss(x, y, z) = -sum(log(prob(x + r.x, y + r.y, z + r.z)));

  /* TODO: define schedule */
  Image<float> output = loss.realize(width, height, channels);
  return output;
}

} /* namespace latte */
