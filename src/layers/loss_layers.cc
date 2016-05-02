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

/*************************** Softmax Layer ***************************/
Softmax::Softmax(string layer_name) {
  set_name(layer_name);
  set_type("Softmax");
}

#if 0
Image<float> 
Softmax::run(Image<float> input) 
{
  #if 0
  Func prob;
  Func normalizer;
  Var x, y, z;
  int width     = input.width();
  int height    = input.height();
  int channels  = input.channels();

  cout << "create RDom" << endl;
  RDom r(0, 1, 0, 1, 0, channels);
  cout << "compute prob 0" << endl;
  prob(x, y, z) = Halide::exp(input(x, y, z));
  cout << "compute prob 1" << endl;
  normalizer(x, y) = Halide::sum(prob(x, y, r.z));
  cout << "compute prob 2" << endl;
  prob(x, y, z) = prob(x, y, z)/normalizer(x, y);
  normalizer.compute_at(prob, x);
  /* TODO: define schedule */
  cout << "realize prob" << endl;
  Image<float> output = prob.realize(width, height, channels);
  cout << "return" << endl;
  return output;
  #endif

  Image<float> normalizer(input.width(), input.height(), 1);
  Image<float> output(input.width(), input.height(), input.channels());
  #pragma omp parallel for
  for (int k = 0; k < input.channels(); k++) {
    for (int j = 0; j < input.height(); j++) {
      for (int i = 0; i < input.width(); i++) {
        output(i, j, k) = exp(output(i, j, k));
        normalizer(i, j) += output(i, j, k);
      }
    }
  }

  #pragma omp parallel for
  for (int j = 0; j < input.height(); j++) {
    for (int i = 0; i < input.width(); i++) {
      for (int k = 0; k < input.channels(); k++) {
        output(i, j, k) = output(i, j, k) / normalizer(i, j);
      }
    }
  }

  return output;
}
#endif

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

/*************************** SoftmaxWithLoss Layer ***************************/
SoftmaxWithLoss::SoftmaxWithLoss(string layer_name) 
{
  set_name(layer_name);
  set_type("SoftmaxWithLoss");
}

#if 0
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
#endif

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
