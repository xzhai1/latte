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
