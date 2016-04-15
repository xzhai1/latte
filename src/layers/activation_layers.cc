#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "activation_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

ReLU::ReLU(string layer_name, const ReLUParameter *param) 
{
  name = layer_name;
  if (param->has_negative_slope())
    negative_slope = param->negative_slope();
}

Image<float> 
ReLU::rectify(Image<float> input)
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
  Image<float> output = rectified.realize(width, height, channels);
  return output;
}

} /* namespace latte */
