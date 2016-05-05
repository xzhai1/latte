#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "layers/activation_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

ReLU::ReLU(string layer_name, 
           const Layer *prev, 
           const ReLUParameter *param) 
  :Layer(layer_name, RELU)
{
  if (param->has_negative_slope())
    negative_slope = param->negative_slope();
  SetOutputDim(prev);
  storage(i, j, k, l) = max(0, prev->storage(i, j, k, l)) + 
                        negative_slope*min(0, prev->storage(i, j, k, l));

  /* Dynamic scheduling */
  storage.compute_root();
  int vector_size = (get_width() >= 16) ? 16 : 8;
  if (get_width()*2 <= get_channels()) {
    Var ko, ki, fused1, fused2;
    storage.split(k, ko, ki, 4);
    storage.parallel(ko);
    storage.fuse(i, j, fused1).fuse(fused1, ki, fused2);
    storage.vectorize(fused2, vector_size);
  } else {
    storage.parallel(k).vectorize(i, vector_size);
  }
}

} /* namespace latte */
