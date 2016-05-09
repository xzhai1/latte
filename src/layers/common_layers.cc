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

} /* namespace latte */
