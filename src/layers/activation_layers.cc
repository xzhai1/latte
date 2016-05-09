#include <iostream>
#include <algorithm>
#include <stdlib.h>

#include "Halide.h"
#include "caffe.pb.h"

#include "layers/activation_layers.h"

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

ReLU::ReLU(string layer_name, 
           const Layer *prev, 
           const ReLUParameter *param) 
  :Layer(layer_name, RELU)
{
  if (param->has_negative_slope())
    negative_slope = param->negative_slope();

  /* Set output dimension */
  SetOutputDim(prev);

  /* Define algorithm */
  storage(i, j, k, l) = max(0, prev->storage(i, j, k, l)) + 
                        negative_slope*min(0, prev->storage(i, j, k, l));

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
