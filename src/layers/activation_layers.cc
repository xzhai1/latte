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

#if 0
  /* v1 */
  int vector_size = (get_width() >= 16) ? 16 : 8;
  Var fused;
  storage.compute_root();
  storage.fuse(k, l, fused);
  storage.parallel(fused);
  storage.vectorize(i, vector_size);
#endif

#if 0
  /* v2 */
  storage.compute_root();
  storage.parallel(k);
  int split_num = get_height() > 15 ? get_height() / 15 : 8;
  int vector_size = (get_width() >= 16) ? 16 : 8;
  Var jo, ji;
  storage.split(j, jo, ji, split_num).parallel(jo);
  storage.vectorize(i, vector_size);
#endif

  /* v3 */
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
