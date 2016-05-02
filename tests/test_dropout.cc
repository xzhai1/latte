#include <iostream>
#include <fstream>
#include <string>

#include "caffe.pb.h"
#include "halide_image_io.h"
#include "CycleTimer.h"

#include "layers/common_layers.h"
#include "layers/layers.h"
#include "io_utils.h"

#include "tests.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

bool
test_dropout(string image_path, NetParameter *net_model) 
{
  /* Loads the image */
  Image<float> img = load_image(image_path);
  int batch_size = 1;
  Image<float> input(img.extent(0), img.extent(1), img.extent(2), batch_size);

  for (int w = 0; w < input.extent(3); w++)
    for (int z = 0; z < input.extent(2); z++) 
      for (int y = 0; y < input.extent(1); y++)
        for (int x = 0; x < input.extent(0); x++)
          input(x, y, z, w) = img(x, y, z);

  /* Let's just play with the first deconv layer */
  LayerParameter layer = net_model->layer(36);
  DropoutParameter dropout_param = layer.dropout_param();
  Dropout dropout_layer = Dropout(layer.name(), &dropout_param);

  /* Not finished */

  return true;
}