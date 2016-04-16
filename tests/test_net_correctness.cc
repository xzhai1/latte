#include <iostream>
#include <fstream>
#include <string>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "io_utils.h"
#include "CycleTimer.h"

#include "net.h"
#include "layers.h" 
#include "vision_layers.h"
#include "activation_layers.h"
#include "common_layers.h"
#include "loss_layers.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

bool
test_net(string image_path, NetParameter *net_model) 
{
  /* Loads the image */
  Image<float> input = load_image(image_path);

  /* Build the net */
  Net network(net_model);

  network.print_net();
  Image<float> final_image = network.run(input);

  /* Save first channel as image */
  Func get_slice;
  Var x, y, z;
  get_slice(x, y, z) = final_image(x, y, z);
  Image<float> slice = get_slice.realize(
      final_image.width(), final_image.height(), 1);
  save_image(slice, "xxx.png");

  return true;
}
