#include <iostream>
#include <fstream>
#include <string>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "io_utils.h"
#include "CycleTimer.h"

#include "layers/layers.h" 
#include "layers/vision_layers.h"
#include "layers/activation_layers.h"
#include "layers/common_layers.h"
#include "layers/loss_layers.h"

#include "net.h"

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

  /* Save all channels */
  for (int k = 0; k < final_image.channels(); k++) {
    ofstream outfile;
    outfile.open ("./outputs/softmax_channel" + to_string(k) + ".txt");
    for (int j = 0; j < final_image.height(); j++) {
      for (int i = 0; i < final_image.width(); i++) {
        outfile << final_image(i, j, k) << " ";
      }
      outfile << endl;
    }
    outfile.close();
  }

  return true;
}
