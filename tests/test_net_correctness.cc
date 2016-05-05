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
  Image<float> img = load_image(image_path);
  int batch_size = 1;
  Image<float> input(img.extent(0), img.extent(1), img.extent(2), batch_size);

  for (int w = 0; w < input.extent(3); w++)
    for (int z = 0; z < input.extent(2); z++) 
      for (int y = 0; y < input.extent(1); y++)
        for (int x = 0; x < input.extent(0); x++)
          input(x, y, z, w) = img(x, y, z);

  /* Build the net */
  Net network(net_model);

  network.print_net();
  Image<float> final_image = network.run(input);

  cout << "final_image.width = " << final_image.width() << endl;
  cout << "final_image.height = " << final_image.height() << endl;
  cout << "final_image.channels = " << final_image.channels() << endl;
  /* Save all channels */
  for (int k = 0; k < final_image.channels(); k++) {
    ofstream outfile;
    outfile.open ("./outputs/channel" + to_string(k) + ".txt");
    for (int j = 0; j < final_image.height(); j++) {
      for (int i = 0; i < final_image.width(); i++) {
        outfile << final_image(i, j, k, 0) << " ";
      }
      outfile << endl;
    }
    outfile.close();
  }
  return true;
}
