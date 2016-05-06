#include <string>
#include <fstream>

#include "glog/logging.h"
#include "CycleTimer.h"
#include "caffe.pb.h"
#include "halide_image_io.h"

#include "layers/layers.h" 
#include "layers/vision_layers.h"
#include "layers/activation_layers.h"
#include "layers/common_layers.h"
#include "layers/loss_layers.h"

#include "io_utils.h"
#include "net.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

bool
TestNet(string image_path, 
        NetParameter *net_model, 
        int batch_size, 
        int iterations) 
{
  /* Loads the image */
  Image<float> img = load_image(image_path);
  
  /* Reshape the input image because we are dealing with one input image */
  Image<float> input(img.extent(0), img.extent(1), img.extent(2), batch_size);

  for (int w = 0; w < input.extent(3); w++)
    for (int z = 0; z < input.extent(2); z++) 
      for (int y = 0; y < input.extent(1); y++)
        for (int x = 0; x < input.extent(0); x++)
          input(x, y, z, w) = img(x, y, z);

  /* Build the net */
  Net network(net_model, input);
  network.PrintNet();

  /* Run the network */
  Image<float> final_image = network.Run(input, iterations);
  
  LOG(INFO) << "final_image.width    = " << final_image.extent(0);
  LOG(INFO) << "final_image.height   = " << final_image.extent(1);
  LOG(INFO) << "final_image.channels = " << final_image.extent(2);
  LOG(INFO) << "final_image.num      = " << final_image.extent(3);

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
