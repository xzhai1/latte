#include <iostream>
#include <fstream>
#include <string>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "layers.h"
#include "tests.h"
#include "io_utils.h"
#include "vision_layers.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

bool
test_deconvolution(NetParameter *net_model) 
{
  /* Loads data */
  int width = 24, height = 40, channel = 60;
  Image<float> input(width, height, channel);
  for (int k = 0; k < channel; k++) {
    ifstream infile("./outputs/channel" + to_string(k) + ".txt");
    for (int j = 0; j < height; j++) {
      for (int i = 0; i < width; i++) {
        infile >> input(i, j, k) >> ws;
        cout << input(i, j, k);
      }
      cout << endl;
    }
    cout << "------------------------" << endl;
    infile.close();
  }

  /* Let's just play with the first deconv layer */
  LayerParameter layer = net_model->layer(41);
  BlobProto kernel_blob = layer.blobs(0);
  ConvolutionParameter deconv_param = layer.convolution_param();
  Deconvolution deconv_layer = Deconvolution(layer.name(), &deconv_param, 
                                            &kernel_blob, NULL);
  Image<float> output = deconv_layer.run(input);

  for (int k = 0; k < output.channels(); k++) {
    ofstream outfile("./outputs/deconv_channel" + to_string(k) + ".txt");
    for (int j = 0; j < output.height(); j++) {
      for (int i = 0; i < output.width(); i++) {
        outfile << input(i, j, k) << " ";
      }
      outfile << endl;
    }
    outfile.close();
  }

  return true;
}
