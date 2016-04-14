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
test_convolution(string image_path, NetParameter *net_model) 
{
  /* Loads the image */
  Image<float> input = load_image(image_path);

  for (int i = 0; i < net_model->layer_size(); i++) {
    LayerParameter layer = net_model->layer(i);
    if (layer.type() == CONVOLUTION) {
      cout << layer.name() << endl;
      /* TODO Just the weights for now */
      BlobProto blob = layer.blobs(0);
      ConvolutionParameter conv_param = layer.convolution_param();
      Convolution conv_layer = Convolution(layer.name(), &conv_param, &blob);
      Image<float> output = conv_layer.convolve(input);
    }
  }
  return true;
}
