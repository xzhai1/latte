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

  /* Let's just play with the first conv layer */
  LayerParameter layer = net_model->layer(3);
  BlobProto kernel_blob = layer.blobs(0);
  BlobProto weight_blob = layer.blobs(1);
  ConvolutionParameter conv_param = layer.convolution_param();
  Convolution conv_layer = Convolution(layer.name(), &conv_param, &kernel_blob, &weight_blob);
  Image<float> output = conv_layer.convolve(input);

  return true;
}
