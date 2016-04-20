#include <iostream>
#include <fstream>
#include <string>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "layers/layers.h"
#include "layers/vision_layers.h"
#include "io_utils.h"

#include "tests.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

bool
test_convolution(string image_path, NetParameter *net_model) 
{
#if 0
  /* Loads the image */
  Image<float> input = load_image(image_path);

  /* Let's just play with the first conv layer */
  LayerParameter layer = net_model->layer(3);
  BlobProto kernel_blob = layer.blobs(0);
  BlobProto weight_blob = layer.blobs(1);
  ConvolutionParameter conv_param = layer.convolution_param();
  Convolution conv_layer = Convolution(layer.name(), &conv_param, 
                                       &kernel_blob, &weight_blob);
  Image<float> output = conv_layer.run(input);

  /* TODO Grab first layer and try saving the image */
  Func get_slice;
  Var x, y, z;
  get_slice(x, y, z) = output(x, y, z);
  Image<float> slice = get_slice.realize(output.width(), output.height(), 1);
  save_image(slice, "xxx.png");
#endif
  return true;
}
