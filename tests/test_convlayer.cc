#include <iostream>
#include <fstream>
#include <string>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "layers/layers.h"
#include "layers/vision_layers.h"
#include "io_utils.h"

#include "proto2img_utils.h"

#include "tests.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

static bool
check_output(Image<float> halide_output, Image<float> serial_output)
{
  int h_width    = halide_output.width();
  int h_height   = halide_output.height();
  int h_channels = halide_output.channels();

  int s_width    = serial_output.width();
  int s_height   = serial_output.height();
  int s_channels = serial_output.channels();

  /* Check dimensions first */
  if (h_width != s_width || h_height != s_height || h_channels != s_channels) {
    cout << "Halide convolution output dimensions and "
            "serial convolution dimensions do not agree" << endl;
    cout << "Halide output [" 
         << h_width << "x" << h_height << "x" << h_channels << "]";
    cout << "Serial output [" 
         << s_width << "x" << s_height << "x" << s_channels << "]";
    return false;
  }
  /* TODO loop through the pixels and check */
  return true;
}

bool
test_convolution(string image_path, NetParameter *net_model) 
{
  /* Loads the image */
  Image<float> input = load_image(image_path);

  /* TODO Let's just play with the first conv layer */
  LayerParameter layer = net_model->layer(3);
  BlobProto kernel_blob = layer.blobs(0);
  BlobProto weight_blob = layer.blobs(1);
  ConvolutionParameter conv_param = layer.convolution_param();
  Convolution conv_layer = Convolution(layer.name(), &conv_param, 
                                       &kernel_blob, &weight_blob);
  
  /* Realize the output because we want to compare */
  int width = conv_layer.get_width();
  int height = conv_layer.get_height();
  int channels = conv_layer.get_channels();
  Image<float> halide_output = 
    conv_layer.run(
        (Func)input, input.width(), input.height(), input.channels()).
        realize(width, height, channels);
  Image<float> serial_output = conv_layer.SerialConv(input);
  check_output(halide_output, serial_output);

#if 0
  /* TODO Grab first layer and try saving the image */
  Func get_slice;
  Var x, y, z;
  get_slice(x, y, z) = output(x, y, z);
  Image<float> slice = get_slice.realize(output.width(), output.height(), 1);
  save_image(slice, "xxx.png");
#endif
  return true;
}
