#include <stdlib.h> /* abs */
#include <fstream>
#include <string>

#include <glog/logging.h>  /* Google's logging module */

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

/**
 * @brief check_output Validates halide output and serial output pixel by pixel
 *
 * @param halide_output
 * @param serial_output
 * @param conv_layer
 *
 * @return 
 */
static bool
check_output(const Image<float> halide_output, 
             const Image<float> serial_output, 
             const Convolution *conv_layer)
{
  /* Output dimension */
  int width    = conv_layer->get_width();
  int height   = conv_layer->get_height();
  int channels = conv_layer->get_channels();

  for (int c = 0; c < channels; c++) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        float halide_val = halide_output(x, y, c);
        float serial_val = serial_output(x, y, c);
        if (abs(halide_val - serial_val) > PIXEL_THRESHOLD) {
          LOG(ERROR) << "Value disagreement at (" 
               << x << "," << y << "," << c << ")";
          LOG(ERROR) << "halide_val = " << halide_val;
          LOG(ERROR) << "serial_val = " << serial_val;
          return false;
        }
      }
    }
  }
  return true;
}

bool
test_convolution(string image_path, NetParameter *net_model) 
{
  /* Loads the image */
  Image<float> input = load_image(image_path);

  int layer_idx = 3;
  LOG(INFO) << "Running convolution test with layer " << layer_idx;

  /* TODO Let's just play with the first conv layer */
  LayerParameter layer = net_model->layer(layer_idx);
  BlobProto kernel_blob = layer.blobs(0);
  BlobProto bias_blob = layer.blobs(1);
  ConvolutionParameter conv_param = layer.convolution_param();
  Convolution conv_layer = Convolution(layer.name(), &conv_param, 
                                       &kernel_blob, &bias_blob);
 
  /* IMPT: need to run it before we have an output dimension */
  Func storage = conv_layer.run(
      (Func)input, input.width(), input.height(), input.channels());

  /* Realize the output because we want to compare */
  int width = conv_layer.get_width();
  int height = conv_layer.get_height();
  int channels = conv_layer.get_channels();
  Image<float> halide_output = storage.realize(width, height, channels);
  Image<float> serial_output = conv_layer.SerialConv(input);
  bool result = check_output(halide_output, serial_output, &conv_layer);

  /* Uncomment if you want to see the result */
#ifdef DEBUG
  Func get_halide_slice, get_serial_slice;
  Var x, y, z;
  get_halide_slice(x, y, z) = halide_output(x, y, z);
  get_serial_slice(x, y, z) = serial_output(x, y, z);

  Image<float> halide_slice = get_halide_slice.realize(width, height, 1);
  Image<float> serial_slice = get_serial_slice.realize(width, height, 1);

  save_image(halide_slice, "halide_convolution_slice.png");
  save_image(serial_slice, "serial_convolution_slice.png");
#endif
  return result; 
}
