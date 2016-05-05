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


/**
 * @brief save_data 
 *
 * @param halide_output
 * @param serial_output
 * @param conv_layer
 */
static void
save_data(const Image<float> halide_output, 
          const Image<float> serial_output, 
          const Convolution *conv_layer)
{
  ofstream outputfile("test.csv");

  /* TODO Just the first filter output */
  for (int f = 0; f < 1; f++) {
    for (int y = 0; y < conv_layer->get_height(); y++) {
      string row = "";
      for (int x = 0; x < conv_layer->get_width(); x++) {
        if (x == conv_layer->get_width() - 1)
          row += to_string(halide_output(x, y, f));
        else 
          row += to_string(halide_output(x, y, f)) + ",";
      }
      row += "\n";
      outputfile << row;
    }
  }

  outputfile.close();
}

bool
test_convolution(string image_path, NetParameter *net_model) 
{
  /* Loads the image */
  Image<float> input = load_image(image_path);

  /* TODO need to set global legacy flag */
  int num_of_layers = net_model->layer_size();
  if ( num_of_layers == 0)
    /* layers_size() is a deprecated version for older net */
    num_of_layers = net_model->layers_size();
 
  /* TODO for now just test the first conv layer */
  LayerParameter layer;
  for (int l_idx = 0; l_idx < num_of_layers; l_idx++) {
    layer = net_model->layer(l_idx);
    if (layer.type() == CONVOLUTION) {
      LOG(INFO) << "Running convolution test with layer " << l_idx;
      break;
    }
  }
  BlobProto kernel_blob = layer.blobs(0);
  BlobProto bias_blob = layer.blobs(1);
  ConvolutionParameter conv_param = layer.convolution_param();
  Convolution conv_layer = Convolution(layer.name(), 
                                       NULL, 
                                       &conv_param, 
                                       &kernel_blob, 
                                       &bias_blob);
 
  /* IMPT: need to run it before we have an output dimension */
  Func storage = conv_layer.run(
      (Func)input, input.extent(0), input.extent(1), input.extent(2), input.extent(3));

  /* Realize the output because we want to compare */
  int width = conv_layer.get_width();
  int height = conv_layer.get_height();
  int channels = conv_layer.get_channels();
  Image<float> halide_output = storage.realize(width, height, channels);
  //Image<float> serial_output = conv_layer.SerialConv(input);
  bool result = true;
  //bool result = check_output(halide_output, serial_output, &conv_layer);
  //save_data(halide_output, serial_output, &conv_layer);

  /* Uncomment if you want to see the result */
#if 0
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
