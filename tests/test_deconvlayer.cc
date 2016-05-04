#include <iostream>
#include <fstream>
#include <string>

#include "caffe/proto/caffe.pb.h"
#include "halide_image_io.h"
#include "CycleTimer.h"

#include "layers/vision_layers.h"
#include "layers/layers.h"
#include "io_utils.h"

#include "tests.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

bool
test_deconvolution(NetParameter *net_model) 
{
  /* Loads data */
  int width = 8, height = 9, channel = 60;
  Image<float> input(width, height, channel);
  for (int k = 0; k < channel; k++) {
    ifstream infile("./outputs/channel" + to_string(k) + ".txt");
    for (int j = 0; j < height; j++) {
      for (int i = 0; i < width; i++) {
        infile >> input(i, j, k) >> ws;
      }
    }
    infile.close();
  }

  /* Let's just play with the first deconv layer */
  LayerParameter layer = net_model->layer(41);
  BlobProto kernel_blob = layer.blobs(0);
  ConvolutionParameter deconv_param = layer.convolution_param();
  Deconvolution deconv_layer = Deconvolution(layer.name(), &deconv_param, 
                                            &kernel_blob, NULL);
  double startTime, endTime;
  startTime = CycleTimer::currentSeconds();
/*  
  Func input_func(input);
  Func output_func = deconv_layer.run(input_func,width,height, channel);

  int output_width = deconv_layer.get_width();
  int output_height = deconv_layer.get_height();
  int output_channels = deconv_layer.get_channels();

  Image<float> output(output_width, output_height, output_channels);

  Target target = get_host_target();
  target.set_feature(Target::CUDA);
  output_func.compile_jit(target);
  output_func.realize(output);
*/

  Image<float> output = deconv_layer.run(input);

  endTime = CycleTimer::currentSeconds();
  cout << "time elapsed: " << (endTime - startTime) * 1000 << " ms  " << endl;


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
