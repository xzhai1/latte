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

  //for (int i = 0; i < net_model->layer_size(); i++) {
  // TODO this is a hack. Just looking at first conv layer
  int i = 3;
  LayerParameter layer = net_model->layer(i);
  if (layer.type() == CONVOLUTION) {
    cout << layer.name() << endl;
    BlobProto weights = layer.blobs(0);
    BlobProto bias = layer.blobs(1);
    ConvolutionParameter conv_param = layer.convolution_param();
    Convolution conv_layer = Convolution(layer.name(), &conv_param, 
                                         &weights, &bias);
    Image<float> output = conv_layer.convolve(input);
    conv_layer.export_filters();
#if 0
    /* Grab each layer and try saving the image */
    Func get_slice;
    Var x, y, z;
    get_slice(x, y, z) = output(x, y, z);
    for (int s = 0; s < 5; s++) {
      output.set_min(0, 0, s);
      Image<float> slice = get_slice.realize(output.width(), output.height(), 1);
      save_image(slice, "filter" + to_string(s) + ".png");
    }
#endif
  }
  //}
  return true;
}
