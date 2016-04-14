#include <iostream>
#include <fstream>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "io_utils.h"
#include "vision_layers.h"
//#include "proto2img_utils.h"

using namespace std;
using namespace caffe;

using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

static bool
test_conv(const char *img_path, const char *fpath) 
{
  /* Loads the model */
  NetParameter net_model;
  if (!LoadFromBinaryFile(fpath, &net_model)) {
    cerr << "LoadFromBinaryFile failed" << endl;
    return false;
  }

  /* Loads the image */
  Image<float> input = load_image(img_path);

  /* Let's just play with the first conv layer */
  LayerParameter layer = net_model.layer(3);
  BlobProto blob = layer.blobs(0);
  ConvolutionParameter conv_param = layer.convolution_param();
  Convolution conv_layer = Convolution(layer.name(), &conv_param, &blob);
  Image<float> output = conv_layer.convolve(input);

  return true;
}

#if 0
int 
main(int argc, char *argv[]) 
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc != 3) {
    cerr << "Usage:  " << argv[0] 
      << "test_image.png trained.caffemodel" << endl;
    return 1;
  }

  test_conv(argv[1], argv[2]);
  return 0;
}
#endif
