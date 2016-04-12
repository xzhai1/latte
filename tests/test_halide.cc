#include <iostream>
#include <fstream>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "io_utils.h"
#include "proto2img_utils.h"

using namespace std;
using namespace caffe;

using namespace Halide;
using namespace Halide::Tools;

static bool
test_conv(const char *img_path, const char *fpath) 
{
  NetParameter net_model;
  LayerParameter layer;
  BlobProto blob;
  ConvolutionParameter conv_param;

  int input_x, input_y, input_z;
  int k_size, num_output;

  Image<uint8_t> input;
  Func clamped;
  Image<float> output;
  Image<float> kernel;
  Func convolution("convolution");
  Var x("x"), y("y"), z("z");

  /* Loads the model */
  if (!LoadFromBinaryFile(fpath, &net_model)) {
    cerr << "LoadFromBinaryFile failed" << endl;
    return false;
  }

  /* Loads the image */
  input = load_image(img_path);
  clamped = BoundaryConditions::repeat_edge(input);

  input_x = input.width();
  input_y = input.height();
  input_z = input.channels();
  cout << "input size " << input_x << " X " 
    << input_y << " X " << input_z << endl;

  /* Let's just play with the first conv layer */
  layer = net_model.layer(3);
  conv_param = layer.convolution_param();
  k_size = conv_param.kernel_size(0);
  num_output = conv_param.num_output();

  /* and the first blob which is the kernel weights */
  blob = layer.blobs(0);
  kernel = LoadKernelFromBlob(&blob, k_size, num_output);

  /* We are reducing over the reduction domain */
  RDom r(-1, k_size, -1, k_size, 0, input_z);

  convolution(x, y, z) = sum(
      kernel(r.x, r.y, r.z + z*input_z) * clamped(x + r.x, y + r.y, r.z));
  //convolution.print_loop_nest();

  output = convolution.realize(
              input_x, input_y, num_output);

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
