#include <iostream>
#include <algorithm>

#include "Halide.h"
#include "caffe.pb.h"

#include "vision_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

/* Definitions for class Convolution */
Convolution::Convolution(string name, const ConvolutionParameter *param, 
                         const BlobProto *blob) {
  name = name;
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->pad_size()) {
    pad = param->pad(0);
  }
  /* Fill in the kernel */
  kernel = LoadKernelFromBlob(blob, kernel_size, num_output);
}

Image<float> Convolution::convolve(Image<float> input) {
  Func convolution("convolution");
  Var x("x"), y("y"), z("z");  
  int width = input.width();
  int height = input.height();
  int channels = input.channels();
  int start = -(kernel_size - 1)/ 2;
 
  /* TODO for now, we repead the edge. Don't know what to do with pad yet */
  Func clamped = BoundaryConditions::repeat_edge(input);
  RDom r(start, kernel_size, start, kernel_size, 0, channels);
  convolution(x, y, z) = sum(
      kernel(r.x, r.y, r.z + z*channels) * clamped(x + r.x, y + r.y, r.z));

  /* TODO define schedule */
  // Might have some problem with output width and height
  Image<float> output = convolution.realize(width, height, num_output);
  return output;
}

/* Definitions for class ReLU */
ReLU::ReLU(string name, const ReLUParameter *param,
                 float negative_slope) {
  name = name;
  num_output = param->num_output();
  negative_slope = negative_slope;
}

/* Definitions for class Pooling */
Image<float> ReLU::relu(Image<float> input) {
  Func rectified("relu");
  Var x("x"), y("y"), z("z");
  int width = input.width();
  int height = input.height();
  int channels = input.channels();

  rectified(x, y, z) = Halide::max(0, input(x, y, z)) + negative_slope *
                       Halide::min(0, input(x, y, z));

  Image<float> output = rectified.realize(width, height, num_output);
  return output;
}


/* Definitions for class Dropout */


/* Definitions for class Deconvolution */


/* Definitions for class Crop */


/* Definitions for class Split */


/* Definitions for class SoftmaxWithLoss */


/* Definitions for class Softmax */


/* Definitions for class Silence */

} /* namespace latte */
