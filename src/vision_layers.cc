#include <iostream>

#include "Halide.h"
#include "caffe.pb.h"

#include "vision_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

Convolution::Convolution(string name, const ConvolutionParameter *param, 
                         const BlobProto *weights, const BlobProto *bias_b) 
{
  name = name;
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->pad_size()) {
    pad = param->pad(0);
  }
  kernel = LoadKernelFromBlob(weights, kernel_size, num_output);
  bias = LoadBiasFromBlob(bias_b, num_output);
}

Image<float> Convolution::convolve(Image<float> input) {
  Func convolution;
  Var x, y, z;  
  int width = input.width();
  int height = input.height();
  int channels = input.channels();
  int start = -(kernel_size - 1)/ 2;
 
  /* TODO for now, we repead the edge. Don't know what to do with pad yet */
  Func clamped = BoundaryConditions::repeat_edge(input);
  RDom r(start, kernel_size, start, kernel_size, 0, channels);

  /* Reduce over kernel */
  convolution(x, y, z) = sum(
      kernel(r.x, r.y, r.z + z*channels) * clamped(x + r.x, y + r.y, r.z));
  /* and add bias */
  convolution(x, y, z) += bias(0, 0, z);
    
  /* TODO define schedule */
  Image<float> output = convolution.realize(width, height, num_output);
  return output;
}

} /* namespace latte */
