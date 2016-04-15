#include <iostream>

#include "Halide.h"
#include "halide_image_io.h"
#include "caffe.pb.h"

#include "vision_layers.h"
#include "proto2img_utils.h" /* LoadKernelFromBlob */

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;

Convolution::Convolution(string l_name, const ConvolutionParameter *param, 
                         const BlobProto *weights, const BlobProto *bias_b) 
{
  name = l_name;
  kernel_size = param->kernel_size(0);
  num_output = param->num_output();
  if (param->pad_size()) {
    pad = param->pad(0);
  }
  kernel = LoadKernelFromBlob(weights, kernel_size, num_output);
  bias = LoadBiasFromBlob(bias_b, num_output);
  initialized = true;
}

bool
Convolution::export_filters()
{  
  /* TODO can't save without initialization */
  //if (!initialized) {
  //}
  int input_channels = kernel.channels() / num_output;
  for (int z = 0; z < kernel.channels(); z++) {
    if (z % input_channels == 0)
      cout << endl;
    cout << "--------------------------------" << endl;
    cout << "|" << kernel(0, 0, z) << "|" << kernel(1, 0, z) << "|" 
         << kernel(2, 0, z) << "|" << endl;
    cout << "|" << kernel(0, 1, z) << "|" << kernel(1, 1, z) << "|" 
         << kernel(2, 1, z) << "|" << endl;
    cout << "|" << kernel(0, 2, z) << "|" << kernel(1, 2, z) << "|" 
         << kernel(2, 2, z) << "|" << endl;
    cout << "--------------------------------" << endl;
  }

  return true;
}

Image<float>
Convolution::convolve(Image<float> input) 
{
  /* TODO can't convolve without initialization */
  //if (!initialized) {
  //}
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
