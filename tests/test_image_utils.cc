#include <string>
#include <iostream>

#include "Halide.h"
#include "halide_image_io.h"

#include "image_utils.h"

using namespace std;
using namespace Halide;
using namespace Halide::Tools;

bool
test_im2col(string image_path)
{
  Image<float> input = load_image(image_path);
  cout << "input dimension" << endl;
  cout << input.width() << endl;
  cout << input.height() << endl;

  int kernel_size = 3;
  int stride = 1;
  int pad = 1;
  Image<float> output1 = im2col(input, kernel_size, pad, stride);

  #if 0
  cout << "output dimension" << endl;
  cout << output.width() << endl;
  cout << output.height() << endl;
  save_image(output, "im2col.png");
  #endif

  Image<float> output2 = col2im(output1, kernel_size, pad, stride, input.width(), input.height(), input.channels());
  cout << "output2 dimension" << endl;
  cout << output2.width() << endl;
  cout << output2.height() << endl;
  save_image(output2, "col2im.png");

  return true;
}
