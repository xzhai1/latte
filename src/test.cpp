#include "Halide.h"
#include "halide_image_io.h"

using namespace Halide;
using namespace Halide::Tools;

int 
main(int argc, char *argv[]) 
{
  Image<uint8_t> input;
  Image<int8_t> output; 
  Func convolution("convolution");
  Var x("x"), y("y"), c("c");

  input = load_image("images/gray.png");

  /* Define convolution kernel */
  Image<int8_t> kernel(3, 3);
  kernel(-1, -1) = -1;
  kernel(-1,  0) = -1;
  kernel(-1,  1) = -1;
  kernel( 0, -1) =  0;
  kernel( 0,  0) =  0;
  kernel( 0,  1) =  0;
  kernel( 1, -1) =  1;
  kernel( 1,  0) =  1;
  kernel( 1,  1) =  1;
  RDom r(kernel);
  convolution(x, y) = 
    sum(kernel(r.x, r.y) * input(x + r.x, y + r.y));

  output = convolution.realize(
        input.width() - 2, input.height() - 2);
  save_image(output, "xxx.png");
  
  printf("Success!\n");
  return 0;
}
