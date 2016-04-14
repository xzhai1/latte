#include <stdio.h>  /* perror() */
#include <iostream> /* cerr */
#include <fstream>  /* endl */

#include "proto2img_utils.h"

using namespace std;
using namespace Halide;
using namespace caffe;

Image<float>
LoadKernelFromBlob(const BlobProto *blob, int k_size, int num_output)
{
  /* Our kernel is num_output of [k_size, k_size, prev_num_output] slices 
   * stacked together. 
   * For example, the first convolution layer,
   *  prev_num_output = 3 
   *  k_size          = 3
   *  num_output      = 64
   * So the total number of weights in the blob is 3*3*3*64 = 1728 which is
   * exactly the size of the data. Hence, we can infer the previous layer
   * size from the total data size */
  int prev_num_output = blob->data_size() / (k_size*k_size*num_output);

  /* We want to center the kernel around 0. So a k_size = 3 will have the
   * kernel range from -1, to 1 */
  //int shifted = -(k_size - 1) / 2;
  Image<float> kernel(k_size, k_size, prev_num_output*num_output);

  int data_idx = 0;

  /* For each filter */
  for (int w = 0; w < num_output ; w++) {
    /* For each depth value */
    for (int k = 0; k < prev_num_output; k++) {
      /* Fill in the value */
      for (int j = 0; j < k_size; j++) {
        for (int i = 0; i < k_size; i++)
          kernel(i, j, w*prev_num_output + k) = blob->data(data_idx);
          data_idx++;
      }
    }
  }

  /* Center our kernel around 0 */
  //kernel.set_min(shifted, shifted, 0);
  return kernel;
}
