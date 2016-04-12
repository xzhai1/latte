#include <stdio.h>  /* perror() */
#include <iostream> /* cerr */
#include <fstream>  /* endl */

#include "proto2img_utils.h"

using namespace std;

Image<float>
LoadKernelFromBlob(const BlobProto *blob, int k_size, int num_output)
{
  int i, j, k, w, data_idx;
  int prev_num_output;
  int data_size;
  data_size = blob->data_size();

  /* Our kernel is curr_depth of [k_size, k_size, chunk_size] slices stacked
   * together. For example, the first convolution layer,
   *  prev_num_output = 3 
   *  k_size          = 3
   *  num_output      = 64
   * So the total number of weights in the blob is 3*3*3*64 = 1728 which is
   * exactly the size of the data. Hence, we can infer the previous layer
   * size from the total data size */
  prev_num_output = data_size / (k_size*k_size*num_output);

  Image<float> kernel(k_size, k_size, prev_num_output*num_output);

  data_idx = 0;

  /* For each filter */
  for (w = 0; w < data_size; w += num_output) {
    /* For each depth value */
    for (k = 0; k < prev_num_output; k++) {
      /* Fill in the value */
      for (j = 0; j < k_size; j++) {
        for (i = 0; i < k_size; i++)
          kernel(i, j, w*prev_num_output + k) = blob->data(data_idx);
          data_idx++;
      }
    }
  }
  return kernel;
}
