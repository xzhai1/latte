#include <stdio.h>  /* perror() */
#include <iostream> /* cerr */
#include <fstream>  /* endl */

#include "proto2img_utils.h"

using namespace std;
using namespace Halide;
using namespace caffe;

Image<float>
LoadBiasFromBlob(const BlobProto *blob, int num_output)
{
  /* Bias is a "column" with num_output of bias values, each for a layer of
   * the output */
  Image<float> bias(1, 1, num_output);

  for (int k = 0; k < num_output; k++) {
    bias(0, 0, k) = blob->data(k);
  }

  return bias;
}

Image<float>
LoadKernelFromBlob(const BlobProto *blob, int k_size, int num_output)
{
  int prev_num_output = blob->data_size() / (k_size*k_size*num_output);

  /* IMPT: blob->data is num x channels x height x width */
  Image<float> kernel(k_size, k_size, prev_num_output, num_output);

  int data_idx = 0;

  /* For each filter */
  for (int w = 0; w < num_output ; w++) {
    /* For each depth value */
    for (int k = 0; k < prev_num_output; k++) {
      /* Fill in the value */
      for (int j = 0; j < k_size; j++) {
        for (int i = 0; i < k_size; i++) {
          kernel(i, j, k, w) = blob->data(data_idx);
          data_idx++;
        }
      }
    }
  }

  /* Center our kernel around 0 */
  return kernel;
}
