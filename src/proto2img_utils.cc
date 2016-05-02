#include <stdio.h>  /* perror() */
#include <iostream> /* cerr */
#include <fstream>  /* endl */

#include "proto2img_utils.h"

using namespace std;
using namespace Halide;
using namespace caffe;

Image<float>
LoadBiasFromBlob(const BlobProto *blob, int num)
{
  /* Bias is a "column" with num of bias values, each for a layer of
   * the output */
  Image<float> bias(1, 1, 1, num);

  for (int l = 0; l < num; l++) {
    bias(0, 0, 0, l) = blob->data(l);
  }

  return bias;
}

/* Assume square kernel */
Image<float>
LoadKernelFromBlob(const BlobProto *blob, int kernel_size, int num)
{
  int channels = blob->data_size() / (kernel_size*kernel_size*num);

  /* INPUT FORMAT:  blob->data is num x channels x height x width 
   * OUTPUT FORMAT: kernel is width x height x channels x num */
  Image<float> kernel(kernel_size, kernel_size, channels, num);

  int data_idx = 0;

  /* For each filter */
  for (int w = 0; w < num ; w++) {
    /* For each depth value */
    for (int k = 0; k < channels; k++) {
      /* Fill in the value */
      for (int j = 0; j < kernel_size; j++) {
        for (int i = 0; i < kernel_size; i++) {
          kernel(i, j, k, w) = blob->data(data_idx);
          data_idx++;
        }
      }
    }
  }

  return kernel;
}
