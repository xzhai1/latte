#include <iostream>
#include <fstream>
#include "caffe.pb.h"

#include "layers/layers.h"
#include "tests.h"
#include "io_utils.h"
#include "proto2img_utils.h"

#include "halide_image_io.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

static void
SaveLoadedKernel(const NetParameter *net)
{
  /* The first conv layer */
  int layer_idx = 3;
  int input_channels = 3;
  //const LayerParameter layer = net->layer(layer_idx);
  const V1LayerParameter layer = net->layers(0);
  ConvolutionParameter conv_param = layer.convolution_param();
  string layer_name = layer.name();

  int kernel_size = conv_param.kernel_size(0);
  int num_output = conv_param.num_output();
  BlobProto weight_blob = layer.blobs(0);
  Image<float> kernel = LoadKernelFromBlob(
      &weight_blob, kernel_size, num_output);

  int k_width = kernel.width();
  int k_height = kernel.height();
 
  /* TODO we know kernel size and input depth */
  Image<float> filter(kernel_size, kernel_size, input_channels);

  /* Loop through the kernel and save filters */
  for (int k = 0; k < num_output; k++) {
    string k_path = "./outputs/" + layer_name + "_kernel" + to_string(k) + ".png";
    for (int c = 0; c < input_channels; c++) {
      for (int j = 0; j < k_height; j++) {
        for (int i = 0; i < k_width; i++) {
          filter(i, j, c) = kernel(i, j, k*input_channels + c);
        }
      }
    }
    save_image(filter, k_path);
  }
}

static void 
ListTxtLayer(const NetParameter *net) 
{
  cout << "name" << "\t" << "type" << endl;
  for (int i = 0; i < net->layer_size(); i++) {
    const LayerParameter layer = net->layer(i);
    cout << layer.name() << "\t" 
         << layer.type() << endl;
  }
}

static void 
ListBinaryLayerLegacy(const NetParameter *net) 
{
  cout << "name" << "\t\t\t" << "type" << "\t" 
       << "num_output" << "\t" << "pad" << "\t" << "kernel_size" 
       << "\t" << "stride" << endl;
  int layer_size = net->layers_size();

  for (int i = 0; i < layer_size; i++) {
    const V1LayerParameter layer = net->layers(i);
    string name = layer.name();
    cout << name << "\t" << endl;
  }
}


static void 
ListBinaryLayer(const NetParameter *net) 
{
  cout << "name" << "\t\t\t" << "type" << "\t" 
       << "num_output" << "\t" << "pad" << "\t" << "kernel_size" 
       << "\t" << "stride" << endl;
  int layer_size = net->layer_size();

  for (int i = 0; i < layer_size; i++) {
    const LayerParameter layer = net->layer(i);

    string name = layer.name();
    string type = layer.type();
    if (type == CONVOLUTION || type == DECONVOLUTION) {
      ConvolutionParameter conv_param = layer.convolution_param();
      int kernel_size = conv_param.kernel_size(0);
      int num_output = conv_param.num_output();
      int pad = 0;
      int stride = 1;
      if (conv_param.pad_size()) {
        pad = conv_param.pad(0);
      }
      if (conv_param.stride_size()) {
        stride = conv_param.stride(0);
      }
      cout << name << "\t\t\t" << type << "\t" 
           << num_output << "\t" << pad << "\t" << kernel_size 
           << "\t" << stride << endl;
    } else if (type == POOLING) {
      PoolingParameter pool_param = layer.pooling_param();
      int kernel_size = 1;
      if (pool_param.has_kernel_size()) {
        kernel_size = pool_param.kernel_size();
      }
      int pad = 0;
      int stride = 1;
      if (pool_param.has_pad()) {
        pad = pool_param.pad();
      }
      if (pool_param.has_stride()) {
        stride = pool_param.stride();
      }
      cout << name << "\t\t\t" << type << "\t" 
           << "---" << "\t" << pad << "\t" << kernel_size 
           << "\t" << stride << endl;

    } else {
      cout << name << "\t\t\t" << type << endl;
    }
  }
}

bool
test_LoadFromTextFile(string fpath, NetParameter *net_model) 
{
  cout << "Loading prototxt from " << fpath << endl;
  if (!LoadFromTextFile(fpath, net_model)) {
    cerr << "test_LoadFromTextFile failed" << endl;
    return false;
  }
  ListTxtLayer(net_model);
  return true;
}

bool
test_LoadFromBinaryFile(string fpath, NetParameter *net_model) 
{
  cout << "Loading trained binary file from " << fpath << endl;
  if (!LoadFromBinaryFile(fpath, net_model)) {
    cerr << "test_LoadFromBinaryFile failed" << endl;
    return false;
  }
  if (net_model->layer_size() == 0) {
    ListBinaryLayerLegacy(net_model);
  } else {
    ListBinaryLayer(net_model);
  }
  SaveLoadedKernel(net_model);
  return true;
}
