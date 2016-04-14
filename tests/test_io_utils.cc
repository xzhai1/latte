#include <iostream>
#include <fstream>
#include "caffe.pb.h"

#include "layers.h"
#include "tests.h"
#include "io_utils.h"
#include "proto2img_utils.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Latte;

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
ListBinaryLayer(const NetParameter *net) 
{
  cout << "name" << "\t\t\t" << "type" << "\t" 
       << "num_output" << "\t" << "pad" << "\t" << "kernel_size" 
       << "\t" << "stride" << endl;
  for (int i = 0; i < net->layer_size(); i++) {
    const LayerParameter layer = net->layer(i);
    string name = layer.name();
    string type = layer.type();
    if (type == CONVOLUTION) {
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
  ListBinaryLayer(net_model);
  return true;
}
