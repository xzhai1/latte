#include <iostream>
#include <fstream>
#include "caffe.pb.h"

#include <glog/logging.h>  /* Google's logging module */

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

/**
 * @brief ListTxtLayer Display layers loaded in from prototxt 
 *
 * @param net
 */
static void 
ListTxtLayer(const NetParameter *net) 
{
  LOG(INFO) << "Name" << "\t" << "Type";
  for (int i = 0; i < net->layer_size(); i++) {
    const LayerParameter layer = net->layer(i);
    LOG(INFO) << layer.name() << "\t" << layer.type();
  }
}


/**
 * @brief ListBinaryLayer Display layers loaded in from .caffemodel
 * It prints out number of outputs, pad, kernel size and stride for layers
 * that have them
 *
 * @param net   the model loaded in from .caffemodel
 * @param fpath the path from which it is loaded
 */
static void 
ListBinaryLayer(const NetParameter *net, string fpath) 
{
  LOG(INFO) << "-------------------------------------------------------------"
    "--------"; 
  LOG(INFO) << "|" << fpath << "|";
  LOG(INFO) << "| has these layers and parameters                            "
    "       |";
  LOG(INFO) << "-------------------------------------------------------------"
    "--------"; 
  LOG(INFO) << "Name" << "\t\t" << "Type" << "\t\t" 
            << "num_output" << "\t" << "pad" << "\t" << "kernel_size" 
            << "\t" << "stride";

  int layer_size = net->layer_size();
  for (int i = 0; i < layer_size; i++) {
    const LayerParameter layer = net->layer(i);
    string name = layer.name();
    string type = layer.type();

    /* Some layers don't have the information we needed */
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

      if (name.length() > 5)
        LOG(INFO) << name << "\t" << type << "\t" 
                  << num_output << "\t\t" << pad << "\t" << kernel_size 
                  << "\t\t" << stride << endl;
      else
        LOG(INFO) << name << "\t\t" << type << "\t" 
                  << num_output << "\t\t" << pad << "\t" << kernel_size 
                  << "\t\t" << stride << endl;

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
      LOG(INFO) << name << "\t\t" << type << "\t\t" 
                << "---" << "\t\t" << pad << "\t" << kernel_size 
                << "\t\t" << stride << endl;

    } else if (type != SPLIT) {
      if (name.length() > 5)
        LOG(INFO) << name << "\t" << type << endl;
      else
        LOG(INFO) << name << "\t\t" << type << endl;
    }
  }
}

bool
TestLoadFromTextFile(string fpath, NetParameter *net_model) 
{
  LOG(INFO) << "Loading prototxt from " << fpath;
  if (!LoadFromTextFile(fpath, net_model)) {
    LOG(FATAL )<< "test_LoadFromTextFile failed";
    return false;
  }
  ListTxtLayer(net_model);
  return true;
}

bool
TestLoadFromBinaryFile(string fpath, NetParameter *net_model) 
{
  LOG(INFO) << "Loading trained binary file from " << fpath;
  if (!LoadFromBinaryFile(fpath, net_model)) {
    LOG(FATAL) << "test_LoadFromBinaryFile failed" << endl;
    return false;
  }
  ListBinaryLayer(net_model, fpath);
  return true;
}
