#include <iostream>
#include <fstream>
#include "caffe.pb.h"

#include "io_utils.h"
#include "proto2img_utils.h"

using namespace std;
using namespace caffe;

static void 
ListLayer(const NetParameter& net) 
{
  int i;

  for (i = 0; i < net.layer_size(); i++) {
    /* Get the layer */
    const LayerParameter& layer = net.layer(i);
    cout << layer.name() << endl;
    cout << layer.type() << endl;
  }
}

static void 
ListLayerWeights(const NetParameter& net) 
{
  int i;
  BlobProto blob;
  ConvolutionParameter conv_param;
  int k_size, num_output;
  Image<float> kernel;

  for (i = 0; i < net.layer_size(); i++) {
    const LayerParameter& layer = net.layer(i);
    cout << "Layer name: "<< layer.name() << endl;
    cout << "Layer type: " << layer.type() << endl;
    /* print out conv layer kernel size */
    if (layer.has_convolution_param()) {
      conv_param = layer.convolution_param();
      /* Look at the proto, kernel_size, pad and stride are repeated... */
      k_size = conv_param.kernel_size(0);
      cout << "kernel size " << k_size << endl;
      if (conv_param.pad_size()) {
        cout << "pad " << conv_param.pad(0) << endl;
      }
      if (conv_param.stride_size()) {
        cout << "stride " << conv_param.stride(0) << endl;
      }
      num_output = conv_param.num_output();
      cout << "num output " << num_output << endl;
      /* TODO the first one if the conv */
      blob = layer.blobs(0);
      kernel = LoadKernelFromBlob(&blob, k_size, num_output);
    }
   
#if 0
    /* print out the parameters */
    for (j = 0; j < layer.blobs_size(); j++) {
      blob = layer.blobs(j);
      cout << "blob data_size(): " << blob.data_size() << endl;
      /* Say we just grab a data off it */
      cout << "sample data: "<< blob.data(0) << endl; 
    }
#endif
    cout << endl;
  }
}

static bool
test_LoadFromTextFile(const char *fpath) 
{
  caffe::NetParameter net_model;

  if (!LoadFromTextFile(fpath, &net_model)) {
    cerr << "test_LoadFromTextFile failed" << endl;
    return false;
  }

  //ListLayer(net_model);
  return true;
}

static bool
test_LoadFromBinaryFile(const char *fpath) 
{
  caffe::NetParameter net_model;

  if (!LoadFromBinaryFile(fpath, &net_model)) {
    cerr << "test_LoadFromBinaryFile failed" << endl;
    return false;
  }

  ListLayerWeights(net_model);
  return true;
}

#if 0
int 
main(int argc, char *argv[]) 
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc != 3) {
    cerr << "Usage:  " << argv[0] << " .prototxt .caffemodel" << endl;
    return 1;
  }

  //test_LoadFromTextFile(argv[1]);
  test_LoadFromBinaryFile(argv[2]);
}
#endif
