#include <iostream>
#include <fstream>
#include <string>

#include "caffe.pb.h"
#include "halide_image_io.h"

#include "io_utils.h"

#include "layers.h" 
#include "vision_layers.h"
#include "activation_layers.h"
#include "common_layers.h"
#include "loss_layers.h"

using namespace std;
using namespace caffe;
using namespace Halide;
using namespace Halide::Tools;
using namespace Latte;

bool
test_net(string image_path, NetParameter *net_model) 
{
  /* Loads the image */
  Image<float> input = load_image(image_path);

  Layer *head = NULL;
  Layer *prev_layer = NULL;

  /* Let's build the net */
  for (int i = 0; i < net_model->layer_size(); i++) {
    LayerParameter layer = net_model->layer(i);
    string name = layer.name();
    string type = layer.type();
    cout << "name " << name << endl;
    /* TODO we are ignoring a couple of types here */
    /* This is a going to be a big switch statement */
    if (type == CONVOLUTION) {
      ConvolutionParameter conv_param = layer.convolution_param();
      BlobProto kernel_blob = layer.blobs(0);
      BlobProto bias_blob = layer.blobs(1);
      Convolution conv_layer = Convolution(name, &conv_param, 
                                           &kernel_blob, &bias_blob);
    } 
    else if (type == RELU) {
      ReLUParameter relu_param = layer.relu_param();
      ReLU relu_layer(name, &relu_param);
    } else if (type == POOLING) {
      PoolingParameter pool_param = layer.pooling_param();
      Pooling pool_layer(name, &pool_param);
    } else if (type == DROPOUT) {
      DropoutParameter drop_param = layer.dropout_param();
      Dropout dropout_layer(name, &drop_param);
    }
  }
  return true;
}
