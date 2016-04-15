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
  Layer *next_ptr = NULL;

  cout << "name \taddr" << endl;
  /* Let's build the net */
  /* TODO we are cheating here. We know the first conv layer starts at 3 */
  for (int i = 3; i < net_model->layer_size(); i++) {
    LayerParameter layer = net_model->layer(i);
    string name = layer.name();
    string type = layer.type();
    /* TODO we are ignoring a couple of types here */
    /* This is a going to be a big switch statement */
    if (type == CONVOLUTION) {
      ConvolutionParameter conv_param = layer.convolution_param();
      BlobProto weight_blob = layer.blobs(0);
      Convolution *conv_layer = new Convolution(name, &conv_param, 
                                                &weight_blob, NULL);
      if (conv_param.has_bias_term()) {
        BlobProto bias_blob = layer.blobs(1);
        conv_layer = new Convolution(name, &conv_param, 
                                                  &weight_blob, &bias_blob);
      }
      next_ptr = conv_layer;
    }
    else if (type == RELU) {
      ReLUParameter relu_param = layer.relu_param();
      ReLU *relu_layer = new ReLU(name, &relu_param);
      next_ptr = relu_layer;
    } else if (type == POOLING) {
      PoolingParameter pool_param = layer.pooling_param();
      Pooling *pool_layer = new Pooling(name, &pool_param);
      next_ptr = pool_layer;
    } else if (type == DROPOUT) {
      DropoutParameter drop_param = layer.dropout_param();
      Dropout *dropout_layer = new Dropout(name, &drop_param);
      next_ptr = dropout_layer;
    }

    cout << name << "\t" << next_ptr << endl;
    if (!head)
      head = next_ptr;

    /* Update prev layer's next */
    if (prev_layer)
      prev_layer->next = next_ptr;
    prev_layer = next_ptr;

  }
  cout << "layers building done!" << endl;
  return true;
}
