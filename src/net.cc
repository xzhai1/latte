#include <iostream>
#include <string>

#include "caffe.pb.h"
#include "Halide.h"

#include "CycleTimer.h"
#include "layers.h"
#include "vision_layers.h"
#include "activation_layers.h"
#include "common_layers.h"
#include "loss_layers.h"

#include "net.h"

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

/**
 * @brief build_convlayer 
 *
 * @param layer
 *
 * @return 
 */
static Layer *
build_convlayer(LayerParameter *layer)
{
  ConvolutionParameter conv_param = layer->convolution_param();
  BlobProto weight_blob = layer->blobs(0);
  string name = layer->name();
  Convolution *conv_layer;

  /* Not all convolution layers have bias term */
  if (conv_param.has_bias_term()) {
    BlobProto bias_blob = layer->blobs(1);
    conv_layer = new Convolution(name, &conv_param, &weight_blob, &bias_blob);
  } else {
    conv_layer = new Convolution(name, &conv_param, &weight_blob, NULL);
  }

  return conv_layer;
}

/**
 * @brief build_deconvlayer 
 *
 * @param layer
 *
 * @return 
 */
static Layer *
build_deconvlayer(LayerParameter *layer)
{
  ConvolutionParameter deconv_param = layer->convolution_param();
  BlobProto weight_blob = layer->blobs(0);
  string name = layer->name();
  Deconvolution *deconv_layer;

  /* Not all convolution layers have bias term */
  /* TODO deconv_param.has_bias_term() says true but blobs only has weights */
  deconv_layer = new Deconvolution(name, &deconv_param, &weight_blob, NULL);
  return deconv_layer;
}

/**
 * @brief build_relulayer 
 *
 * @param layer
 *
 * @return 
 */
static Layer *
build_relulayer(LayerParameter *layer)
{
  ReLUParameter relu_param = layer->relu_param();
  string name = layer->name();
  ReLU *relu_layer = new ReLU(name, &relu_param);
  return relu_layer;
}

/**
 * @brief build_poollayer 
 *
 * @param layer
 *
 * @return 
 */
static Layer *
build_poollayer(LayerParameter *layer)
{
  PoolingParameter pool_param = layer->pooling_param();
  string name = layer->name();
  Pooling *pool_layer = new Pooling(name, &pool_param);
  return pool_layer;
}

/**
 * @brief build_dropoutlayer 
 *
 * @param layer
 *
 * @return 
 */
static Layer *
build_dropoutlayer(LayerParameter *layer)
{
  DropoutParameter drop_param = layer->dropout_param();
  string name = layer->name();
  Dropout *dropout_layer = new Dropout(name, &drop_param);
  return dropout_layer;
}

/**
 * @brief Net 
 *
 * @param net_model
 */
Net::Net(NetParameter *net_model)
{
  Layer *prev_layer = NULL;
  Layer *curr_layer = NULL;
  int num_layers = net_model->layer_size();

  cout << "name \taddr" << endl;

  /* TODO we are treating here */
  for (int i = 3; i < num_layers; i++) {
    LayerParameter layer = net_model->layer(i);
    string name = layer.name();
    string type = layer.type();

    /* TODO hack to indicate we only care about the following layers */
    bool hit = false;

    /* TODO we are ignoring a couple of types here */
    if (type == CONVOLUTION) {
      cout << "hit convolution" << endl;
      curr_layer = build_convlayer(&layer);
      hit = true;
      cout << "finish processing convolution" << endl;
    } else if (type == DECONVOLUTION) {
      cout << "hit deconv" << endl;
      curr_layer = build_deconvlayer(&layer);
      hit = true;
      cout << "finish processing deconv" << endl;
    } else if (type == RELU) {
      curr_layer = build_relulayer(&layer);
      hit = true;
    } else if (type == POOLING) {
      curr_layer = build_poollayer(&layer);
      hit = true;
    } else if (type == DROPOUT) {
      curr_layer = build_dropoutlayer(&layer);
      hit = true;
    }

    if (hit) {
      cout << name << "\t" << curr_layer << endl;
      /* On entry, update head */
      if (!head)
        head = curr_layer;

      /* Update prev layer's next */
      if (prev_layer)
        prev_layer->set_next(curr_layer);
      prev_layer = curr_layer;
    }
  }
}

void
Net::print_net()
{
  cout << "Network has the following layers" << endl;
  cout << "name" << "\t\t\t" << "type" << endl;
  for (Layer *ptr = head; ptr != NULL; ptr = ptr->get_next()) {
    cout << ptr->get_name() << "\t\t\t" << ptr->get_type() << endl;
  }
}

Image<float>
Net::run(Image<float> input)
{
  /* TODO each layer is supposed to call the next layer's run */
  Image<float> prev_output = input;
  Image<float> curr_output;
  for (Layer *ptr = head; ptr != NULL; ptr = ptr->get_next()) {
    cout << "passing volume into [" 
      << ptr->get_name() << "," << ptr->get_type() << "]  " << endl;

    double startTime = CycleTimer::currentSeconds();
    curr_output = ptr->run(prev_output);
    double endTime = CycleTimer::currentSeconds();
    cout << "time elapsed: " << (endTime - startTime) * 1000 << " ms  " << endl;
    prev_output = curr_output;
  }

  return curr_output;
}

} /* namespace Latte */
