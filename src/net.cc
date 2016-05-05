#include <iostream>
#include <string>

#include <glog/logging.h>

#include "CycleTimer.h"
#include "caffe.pb.h"
#include "Halide.h"

#include "layers/layers.h"
#include "layers/vision_layers.h"
#include "layers/activation_layers.h"
#include "layers/common_layers.h"
#include "layers/loss_layers.h"

#include "net.h"

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

class Data : public Layer {
  public:
    Data(std::string name, int width, int height, int channels, int num)
      :Layer(name, DATA) {
        set_width(width);
        set_height(height);
        set_channels(channels);
        set_num(num);
        Image<float> dummy(width, height, channels, num);
        storage = Func(dummy);
    }

    void SetData(Image<float> image) {
      if (get_width()    != image.extent(0) ||
          get_height()   != image.extent(1) ||
          get_channels() != image.extent(2) ||
          get_num()      != image.extent(3)) {
        LOG(FATAL) << "Input dimension is not compatible";
      }
      storage = Func(image);
    }
};

static Layer *
build_convlayer(LayerParameter *layer, Layer *prev)
{
  ConvolutionParameter conv_param = layer->convolution_param();
  BlobProto weight_blob = layer->blobs(0);
  string name = layer->name();
  Convolution *conv_layer;

  /* Not all convolution layers have bias term */
  if (conv_param.has_bias_term()) {
    BlobProto bias_blob = layer->blobs(1);
    conv_layer = new Convolution(name, prev, 
        &conv_param, &weight_blob, &bias_blob);
  } else {
    conv_layer = new Convolution(name, prev, 
        &conv_param, &weight_blob, NULL);
  }

  return conv_layer;
}

static Layer *
build_deconvlayer(LayerParameter *layer, Layer *prev)
{
  ConvolutionParameter deconv_param = layer->convolution_param();
  BlobProto weight_blob = layer->blobs(0);
  string name = layer->name();
  Deconvolution *deconv_layer;

  /* Not all convolution layers have bias term */
  /* TODO deconv_param.has_bias_term() says true but blobs only has weights */
  deconv_layer = new Deconvolution(name, prev, 
      &deconv_param, &weight_blob, NULL);
  return deconv_layer;
}

static Layer *
build_relulayer(LayerParameter *layer, Layer *prev)
{
  ReLUParameter relu_param = layer->relu_param();
  string name = layer->name();
  ReLU *relu_layer = new ReLU(name, prev, &relu_param);
  return relu_layer;
}

static Layer *
build_poollayer(LayerParameter *layer, Layer *prev)
{
  PoolingParameter pool_param = layer->pooling_param();
  string name = layer->name();
  Pooling *pool_layer = new Pooling(name, prev, &pool_param);
  return pool_layer;
}

#if 0
static Layer *
build_dropoutlayer(LayerParameter *layer)
{
  DropoutParameter drop_param = layer->dropout_param();
  string name = layer->name();
  Dropout *dropout_layer = new Dropout(name, &drop_param);
  return dropout_layer;
}

static Layer *
build_softmaxlayer(LayerParameter *layer)
{
  string name = layer->name();
  Softmax *softmax_layer = new Softmax(name);
  return softmax_layer;
}
#endif

Net::Net(NetParameter *net_model)
{
  int count = 0;
  Layer *prev_layer = NULL;
  /* First layer is DATA */
  /* TODO change hard code */
  Layer *curr_layer = new Data("Dummy", 500, 500, 3, 1);
  data = (Data *)curr_layer;
  int num_layers = net_model->layer_size();

  for (int i = 0; i < num_layers; i++) {
    LayerParameter layer = net_model->layer(i);
    string name = layer.name();
    string type = layer.type();

    /* TODO hack to indicate we only care about the following layers */
    bool hit = false;

    if (type == CONVOLUTION) {
      count++;
      if (count == 5) break;
      curr_layer = build_convlayer(&layer, curr_layer);
      hit = true;
    }
    else if (type == DECONVOLUTION) {
      curr_layer = build_deconvlayer(&layer, curr_layer);
      hit = true;
    }
    else if (type == RELU) {
      //count++;
      //if (count == 14) break;
      curr_layer = build_relulayer(&layer, curr_layer);
      hit = true;
    } else if (type == POOLING) {
      curr_layer = build_poollayer(&layer, curr_layer);
      hit = true;
    } 
    #if 0
    else if (type == DROPOUT) {
      curr_layer = build_dropoutlayer(&layer);
      hit = true;
    } 
    else if (type == SOFTMAX) {
      curr_layer = build_softmaxlayer(&layer);
      hit = true;
    }
    #endif
  
    if (hit) {
      //cout << name << "\t" << curr_layer << endl;
      /* On entry, update head */
      if (!head)
        head = curr_layer;

      /* Update prev layer's next */
      if (prev_layer)
        prev_layer->set_next(curr_layer);
      prev_layer = curr_layer;

      // if (type == CONVOLUTION) break;
      //if (count == 14) break;
    }
  }
  tail = curr_layer;
}

void
Net::print_net()
{
  cout << "--------------------------------------------" << endl;
  cout << "Network has the following layers" << endl;
  cout << "--------------------------------------------" << endl;
  cout << "name" << "\t\t\t" << "type" << endl;
  for (Layer *ptr = head; ptr != NULL; ptr = ptr->get_next()) {
    cout << ptr->get_name() << "\t\t\t" << ptr->get_type() << endl;
  }
  cout << "--------------------------------------------" << endl;
}

Image<float>
Net::run(Image<float> input)
{
  /* Display input image dimension */
  cout << "Input dimension [W, H, C, N]:"
       << input.extent(0) << ", "
       << input.extent(1) << ", "
       << input.extent(2) << ", "
       << input.extent(3)
       << endl;
  ((Data *)data)->SetData(input);

  double inferenceStartTime, inferenceEndTime, startTime, endTime;
  inferenceStartTime = CycleTimer::currentSeconds();
  Image<float> output = tail->storage.realize(tail->get_width(), 
                                              tail->get_height(), 
                                              tail->get_channels(), 
                                              tail->get_num());
  inferenceEndTime = CycleTimer::currentSeconds();
  cout << "Inference time: " 
       << (inferenceEndTime - inferenceStartTime) * 1000 << " ms  " << endl;

  return output;
}

} /* namespace Latte */
