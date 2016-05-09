#include <string>

#include <glog/logging.h>
#include "CycleTimer.h"
#include "caffe.pb.h"
#include "Halide.h"

#include "layers/layers.h"
#include "layers/vision_layers.h"
#include "layers/activation_layers.h"
#include "layers/common_layers.h"
//#include "layers/loss_layers.h"

#include "net.h"

namespace Latte {

using namespace std;
using namespace caffe;
using namespace Halide;

#if 0
/**
 * @brief Data layer is just a dummy layer to hold the image that comes in so
 * we can bootstrap the whole net
 */
class Data : public Layer {
  public:
    Data(std::string name, 
         int width, 
         int height, 
         int channels, 
         int num, 
         ImageParam img)
      :Layer(name, DATA, img) {
        set_output_dim(width, height, channels, num);
        storage(i, j, k, l) = img(i, j, k, l);
    }
};
#endif

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

static Layer *
build_croplayer(LayerParameter *layer, 
                Layer *prev, 
                int input_width, 
                int input_height)
{
  CropParameter crop_param = layer->crop_param();
  string name = layer->name();
  Crop *crop_layer = new Crop(name, prev, &crop_param, 
      input_width, input_height);
  return crop_layer;
}

Net::Net(NetParameter *net_model, Image<float> img)
{
  int count = 0;
  Layer *prev_layer = NULL;

  /* First layer is DATA */
  ImageParam input(type_of<float>(), 4);
  input.set(img);
  Layer *curr_layer = 
    new Data("Data", 
             img.extent(0), 
             img.extent(1), 
             img.extent(2), 
             img.extent(3), 
             input);

  /* remeber the input size */
  input_width = img.extent(0);
  input_height = img.extent(1);

  int num_layers = net_model->layer_size();
  for (int i = 0; i < num_layers; i++) {
    LayerParameter layer = net_model->layer(i);
    string name = layer.name();
    string type = layer.type();

    /* TODO hack to indicate we only care about the following layers */
    bool hit = false;

    if (type == CONVOLUTION) {
      // count++;
      //if (count == 5) break;
      curr_layer = build_convlayer(&layer, curr_layer);
      hit = true;
    }
    else if (type == DECONVOLUTION) {
      curr_layer = build_deconvlayer(&layer, curr_layer);
      hit = true;
    }
    else if (type == RELU) {
      // count++;
      //if (count == 14) break;
      curr_layer = build_relulayer(&layer, curr_layer);
      hit = true;
    } else if (type == POOLING) {
      // count++;
      curr_layer = build_poollayer(&layer, curr_layer);
      hit = true;
    } else if (type == CROP) {
      curr_layer = build_croplayer(
          &layer, curr_layer, input_width, input_height);
      hit = true;
    }

    if (hit) {
      count++;

      /* On entry, update head */
      if (!head)
        head = curr_layer;

      /* Update prev layer's next */
      if (prev_layer)
        prev_layer->set_next(curr_layer);
      prev_layer = curr_layer;
    }
  }
  tail = curr_layer;
}

void
Net::PrintNet()
{
  LOG(INFO) << "--------------------------------------------";
  LOG(INFO) << "|     Network has the following layers     |";
  LOG(INFO) << "--------------------------------------------";
  LOG(INFO) << "Name" << "\t\t\t" << "Type";
  LOG(INFO) << "--------------------------------------------";
  for (Layer *ptr = head; ptr != NULL; ptr = ptr->get_next()) {
    if (ptr->get_name().length() >= 8)
      LOG(INFO) << ptr->get_name() << "\t\t" << ptr->get_type();
    else
      LOG(INFO) << ptr->get_name() << "\t\t\t" << ptr->get_type();
  }
  LOG(INFO) << "--------------------------------------------";
}

Image<float>
Net::Run(Image<float> input, int iterations)
{
  double inferenceStartTime, inferenceEndTime, startTime, endTime;
  startTime = CycleTimer::currentSeconds();
  /* Display input image dimension */
  LOG(INFO) << "Input dimension W x H x C x N : "
            << input.extent(0) << " x "
            << input.extent(1) << " x "
            << input.extent(2) << " x "
            << input.extent(3);
  tail->storage.compile_jit();
#if 0
  /* compile in GPU */
  Target target = get_host_target();
  target.set_feature(Target::CUDA);
  tail->storage.compile_jit(target);
#endif
  endTime = CycleTimer::currentSeconds();

  LOG(INFO) << "Compile time: " << (endTime - startTime)*1000 << " ms";

  /* Timing for inference */
  Image<float> output;
  /* Buffer for GPU timing */
#if 0
  Buffer output_buffer(Float(32), tail->get_width(),
                                  tail->get_height(),
                                  tail->get_channels(),
                                  tail->get_batchsize());
#endif

  double duration = 1.f*24*60*60;
  for (int t = 0; t < iterations; t++) {
    inferenceStartTime = CycleTimer::currentSeconds();
    output = tail->storage.realize(tail->get_width(), 
                                   tail->get_height(), 
                                   tail->get_channels(), 
                                   tail->get_batchsize());

#if 0
    /* GPU realize */
    tail->storage.realize(output_buffer);
#endif
    inferenceEndTime = CycleTimer::currentSeconds();
    double currentDuration = inferenceEndTime - inferenceStartTime;
    if (currentDuration < duration)
      duration = currentDuration;
  }
  LOG(INFO) << "Inference time: " << duration*1000 << " ms";
  //cout << duration * 1000 << endl;

  return output;
}

} /* namespace Latte */
