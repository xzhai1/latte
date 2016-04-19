#include <iostream>

#include "caffe.pb.h"

#include "tests.h"

using namespace std;
using namespace caffe;

int 
main(int argc, char *argv[]) 
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  /* TODO use getopt to parse more sophisticated command line options */
  if (argc != 4) {
    cerr << "Usage:  " << argv[0] 
      << "test_image.png train_val.prototxt trained.caffemodel" << endl;
    return 1;
  }

  string image_path         = argv[1];
  string train_val_path     = argv[2];
  string trained_model_path = argv[3];

  NetParameter net_model;

  //test_LoadFromTextFile(train_val_path, &net_model);
  test_LoadFromBinaryFile(trained_model_path, &net_model);

  // test_convolution(image_path, &net_model);
  // test_deconvolution(&net_model);
  test_net(image_path, &net_model);
  
  // test_im2col(image_path);

  return 0;
}
