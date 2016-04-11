#include <iostream>
#include <fstream>
#include "caffe.pb.h"

#include "io_utils.h"

using namespace std;
using namespace caffe;

void 
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

static bool
test_LoadFromTextFile(const char *fpath) 
{
  caffe::NetParameter net_model;

  if (!LoadFromTextFile(fpath, &net_model)) {
    cerr << "test_LoadFromTextFile failed" << endl;
    return false;
  }

  ListLayer(net_model);
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

  ListLayer(net_model);
  return true;
}

int 
main(int argc, char *argv[]) 
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc != 3) {
    cerr << "Usage:  " << argv[0] << " .prototxt .caffemodel" << endl;
    return 1;
  }

  test_LoadFromTextFile(argv[1]);
  test_LoadFromBinaryFile(argv[2]);
}
