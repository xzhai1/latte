#include <sys/stat.h>      /* stat */

#include <gflags/gflags.h> /* Google's commandline parser */
#include <glog/logging.h>  /* Google's logging module */

#include "caffe.pb.h"
#include "tests.h"         /* Collection of tests */

using namespace std;
using namespace google;
using namespace gflags;
using namespace caffe;

string usage = "\n ./test "
                      "\t--stderrthreshold=LOGLEVEL\n"
                      "\t--image_path \t\timage.png\n" 
                      "\t--train_val_path \ttrain_val.prototxt\n"
                      "\t--trained_model_path \ttrained_model.caffemodel\n"
                      "\t--iterations 1\n"
                      "\t--batch_size 1\n"
                      "\t--test_loadfromtest";

/**
 * @brief Try to read the file passed in at the command line
 *
 * @param flagname
 * @param path
 *
 * @return false if nonexistent 
 */
static bool
ValidatePath(const char *flagname, const string& path)
{
  struct stat buffer;   
  if (!stat(path.c_str(), &buffer))
    return true; 
  LOG(INFO) << "Invalid path for " << flagname;
  return false;
}

/* Add command line arguements here and validators above */
DEFINE_string(image_path, "", "Path of test image");
DEFINE_string(train_val_path, "", 
              "Path of *train_val.prototxt which defines"
              " the network architecture for training and validation");
DEFINE_string(trained_model_path, "", 
              "Path of *.caffemodel, trained weights in binary format" 
              " serialized by protobuf");
DEFINE_uint64(iterations, 1, 
              "Number of interations to run the network");
DEFINE_uint64(batch_size, 1, 
              "Number of image in the whole batch");
DEFINE_bool(test_loadfromtext, false,
              "Test loading network model from .prototxt");
DEFINE_bool(test_net, false,
              "Run the whole network using the image and weights supplied.");

int 
main(int argc, char *argv[]) 
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  /* Initialize Google's logging library */
  InitGoogleLogging(argv[0]);

  /* Register validator and help message */
  RegisterFlagValidator(&FLAGS_image_path, &ValidatePath);
  RegisterFlagValidator(&FLAGS_train_val_path, &ValidatePath);
  RegisterFlagValidator(&FLAGS_trained_model_path, &ValidatePath);
  SetUsageMessage(usage);

  /* Parse commandline */
  ParseCommandLineFlags(&argc, &argv, true);
  string image_path         = FLAGS_image_path;
  string train_val_path     = FLAGS_train_val_path;
  string trained_model_path = FLAGS_trained_model_path;

  /* Make room at compile time */
  NetParameter net_model;

  /* the prototxt is really useless b/c it doesn't include the weights
   * nor does it offer additional info over the .caffemodel file so we are
   * overwriting the net_model variable */
  if (FLAGS_test_loadfromtext)
    TestLoadFromTextFile(train_val_path, &net_model);
  
  /* The model has to be loaded no matter what and  */
  TestLoadFromBinaryFile(trained_model_path, &net_model);

  /* Run the whole net */
  TestNet(image_path, &net_model, FLAGS_batch_size, FLAGS_iterations);

  return 0;
}
