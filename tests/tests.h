#ifndef TESTS_H
#define TESTS_H

#include <string>

#include "Halide.h"
#include "caffe.pb.h"

/* test_io_utils.cc */
bool TestLoadFromTextFile(std::string fpath, caffe::NetParameter *net_model);
bool TestLoadFromBinaryFile(std::string fpath, caffe::NetParameter *net_model);

/* test_net.cc */
bool TestNet(std::string image_path, caffe::NetParameter *net_model, 
             int batch_size, int iterations);

#endif /* TESTS_H */
