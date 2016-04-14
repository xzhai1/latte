#ifndef TESTS_H
#define TESTS_H

#include <string>

#include "caffe.pb.h"

/* test_io_utils */
bool test_LoadFromTextFile(std::string fpath, caffe::NetParameter *net_model);
bool test_LoadFromBinaryFile(std::string fpath, caffe::NetParameter *net_model);

/* test_convlayer */
bool test_convolution(std::string image_path, caffe::NetParameter *net_model);

#endif /* TESTS_H */
