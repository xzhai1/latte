#ifndef TESTS_H
#define TESTS_H

#include <string>

#include "Halide.h"
#include "caffe.pb.h"

/**
 * @brief A collection of tests to we can run to validate each functionality
 */


/* test_io_utils.cc */
bool test_LoadFromTextFile(std::string fpath, caffe::NetParameter *net_model);
bool test_LoadFromBinaryFile(std::string fpath, caffe::NetParameter *net_model);

/* test_convlayer.cc */
bool test_convolution(std::string image_path, caffe::NetParameter *net_model);

/* test_deconvlayer.cc */
bool test_deconvolution(caffe::NetParameter *net_model);

/* test_net.cc */
bool test_net(std::string image_path, caffe::NetParameter *net_model);

/* test_img2col.cc */
bool test_im2col(std::string fpath);

#endif /* TESTS_H */
