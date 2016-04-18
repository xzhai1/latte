#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include "Halide.h"

Halide::Image<float> im2col(Halide::Image<float> input, 
                            int kernel_size, int stride);
Halide::Image<float> col2im(Halide::Image<float> input);

#endif /* IMAGE_UTILS_H */
