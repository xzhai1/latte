#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include "Halide.h"


/**
 * @brief im2col 
 *
 * @param input       unpadded
 * @param kernel_size
 * @param pad
 * @param stride
 *
 * @return 
 */
Halide::Image<float> im2col(Halide::Image<float> input, 
                            int kernel_size, int pad, 
                            int stride);
Halide::Image<float> col2im(Halide::Image<float> input);

#endif /* IMAGE_UTILS_H */
