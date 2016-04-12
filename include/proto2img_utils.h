#ifndef PROTO2IMG_UTILS_H
#define PROTO2IMG_UTILS_H  

#include "Halide.h"
#include "caffe.pb.h"

using namespace Halide; /* Image */
using namespace caffe; /* Blobproto */

/**
 * @brief LoadKernelFromBlob 
 *
 * The conventional blob dimensions for batches of image data are number N x
 * channel K x height H x width W. Blob memory is row-major in layout, so the
 * last / rightmost dimension changes fastest. For example, in a 4D blob, the
 * value at index (n, k, h, w) is physically located at index 
 * ((n * K + k) * H + h) * W + w.
 *
 * These blobs actually have no shape. So to create the actual kernels, we need
 * to look at the "z" dimension of the previous layer, size of the kernel of
 * current layer, and the "z" dimension of the current layer
 
 * @param blob
 * @param ksize
 * @param curr_depth
 *
 * @return 
 */
Image<float> LoadKernelFromBlob(const BlobProto *blob, 
                                int k_size, int num_output);

#endif /* PROTO2IMG_UTILS_H */
