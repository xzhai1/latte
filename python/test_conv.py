import sys
import argparse

import numpy as np
from PIL import Image
import time

def main(args):
    caffe.set_mode_cpu()

    # Create the network
    net = caffe.Net(args.deploy_prototxt, args.trained_caffemodel, caffe.TEST)

    # create transformer for the input called 'data'
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
    transformer.set_transpose('data', (2,0,1))      # move image channels to outermost dimension
    #transformer.set_mean('data', mu)                # subtract the dataset-mean value in each channel
    transformer.set_raw_scale('data', 255)          # rescale from [0, 1] to [0, 255]
    transformer.set_channel_swap('data', (2,1,0))   # swap channels from RGB to BGR

    net.blobs['data'].reshape(args.batch,       # batch size
                              3,                # 3-channel (BGR) images
                              500, 500)         # image size is 227x227
    image = caffe.io.load_image(args.image)
    transformed_image = transformer.preprocess('data', image)

    # copy the image data into the memory allocated for the net
    for i in range(args.batch):
        net.blobs['data'].data[i,:,:,:] = transformed_image

    start = time.time()
    net.forward(start='input', end='conv1_1')

    #net.forward()

    end = time.time()
    print end - start, "seconds"

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--caffe_python',
                        help='path/to/caffe/python',
                        required=True)
    parser.add_argument('-d', '--deploy_prototxt',
                        help='path to deploy.prototxt',
                        required=True)
    parser.add_argument('-t', '--trained_caffemodel',
                        help='path to model.caffemodel',
                        required=True)
    parser.add_argument('-b', '--batch',
                        type=int,
                        help='batch size',
                        required=True)
    # TODO for now, just duplicate the image
    parser.add_argument('-i', '--image',
                        help='path to test image',
                        required=True)
    args = parser.parse_args()

    # Make sure we can find the caffe python interface before importing it
    sys.path.insert(0, args.caffe_python)
    import caffe

    main(args)
