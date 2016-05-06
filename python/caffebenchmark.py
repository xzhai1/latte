import sys
import argparse
import pickle

import numpy as np
import time

def main(args):
    caffe.set_mode_cpu()

    # Create the network
    net = caffe.Net(args.deploy_prototxt, args.trained_caffemodel, caffe.TEST)

    # create transformer for the input called 'data'
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
    # move image channels to outermost dimension
    transformer.set_transpose('data', (2,0,1))     
    
    # subtract the dataset-mean value in each channel
    #transformer.set_mean('data', mu)

    # rescale from [0, 1] to [0, 255]
    #transformer.set_raw_scale('data', 255)
    # swap channels from RGB to BGR
    transformer.set_channel_swap('data', (2,1,0))
    
    image = caffe.io.load_image(args.image)
    transformed_image = transformer.preprocess('data', image)
    shape = image.shape

    net.blobs['data'].reshape(args.batch,       # batch size
                              shape[2],         # C
                              shape[1],         # H 
                              shape[0])         # W 

    # copy the image data into the memory allocated for the net
    for i in range(args.batch):
        net.blobs['data'].data[i,:,:,:] = transformed_image

    # Ignore the input data layers
    layers = [k for k, _ in net.blobs.items()][3:]

    for layer in layers:
        start = time.time()
        output = net.forward(start='input', end=layer)
        #output = net.forward()
        end = time.time()
        print layer, (end - start)*1000, "ms"

    print output[net.outputs[0]].shape
    pickle.dump(output[net.outputs[0]], open('caffe_result.pkl', 'wb'))

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
