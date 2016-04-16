# latte
CMU 15-418/618 Final Project: Implementing Fully Convolutional Network using Halide and evaluate against Caffe version

## Build Procedure
The following steps are necessary for a brand new machine; some might be unnecessary for you.

### Utilities
Make sure you have the build tool chain:

    sudo apt-get install build-essential

We are going to use ``libpng``:

    sudo apt-get install libpng12-dev

You are going to need ``autoreconf`` for protobuf later:

    sudo apt-get install dh-autoreconf
    
### ``protobuf``
Download ``protobuf``:

    wget https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz
    tar -xvf protobuf-2.6.1.tar.gz
    rm protobuf-2.6.1.tar.gz
    cd protobuf-2.6.1
    
To compile and install it, you can follow the instruction [here](https://github.com/google/protobuf/tree/master/src). But the gist is this:

    ./autogen.sh
	./configure
	make
	make check
	sudo make install
	sudo ldconfig
	
### Halide
Check your ``g++`` version:

    g++ --version
    
and download the right version (trunk) of Halide. Ours is 4.8.4 so we did the following:

    wget https://github.com/halide/Halide/releases/download/release_2016_03_02/halide-linux-64-gcc48-trunk-65bbac2967ebd59994e613431fd5236baf8a5829.tgz
    tar -xvf halide-linux-64-gcc48-trunk-65bbac2967ebd59994e613431fd5236baf8a5829.tgz
    rm halide-linux-64-gcc48-trunk-65bbac2967ebd59994e613431fd5236baf8a5829.tgz

## Build
Clone the project:

    git clone https://github.com/xzhai1/latte.git
    
Your directory structure should look like this:

    xd@xd-Standard-PC-i440FX-PIIX-1996:~/Documents$ ls
    halide  latte  protobuf-2.6.1

Then go into the repo:
    
    cd latte
    
You will need to download the whole CNN and the trained caffemodel:

    mkdir model
    wget http://dl.caffe.berkeleyvision.org/fcn-32s-pascalcontext.caffemodel
    wget https://gist.githubusercontent.com/shelhamer/80667189b218ad570e82/raw/077494f215421b3d9383e1b1a3d75377344b1744/train_val.prototxt
    
You can now finally build the project:

    make

and fingers crossed, it won't throw an error. Then you can run a test:

    LD_LIBRARY_PATH=../halide/bin/ ./run_test images/rgb.png model/train_val.prototxt model/fcn-32s-pascalcontext.caffemodel    

or

    LD_LIBRARY_PATH=../halide/bin/:/usr/local/lib ./run_test images/rgb.png model/train_val.prototxt model/fcn-32s-pascalcontext.caffemodel

## Preliminary Results
### Single core single thread  
-------------------------------------------------
Run layers (test foward pass without parallelism)  
passing volume into [conv1_1,Convolution]  
	 time elapsed: 1998.63 ms  
passing volume into [relu1_1,ReLU]  
	 time elapsed: 99.1048 ms  
passing volume into [conv1_2,Convolution]  
	 time elapsed: 77089.7 ms  
passing volume into [relu1_2,ReLU]  
	 time elapsed: 98.4301 ms  
passing volume into [pool1,Pooling]  
	 time elapsed: 93.7618 ms  
passing volume into [conv2_1,Convolution]  
	 time elapsed: 34400.8 ms  
passing volume into [relu2_1,ReLU]  
	 time elapsed: 70.2868 ms  
passing volume into [conv2_2,Convolution]  
	 time elapsed: 69707.4 ms  
passing volume into [relu2_2,ReLU]  
	 time elapsed: 70.4011 ms  
passing volume into [pool2,Pooling]  
	 time elapsed: 80.3297 ms  
passing volume into [conv3_1,Convolution]  
	 time elapsed: 34853.4 ms  
passing volume into [relu3_1,ReLU]  
	 time elapsed: 55.7537 ms  
passing volume into [conv3_2,Convolution]  
	 time elapsed: 75795.2 ms  
passing volume into [relu3_2,ReLU]  
	 time elapsed: 57.0164 ms  
passing volume into [conv3_3,Convolution]  
	 time elapsed: 75795.5 ms  
passing volume into [relu3_3,ReLU]  
	 time elapsed: 55.5363 ms  
passing volume into [pool3,Pooling]  
	 time elapsed: 70.3825 ms  
passing volume into [conv4_1,Convolution]  
	 time elapsed: 35511.1 ms  
passing volume into [relu4_1,ReLU]  
	 time elapsed: 48.266 ms  
passing volume into [conv4_2,Convolution]  
	 time elapsed: 70100.3 ms  
passing volume into [relu4_2,ReLU]  
	 time elapsed: 48.2869 ms  
passing volume into [conv4_3,Convolution]  
	 time elapsed: 70105.7 ms  
passing volume into [relu4_3,ReLU]  
	 time elapsed: 48.9969 ms  
passing volume into [pool4,Pooling]  
	 time elapsed: 64.3873 ms  
passing volume into [conv5_1,Convolution]  
	 time elapsed: 17960.7 ms  
passing volume into [relu5_1,ReLU]  
	 time elapsed: 44.1457 ms  
passing volume into [conv5_2,Convolution]  
	 time elapsed: 17962.7 ms  
passing volume into [relu5_2,ReLU]  
	 time elapsed: 43.6523 ms  
passing volume into [conv5_3,Convolution]  
	 time elapsed: 17956.2 ms  
passing volume into [relu5_3,ReLU]  
	 time elapsed: 43.4831 ms  
passing volume into [pool5,Pooling]  
	 time elapsed: 61.0401 ms  
passing volume into [fc6,Convolution]  
	 time elapsed: 134015 ms  
passing volume into [relu6,ReLU]  
	 time elapsed: 45.0037 ms  
passing volume into [drop6,Dropout]  
	 time elapsed: 44.8804 ms  
passing volume into [fc7,Convolution]  
	 time elapsed: 37165.8 ms  
passing volume into [relu7,ReLU]  
	 time elapsed: 44.3224 ms  
passing volume into [drop7,Dropout]  
	 time elapsed: 44.9542 ms  
passing volume into [score59,Convolution]  
	 time elapsed: 989.487 ms  
-------------------------------------------------

### CPU parallelism
-------------------------------------------------



-------------------------------------------------

### OpenCL
-------------------------------------------------
passing volume into [conv1_1,Convolution]  
time elapsed: 1032.42 ms  
passing volume into [relu1_1,ReLU]  
time elapsed: 101.37 ms  
passing volume into [conv1_2,Convolution]  
time elapsed: 4389.71 ms  
passing volume into [relu1_2,ReLU]  
time elapsed: 100.841 ms  
passing volume into [pool1,Pooling]  
time elapsed: 96.5304 ms  
passing volume into [conv2_1,Convolution]  
time elapsed: 2228.97 ms  
passing volume into [relu2_1,ReLU]  
time elapsed: 71.9554 ms  
passing volume into [conv2_2,Convolution]  
time elapsed: 4255.55 ms  
passing volume into [relu2_2,ReLU]  
time elapsed: 72.2487 ms  
passing volume into [pool2,Pooling]  
time elapsed: 83.1646 ms  
passing volume into [conv3_1,Convolution]  
time elapsed: 2214.08 ms  
passing volume into [relu3_1,ReLU]  
time elapsed: 57.7185 ms  
passing volume into [conv3_2,Convolution]  
time elapsed: 4272.59 ms  
passing volume into [relu3_2,ReLU]  
time elapsed: 57.716 ms  
passing volume into [conv3_3,Convolution]  
time elapsed: 4279.81 ms  
passing volume into [relu3_3,ReLU]  
time elapsed: 58.4694 ms  
passing volume into [pool3,Pooling]  
time elapsed: 72.2867 ms  
passing volume into [conv4_1,Convolution]  
time elapsed: 2119.28 ms  
passing volume into [relu4_1,ReLU]  
time elapsed: 49.8859 ms  
passing volume into [conv4_2,Convolution]  
time elapsed: 4113.83 ms  
passing volume into [relu4_2,ReLU]  
time elapsed: 49.8044 ms  
passing volume into [conv4_3,Convolution]  
time elapsed: 4121.41 ms  
passing volume into [relu4_3,ReLU]  
time elapsed: 49.652 ms  
passing volume into [pool4,Pooling]  
time elapsed: 67.768 ms  
passing volume into [conv5_1,Convolution]  
time elapsed: 1117.64 ms  
passing volume into [relu5_1,ReLU]  
time elapsed: 45.4795 ms  
passing volume into [conv5_2,Convolution]  
time elapsed: 1117.36 ms  
passing volume into [relu5_2,ReLU]  
time elapsed: 45.9007 ms  
passing volume into [conv5_3,Convolution]  
time elapsed: 1118.67 ms  
passing volume into [relu5_3,ReLU]  
time elapsed: 45.7291 ms  
passing volume into [pool5,Pooling]  
time elapsed: 63.8651 ms  
passing volume into [fc6,Convolution]  
time elapsed: 7423.54 ms  
passing volume into [relu6,ReLU]  
time elapsed: 46.9669 ms  
passing volume into [drop6,Dropout]  
time elapsed: 47.2789 ms  
passing volume into [fc7,Convolution]  
time elapsed: 1136.28 ms  
passing volume into [relu7,ReLU]  
time elapsed: 46.6917 ms  
passing volume into [drop7,Dropout]  
time elapsed: 47.3623 ms  
passing volume into [score59,Convolution]  
time elapsed: 125.551 ms  
__total time elapsed: 46457.6 ms__  


-------------------------------------------------

### CUDA
-------------------------------------------------



-------------------------------------------------
