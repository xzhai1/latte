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
    cd model
    wget http://dl.caffe.berkeleyvision.org/fcn-32s-pascalcontext.caffemodel
    wget https://gist.githubusercontent.com/shelhamer/80667189b218ad570e82/raw/077494f215421b3d9383e1b1a3d75377344b1744/train_val.prototxt
    wget https://gist.githubusercontent.com/shelhamer/80667189b218ad570e82/raw/077494f215421b3d9383e1b1a3d75377344b1744/deploy.prototxt
    
You can now finally build the project:

    make
    
Faster make can be done by

    make -j8

and fingers crossed, it won't throw an error. Then you can run a test:

    LD_LIBRARY_PATH=../halide/bin/ ./run_test images/rgb.png model/train_val.prototxt model/fcn-32s-pascalcontext.caffemodel    

or

    LD_LIBRARY_PATH=../halide/bin/:/usr/local/lib ./run_test images/rgb.png model/train_val.prototxt model/fcn-32s-pascalcontext.caffemodel

## Install Caffe for Benchmarking
Installing caffe is not exactly a cup of caffe...

First, dependencies:

    sudo apt-get install libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev libatlas-base-dev gfortran libgflags-dev libgoogle-glog-dev liblmdb-dev
    sudo apt-get install --no-install-recommends libboost-all-dev
    
If you are comparing that list with caffe's official instructions, you will see we aren't installing ``libprotobuf-dev`` and  ``protobuf-compiler`` because we already build them from source.

Next, clone the source code:

    git clone https://github.com/BVLC/caffe.git
    
and copy the example config:

    cp Makefile.config.example Makefile.config
    
Then modify ``Makefile.config``. For example, I am doing this on my laptop that has no GPU and I want the python layer build so I can quickly push an image through the net. I uncommented the following lines:

    CPU_ONLY := 1
    WITH_PYTHON_LAYER := 1
    
Build. My laptop has 4 logical cores, so:
    
    make all -j4
    make test
    make runtest

I want to the python binding too:

    make pycaffe

Then install all the python dependencies if you want to use python:

    cd python
    for req in $(cat requirements.txt); do sudo pip install $req; done
    
I wondered why the official documentation used the shell script thing. I tried to use ``sudo pip install -r requirements.txt`` with no sucess.

When that is done, you need to append the path of python directory to the ``PYTHONPATH``:

    export PYTHONPATH=/path/to/caffe/python:$PYTHONPATH
    
Just so you know, ``/path/to/caffe/python`` contains the ``_caffe.so`` file in the ``caffe`` directory. 

You also need to let the system know where your ``libcaffe.so`` is:

    export LD_LIBRARY_PATH=/path/to/build/libcaffe.so:$LD_LIBRARY_PATH

Now, you can clone the FCN repo:

    git clone https://github.com/shelhamer/fcn.berkeleyvision.org.git

## Preliminary Results
### Single core single thread  
-------------------------------------------------
passing volume into [conv1_1,Convolution]  
time elapsed: 1997.32 ms  
passing volume into [relu1_1,ReLU]  
time elapsed: 98.9461 ms  
passing volume into [conv1_2,Convolution]  
time elapsed: 77310.3 ms  
passing volume into [relu1_2,ReLU]  
time elapsed: 99.2903 ms  
passing volume into [pool1,Pooling]  
time elapsed: 94.4794 ms  
passing volume into [conv2_1,Convolution]  
time elapsed: 34409.3 ms  
passing volume into [relu2_1,ReLU]  
time elapsed: 70.1058 ms  
passing volume into [conv2_2,Convolution]  
time elapsed: 69834.6 ms  
passing volume into [relu2_2,ReLU]  
time elapsed: 69.3546 ms  
passing volume into [pool2,Pooling]  
time elapsed: 79.8739 ms  
passing volume into [conv3_1,Convolution]  
time elapsed: 34895.1 ms  
passing volume into [relu3_1,ReLU]  
time elapsed: 56.4296 ms  
passing volume into [conv3_2,Convolution]  
time elapsed: 75918.1 ms  
passing volume into [relu3_2,ReLU]  
time elapsed: 55.0539 ms  
passing volume into [conv3_3,Convolution]  
time elapsed: 76071.3 ms  
passing volume into [relu3_3,ReLU]  
time elapsed: 55.741 ms  
passing volume into [pool3,Pooling]  
time elapsed: 69.1565 ms  
passing volume into [conv4_1,Convolution]  
time elapsed: 35555.1 ms  
passing volume into [relu4_1,ReLU]  
time elapsed: 48.3402 ms  
passing volume into [conv4_2,Convolution]  
time elapsed: 70184.4 ms  
passing volume into [relu4_2,ReLU]  
time elapsed: 48.2296 ms  
passing volume into [conv4_3,Convolution]  
time elapsed: 70194.4 ms  
passing volume into [relu4_3,ReLU]  
time elapsed: 47.9176 ms  
passing volume into [pool4,Pooling]  
time elapsed: 64.8116 ms  
passing volume into [conv5_1,Convolution]  
time elapsed: 17980.7 ms  
passing volume into [relu5_1,ReLU]  
time elapsed: 43.5226 ms  
passing volume into [conv5_2,Convolution]  
time elapsed: 17972 ms  
passing volume into [relu5_2,ReLU]  
time elapsed: 43.406 ms  
passing volume into [conv5_3,Convolution]  
time elapsed: 17974.7 ms  
passing volume into [relu5_3,ReLU]  
time elapsed: 43.6008 ms  
passing volume into [pool5,Pooling]  
time elapsed: 60.6082 ms  
passing volume into [fc6,Convolution]  
time elapsed: 134050 ms  
passing volume into [relu6,ReLU]  
time elapsed: 46.0232 ms  
passing volume into [drop6,Dropout]  
time elapsed: 45.6966 ms  
passing volume into [fc7,Convolution]  
time elapsed: 37185.8 ms  
passing volume into [relu7,ReLU]  
time elapsed: 45.7676 ms  
passing volume into [drop7,Dropout]  
time elapsed: 45.0584 ms  
passing volume into [score59,Convolution]  
time elapsed: 726.468 ms  
__total time elapsed: 773599 ms__

-------------------------------------------------

### CPU parallelism
-------------------------------------------------
passing volume into [conv1_1,Convolution]  
time elapsed: 1073.37 ms  
passing volume into [relu1_1,ReLU]  
time elapsed: 105.349 ms  
passing volume into [conv1_2,Convolution]  
time elapsed: 3533.83 ms  
passing volume into [relu1_2,ReLU]  
time elapsed: 104.113 ms  
passing volume into [pool1,Pooling]  
time elapsed: 98.918 ms  
passing volume into [conv2_1,Convolution]  
time elapsed: 2053.85 ms  
passing volume into [relu2_1,ReLU]  
time elapsed: 74.5119 ms  
passing volume into [conv2_2,Convolution]  
time elapsed: 4079.61 ms  
passing volume into [relu2_2,ReLU]  
time elapsed: 75.5188 ms  
passing volume into [pool2,Pooling]  
time elapsed: 87.1797 ms  
passing volume into [conv3_1,Convolution]  
time elapsed: 2133.03 ms  
passing volume into [relu3_1,ReLU]  
time elapsed: 60.324 ms  
passing volume into [conv3_2,Convolution]  
time elapsed: 4027.11 ms  
passing volume into [relu3_2,ReLU]  
time elapsed: 59.0328 ms  
passing volume into [conv3_3,Convolution]  
time elapsed: 4014.86 ms  
passing volume into [relu3_3,ReLU]  
time elapsed: 59.1436 ms  
passing volume into [pool3,Pooling]  
time elapsed: 74.8292 ms  
passing volume into [conv4_1,Convolution]  
time elapsed: 2173.67 ms  
passing volume into [relu4_1,ReLU]  
time elapsed: 50.22 ms  
passing volume into [conv4_2,Convolution]  
time elapsed: 4471.92 ms  
passing volume into [relu4_2,ReLU]  
time elapsed: 50.9201 ms  
passing volume into [conv4_3,Convolution]  
time elapsed: 4477.45 ms  
passing volume into [relu4_3,ReLU]  
time elapsed: 50.8625 ms  
passing volume into [pool4,Pooling]  
time elapsed: 68.437 ms  
passing volume into [conv5_1,Convolution]  
time elapsed: 1567.5 ms  
passing volume into [relu5_1,ReLU]  
time elapsed: 46.8311 ms  
passing volume into [conv5_2,Convolution]  
time elapsed: 1563.75 ms  
passing volume into [relu5_2,ReLU]  
time elapsed: 46.2104 ms  
passing volume into [conv5_3,Convolution]  
time elapsed: 1564.81 ms  
passing volume into [relu5_3,ReLU]  
time elapsed: 46.9002 ms  
passing volume into [pool5,Pooling]  
time elapsed: 64.8444 ms  
passing volume into [fc6,Convolution]  
time elapsed: 4817.26 ms  
passing volume into [relu6,ReLU]  
time elapsed: 47.2458 ms  
passing volume into [drop6,Dropout]  
time elapsed: 47.9491 ms  
passing volume into [fc7,Convolution]  
time elapsed: 4216.37 ms  
passing volume into [relu7,ReLU]  
time elapsed: 47.6139 ms  
passing volume into [drop7,Dropout]  
time elapsed: 48.4362 ms  
passing volume into [score59,Convolution]  
time elapsed: 268.455 ms  
__total time elapsed: 47463 ms__


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
passing volume into [conv1_1,Convolution]  
time elapsed: 1122.85 ms  
passing volume into [relu1_1,ReLU]  
time elapsed: 101.102 ms  
passing volume into [conv1_2,Convolution]  
time elapsed: 4744.04 ms  
passing volume into [relu1_2,ReLU]  
time elapsed: 101.64 ms  
passing volume into [pool1,Pooling]  
time elapsed: 97.39 ms  
passing volume into [conv2_1,Convolution]  
time elapsed: 2482.6 ms  
passing volume into [relu2_1,ReLU]  
time elapsed: 72.5773 ms  
passing volume into [conv2_2,Convolution]  
time elapsed: 4616.55 ms  
passing volume into [relu2_2,ReLU]  
time elapsed: 72.6978 ms  
passing volume into [pool2,Pooling]  
time elapsed: 84.3539 ms  
passing volume into [conv3_1,Convolution]  
time elapsed: 2466.05 ms  
passing volume into [relu3_1,ReLU]  
time elapsed: 57.9958 ms  
passing volume into [conv3_2,Convolution]  
time elapsed: 4626.17 ms  
passing volume into [relu3_2,ReLU]  
time elapsed: 57.9336 ms  
passing volume into [conv3_3,Convolution]  
time elapsed: 4626.45 ms  
passing volume into [relu3_3,ReLU]  
time elapsed: 58.0146 ms  
passing volume into [pool3,Pooling]  
time elapsed: 73.138 ms  
passing volume into [conv4_1,Convolution]  
time elapsed: 2368.19 ms  
passing volume into [relu4_1,ReLU]  
time elapsed: 51.3569 ms  
passing volume into [conv4_2,Convolution]  
time elapsed: 4442.84 ms  
passing volume into [relu4_2,ReLU]  
time elapsed: 50.9285 ms  
passing volume into [conv4_3,Convolution]  
time elapsed: 4442.89 ms  
passing volume into [relu4_3,ReLU]  
time elapsed: 50.928 ms  
passing volume into [pool4,Pooling]  
time elapsed: 68.4493 ms  
passing volume into [conv5_1,Convolution]  
time elapsed: 1311.85 ms  
passing volume into [relu5_1,ReLU]  
time elapsed: 45.6615 ms  
passing volume into [conv5_2,Convolution]  
time elapsed: 1311.25 ms  
passing volume into [relu5_2,ReLU]  
time elapsed: 45.6487 ms  
passing volume into [conv5_3,Convolution]  
time elapsed: 1308.13 ms  
passing volume into [relu5_3,ReLU]  
time elapsed: 45.8167 ms  
passing volume into [pool5,Pooling]  
time elapsed: 64.1192 ms  
passing volume into [fc6,Convolution]  
time elapsed: 7872.58 ms  
passing volume into [relu6,ReLU]  
time elapsed: 46.6583 ms  
passing volume into [drop6,Dropout]  
time elapsed: 46.9071 ms  
passing volume into [fc7,Convolution]  
time elapsed: 1358.89 ms  
passing volume into [relu7,ReLU]  
time elapsed: 46.7101 ms  
passing volume into [drop7,Dropout]  
time elapsed: 47.2493 ms  
passing volume into [score59,Convolution]  
time elapsed: 243.469 ms  
__total time elapsed: 50744.7 ms__


-------------------------------------------------

### First convolved result
For RGB image  
![alt text] (https://github.com/xzhai1/latte/blob/master/images/rgb.png)  
the visualization of scaled feature maps from last conv layer is  
![alt text] (https://github.com/xzhai1/latte/blob/master/outputs/result.png)  
