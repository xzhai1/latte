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

    LD_LIBRARY_PATH=../halide/bin/ ./test images/rgb.png model/fcn-32s-pascalcontext.caffemodel    
