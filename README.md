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

Now, if you are doing this on a mahine with no root access, we will want to do the following:

	./configure --prefix=$HOME/protobuf
	make
	make check
	make install

If you installed it with the ``--prefix`` flag, you want to set the following environmental variable:

	export PKG_CONFIG_PATH=$HOME/protobuf/lib/pkgconfig
	
This is so that ``pkg-config`` will know where to find ``protobuf``.

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

and fingers crossed, it won't throw an error. Then you can run a test. Since the libraries are loaded dynamically, you need to specify where to look for those libraries:

    LD_LIBRARY_PATH=../halide/bin/:~/protobuf/lib/ ./run_test images/rgb.png model/train_val.prototxt model/fcn-32s-pascalcontext.caffemodel    

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

### First convolved result
For RGB image  
![alt text] (https://github.com/xzhai1/latte/blob/master/images/rgb.png)  
the visualization of scaled feature maps from last conv layer is  
![alt text] (https://github.com/xzhai1/latte/blob/master/images/result.png)  
