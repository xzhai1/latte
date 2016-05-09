# latte
CMU 15-418/618 Final Project: Implementing Fully Convolutional Network using Halide and evaluate against Caffe version

## Dependencies on Local Machine
The following steps are necessary for a brand new machine(tested on Ubuntu); some might be unnecessary for you. If you are building on CMU SCS's Lateday node, go straight to the Build section.

### Utilities
Make sure you have the build tool chain:

    sudo apt-get install build-essential

We are going to use ``libpng``:

    sudo apt-get install libpng12-dev

You are going to need ``autoreconf`` for protobuf later:

    sudo apt-get install dh-autoreconf
    
### ``protobuf``
Download it. It is a Google serialization library. Think type JSON. You need it to read in the caffe model and trained weights:

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

Now, if you are doing this on a mahine with no root access, you will want to do the following:

	./configure --prefix=$HOME/protobuf
	make
	make check
	make install

If you installed it with the ``--prefix`` flag, you want to set the following environmental variable in your ``bashrc``:

	export PKG_CONFIG_PATH=$HOME/protobuf/lib/pkgconfig
	
This is so that ``pkg-config`` will know where to find ``protobuf``.

### `gflags` and `glog`
It is best that we don't reinvent the wheels and that is why we use the stuff smarter people built. [`gflags`](https://gflags.github.io/gflags/) is Google's command line module. [`glog`](https://github.com/google/glog) is Google's logging module. 

	sudo apt-get instsall libgflags-dev libgoogle-glog-dev

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
    
If you are doing this on your local machine, your directory structure should look like this:

    xd@xd-Standard-PC-i440FX-PIIX-1996:~/Documents$ ls
    halide  latte  protobuf-2.6.1

Then go into the repo:
    
    cd latte
   
If you are building on local:

    make -f Makefile-local -j8

If you are buidling on ``latedays``. A word of warning for ``latedays`` node: the ``Makefile-latedays`` is a lot less flexible than ``Makefile-local`` in the sense that we have to hardcode in the path to the library that we are linking to, i.e. ``/home/15-418/``. All of those library are built from source without root access and they aren't "installed", i.e. they aren't under system path and we can't use ``pkg-config`` to find it and link them. One of these days, one of those folders might get cleaned up and deleted, and as a result, you might not be able to successfully make it.

    make -f Makefile-latedays -j8
    
Then download the fcn repo:

    cd ..
    git clone https://github.com/shelhamer/fcn.berkeleyvision.org.git
    cd fcn.berkeleyvision.org
    
Now you can use any or the following model to test:

    voc-fcn16s  voc-fcn32s  voc-fcn8s
    
Because the trained caffe model is huge, they aren't in the folder. You need to download them; their url is in ``caffemodel-url`` and you can just ``wget`` them.

Set one last environmental vairable in your ``bashrc``:

	export LD_LIBRARY_PATH=/opt/gcc/4.9.2/lib64:/path/to/halide/bin/:/path/to/protobuf/lib/:${LD_LIBRARY_PATH}

and you can run a test by invoking:

	./run_test.sh

If you want to see what command line options are available to you:

	./test -helpshort
	test: 
	 ./test --image_path            image.png
	        --train_val_path        train_val.prototxt
	        --trained_model_path    trained_model.caffemodel
	        --test_loadfromtext
	        --test_net

## Install Caffe for Benchmarking
### On CMU SCS's Latedays node
Because we don't have privileged access on Latedays and we have only 2GB of disk quota, we can't install anything there. However, there is a class that uses Caffe on Latedays and they have it set up. Follow the instruction [here](https://docs.google.com/document/d/12HEbJd989Uo1zjknc9UygYGsfNn7zVq8qLxJYLjy8g8/edit). The gist is this:

    wget http://ladoga.graphics.cs.cmu.edu/xiaolonw/assignment.tar.gz
    untar -xvf assignment.tar.gz
    rm assignment.tar.gz
    
Download caffe source and move two files from the assignment folder into caffe:

    git clone https://github.com/BVLC/caffe.git
    cd caffe
    mv ../caffe/bashrc_class .
    mv ../caffe/Makefile.config .
    
A word of warning: ``bashrc_class`` changes the paths to a lot of the libraries in the TA's home directory. He had an MKL license which is currently expired (at time of writing, 5/9/2016). You need to figure out a way to get that license or build it with atlas which won't spawn multiple threads to do matrix multiplication and therefore, you won't be able to reproduce our results. Nothing left but to make:

    source bashrc_class
    make
    make pycaffe
    
Then we can run the benchmark:

    cd ../latte/python
    usage: caffebenchmark.py [-h] -c CAFFE_PYTHON -d DEPLOY_PROTOTXT -t
                         TRAINED_CAFFEMODEL -b BATCH -i IMAGE
    optional arguments:
      -h, --help            show this help message and exit
      -c CAFFE_PYTHON, --caffe_python CAFFE_PYTHON
                            path/to/caffe/python
      -d DEPLOY_PROTOTXT, --deploy_prototxt DEPLOY_PROTOTXT
                            path to deploy.prototxt
      -t TRAINED_CAFFEMODEL, --trained_caffemodel TRAINED_CAFFEMODEL
                            path to model.caffemodel
      -b BATCH, --batch BATCH
                            batch size
      -i IMAGE, --image IMAGE
                            path to test image

For example:

    python caffebenchmark.py -c ../../caffe/python/ -d ../../fcn.berkeleyvision.org/voc-fcn32s/deploy.prototxt -t ../../fcn.berkeleyvision.org/voc-fcn32s/fcn32s-heavy-pascal.caffemodel -b 3 -i ../images/cat.png


### On Your Own Damn Machine
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

And then you can run the same test upstairs.

### Deconvolution and segmentation result
For cute ``cat.png``   
![alt text] (https://raw.githubusercontent.com/xzhai1/latte/master/images/cat.png)  
the visualization of scaled feature maps from deconv layer is  
![alt text] (https://github.com/xzhai1/latte/blob/master/images/cat_deconv_results.png)  
segmentation result is  
![alt text] (https://github.com/xzhai1/latte/blob/master/images/cat_seg_result.png)  

|   |Halide   |Caffe   |
|:---:|:---:|:---:|
|Best inference time (s)   | 2.849 | 4.201 |
|Peak memory usage (GB) | 1.472 | 4.8 |
