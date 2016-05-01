
# coding: utf-8

# In[90]:

# The caffe module needs to be on the Python path;
#  we'll add it here explicitly.
import sys
caffe_root = '/home/xd/Projects/CMU/caffe/'  # this file should be run from {caffe_root}/examples (otherwise change this line)
model_root = '/home/xd/Projects/CMU/fcn.berkeleyvision.org/'
sys.path.insert(0, caffe_root + 'python')

import caffe
# If you get "No module named _caffe", either you have not built pycaffe or you have the wrong path.
import numpy as np
get_ipython().magic(u'matplotlib inline')
import matplotlib.pyplot as plt
from PIL import Image


# In[68]:

caffe.set_mode_cpu()

model_def = model_root + 'voc-fcn8s/deploy.prototxt'
model_weights = model_root + 'voc-fcn8s/fcn8s-heavy-pascal.caffemodel'

net = caffe.Net(model_def,      # defines the structure of the model
                model_weights,  # contains the trained weights
                caffe.TEST)     # use test mode (e.g., don't perform dropout)


# In[ ]:




# In[125]:

#im = Image.open('/home/xd/Projects/CMU/TrainVal/VOCdevkit/VOC2011/JPEGImages/2007_000129.jpg')
im = Image.open('../images/cat.png')
in_ = np.array(im, dtype=np.float32)
print in_.shape
in_ = in_[:,:,::-1]
print in_.shape
#in_ -= np.array((104.00698793,116.66876762,122.67891434))                          
in_ = in_.transpose((2,0,1))
print in_.shape


# In[126]:

print in_.shape
input_data = net.blobs['data']

for i in range(4):
    print input_data.shape[i]


# In[194]:

weights = net.params['conv1_1'][0]
#bias = net.params['conv1_1'][1]


# In[195]:

print weights.shape[0], weights.num
print weights.shape[1], weights.channels
print weights.shape[2], weights.height
print weights.shape[3], weights.width
weights.data.shape


# In[168]:

# shape for input (data blob is N x C x H x W), set data                           
net.blobs['data'].reshape(1, *in_.shape)                                           
net.blobs['data'].data[...] = in_

print list(net._layer_names)
# just run till the conv1_1 layer, inclusive
net.forward(start='input', end='conv1_1')


# In[169]:

# result of the convolution, initialized to 0
conv1_1_result = net.blobs['conv1_1']
print conv1_1_result.data.shape


# In[229]:

type(conv1_1_result.data[0])


# In[231]:

# can save all layers here
np.savetxt("caffe.csv", conv1_1_result.data[0, 0], delimiter=",")


# In[213]:

#filt_min, filt_max = conv1_1_result.data.min(), conv1_1_result.data.max()
plt.figure()
#plt.imshow(conv1_1_result.data[0, 0], cmap='gray', vmin=filt_min, vmax=filt_max)
plt.imshow(conv1_1_result.data[0, 0], cmap='gray')
plt.axis('off')
plt.savefig("test.png")


# In[217]:

caffe_data = conv1_1_result.data[0, 0]


# In[218]:

halide_data = np.genfromtxt("../test.csv", delimiter=",")


# In[219]:

caffe_data.shape


# In[220]:

halide_data.shape


# In[223]:

for x in range(734):
    for y in range(734):
        caffe_val = caffe_data[x, y]
        halide_val = caffe_data[x, y]
        if caffe_val != halide_val:
            print x, y


# In[ ]:



