# ./test --image_path images/bird.png \
#        --train_val_path ../fcn.berkeleyvision.org/voc-fcn32s/val.prototxt \
#        --trained_model_path ../fcn.berkeleyvision.org/voc-fcn32s/fcn32s-heavy-pascal.caffemodel \
#        --test_net --stderrthreshold=0 


./test --image_path images/bird.png \
       --train_val_path ../caffe/models/bvlc_reference_caffenet/deploy.prototxt \
       --trained_model_path ../caffe/models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel \
       --test_net --stderrthreshold=0 
