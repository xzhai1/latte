 ./test --image_path images/cat.png \
        --train_val_path ../fcn.berkeleyvision.org/voc-fcn32s/deploy.prototxt \
        --trained_model_path ../fcn.berkeleyvision.org/voc-fcn32s/fcn32s-heavy-pascal.caffemodel \
        --iterations 3 \
        --batch_size 1 \
        --test_net --stderrthreshold=0 

#./test --image_path images/cat.png \
#        --train_val_path ../fcn.berkeleyvision.org/voc-fcn8s/deploy.prototxt \
#        --trained_model_path ../fcn.berkeleyvision.org/voc-fcn8s/fcn8s-heavy-pascal.caffemodel \
#        --test_net --stderrthreshold=0 


