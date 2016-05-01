./test --image_path images/cat.png \
       --train_val_path ../fcn.berkeleyvision.org/voc-fcn8s/deploy.prototxt \
       --trained_model_path ../fcn.berkeleyvision.org/voc-fcn8s/fcn8s-heavy-pascal.caffemodel \
       --test_conv --stderrthreshold=0 
