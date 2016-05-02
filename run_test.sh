./test --image_path images/rgb.png \
       --train_val_path ../fcn.berkeleyvision.org/voc-fcn32s/val.prototxt \
       --trained_model_path ../fcn.berkeleyvision.org/voc-fcn32s/fcn32s-heavy-pascal.caffemodel \
       --test_deconv --stderrthreshold=0 
