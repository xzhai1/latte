#ifndef LOSS_LAYERS_H
#define LOSS_LAYERS_H

#include "Halide.h"
#include "caffe.pb.h"

#include "layers.h"

/**
 * Loss drives learning by comparing an output to a target and assigning cost 
 * to minimize. The loss itself is computed by the forward pass and the 
 * gradient w.r.t. to the loss is computed by the backward pass.
 */
namespace Latte {

class SoftmaxWithLoss;
class Softmax;

/**
 * @brief Softmax layer
 *
 * Softmax regression (or multinomial logistic regression) is a generalization
 * of logistic regression to the case where we want to handle multiple classes. 
 */
class Softmax : public Layer {
  public:
    /**
     * @brief Softmax 
     *
     * @param name
     */
    Softmax(std::string name);

    /**
     * @brief softmax 
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> run(Halide::Image<float> input);
};

/**
 * @brief SoftmaxWithLoss layer
 *
 * The softmax loss layer computes the multinomial logistic loss of the
 * softmax of its inputs. It’s conceptually identical to a softmax layer
 * followed by a multinomial logistic loss layer, but provides a more
 * numerically stable gradient.
 */
class SoftmaxWithLoss : public Layer {
  public:
    /**
     * @brief SoftmaxWithLoss 
     *
     * @param name 
     */
    SoftmaxWithLoss(std::string layer_name);
    
    /**
     * @brief softmaxwithloss 
     *
     * @param input
     *
     * @return 
     */
    Halide::Image<float> run(Halide::Image<float> input);
};

} /* namespace latte */

#endif /* LOSS_LAYERS_H */
