#include "layers.h"

namespace Latte {

/* constructor */
Layer::Layer(): name(""), type(""), next(NULL) {

}

/* destructor */
Layer::~Layer() {

}

/* mutators */
void Layer::set_name(std::string layer_name) {
	name = layer_name;
}

void Layer::set_type(std::string layer_type) {
	type = layer_type;
}

void Layer::set_next(Layer *next_layer) {
	next = next_layer;
}

/* accessors */
std::string Layer::get_name() {
	return name;
}

std::string Layer::get_type() {
	return type;
}

Layer *Layer::get_next() {
	return next;
}

/* virtual function */
Halide::Image<float> Layer::run(Halide::Image<float> input) {
	Halide::Image<float> output(1);
	return output;
}

} /* namespace latte */