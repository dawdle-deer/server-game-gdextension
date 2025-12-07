#include "hvac_sim_parameters.h"

#include "godot_cpp/core/math.hpp"

void HVACSimParameters::set_propagating(bool p_propagating) {
	propagating = p_propagating;
}

bool HVACSimParameters::get_propagating() const {
	return propagating;
}

void HVACSimParameters::set_cooling(bool p_cooling) {
	cooling = p_cooling;
}

bool HVACSimParameters::get_cooling() const {
	return cooling;
}

void HVACSimParameters::set_cool_rate(float p_cool_rate) {
	cool_rate = p_cool_rate;
}
float HVACSimParameters::get_cool_rate() const {
	return cool_rate;
}

void HVACSimParameters::set_ambient_temperature(float p_ambient_temperature) {
	ambient_temperature = p_ambient_temperature;
}

float HVACSimParameters::get_ambient_temperature() const {
	return ambient_temperature;
}

void HVACSimParameters::set_efficiency(float p_efficiency) {
	efficiency = p_efficiency;
}
float HVACSimParameters::get_efficiency() const {
	return efficiency;
}

void HVACSimParameters::set_air_propagation_speed(float p_air_propagation_speed) {
	air_propagation_speed = p_air_propagation_speed;
}
float HVACSimParameters::get_air_propagation_speed() const {
	return air_propagation_speed;
}

void HVACSimParameters::set_element_propagation_speed(float p_element_propagation_speed) {
	element_propagation_speed = p_element_propagation_speed;
}

float HVACSimParameters::get_element_propagation_speed() const {
	return element_propagation_speed;
}

void HVACSimParameters::set_air_propagation_limit(int p_air_propagation_limit) {
	air_propagation_limit = p_air_propagation_limit;
}
int HVACSimParameters::get_air_propagation_limit() const {
	return air_propagation_limit;
}

void HVACSimParameters::_bind_methods() {
	godot::ClassDB::bind_method(D_METHOD("set_propagating", "propagating"), &HVACSimParameters::set_propagating);
	godot::ClassDB::bind_method(D_METHOD("get_propagating"), &HVACSimParameters::get_propagating);
	godot::ClassDB::bind_method(D_METHOD("set_cooling", "cooling"), &HVACSimParameters::set_cooling);
	godot::ClassDB::bind_method(D_METHOD("get_cooling"), &HVACSimParameters::get_cooling);

	godot::ClassDB::bind_method(D_METHOD("set_cool_rate", "cool_rate"), &HVACSimParameters::set_cool_rate);
	godot::ClassDB::bind_method(D_METHOD("get_cool_rate"), &HVACSimParameters::get_cool_rate);
	godot::ClassDB::bind_method(D_METHOD("set_efficiency", "efficiency"), &HVACSimParameters::set_efficiency);
	godot::ClassDB::bind_method(D_METHOD("get_efficiency"), &HVACSimParameters::get_efficiency);
	godot::ClassDB::bind_method(D_METHOD("set_air_propagation_speed", "air_propagation_speed"), &HVACSimParameters::set_air_propagation_speed);
	godot::ClassDB::bind_method(D_METHOD("get_air_propagation_speed"), &HVACSimParameters::get_air_propagation_speed);
	godot::ClassDB::bind_method(D_METHOD("set_element_propagation_speed", "element_propagation_speed"), &HVACSimParameters::set_element_propagation_speed);
	godot::ClassDB::bind_method(D_METHOD("get_element_propagation_speed"), &HVACSimParameters::get_element_propagation_speed);
	godot::ClassDB::bind_method(D_METHOD("set_ambient_temperature", "ambient_temperature"), &HVACSimParameters::set_ambient_temperature);
	godot::ClassDB::bind_method(D_METHOD("get_ambient_temperature"), &HVACSimParameters::get_ambient_temperature);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "propagating"), "set_propagating", "get_propagating");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cooling"), "set_cooling", "get_cooling");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cool_rate"), "set_cool_rate", "get_cool_rate");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "ambient_temperature"), "set_ambient_temperature", "get_ambient_temperature");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "efficiency"), "set_efficiency", "get_efficiency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "air_propagation_speed"), "set_air_propagation_speed", "get_air_propagation_speed");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "element_propagation_speed"), "set_element_propagation_speed", "get_element_propagation_speed");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "air_propagation_limit"), "set_air_propagation_limit", "get_air_propagation_limit");
}