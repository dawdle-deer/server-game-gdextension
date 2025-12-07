#include "hvac_field_sample.h"

#include <godot_cpp/core/math.hpp>

/// @brief One-way blend to single value
/// @param p_temperature
/// @param p_delta
void HVACFieldSample::blend_to_temperature(float p_temperature, float p_delta) {
	temperature = Math::lerp(
			temperature,
			p_temperature,
			Math::clamp(p_delta, 0.0f, 1.0f));
}

void HVACFieldSample::set_position(Vector3 p_position) {
	position = p_position;
}
Vector3 HVACFieldSample::get_position() const {
	return position;
}

void HVACFieldSample::set_neighbors_valid(int p_neighbors_valid) {
	neighbors_valid = p_neighbors_valid;
}
int HVACFieldSample::get_neighbors_valid() const {
	return neighbors_valid;
}

void HVACFieldSample::set_temperature(float p_temperature) {
	temperature = p_temperature;
}
float HVACFieldSample::get_temperature() const {
	return temperature;
}

void HVACFieldSample::set_grid_index(int p_grid_index) {
	grid_index = p_grid_index;
}
int HVACFieldSample::get_grid_index() const {
	return grid_index;
}

void HVACFieldSample::_bind_methods() {
	godot::ClassDB::bind_method(D_METHOD("set_position", "position"), &HVACFieldSample::set_position);
	godot::ClassDB::bind_method(D_METHOD("get_position"), &HVACFieldSample::get_position);
	godot::ClassDB::bind_method(D_METHOD("set_neighbors_valid", "neighbors_valid"), &HVACFieldSample::set_neighbors_valid);
	godot::ClassDB::bind_method(D_METHOD("get_neighbors_valid"), &HVACFieldSample::get_neighbors_valid);
	godot::ClassDB::bind_method(D_METHOD("set_temperature", "temperature"), &HVACFieldSample::set_temperature);
	godot::ClassDB::bind_method(D_METHOD("get_temperature"), &HVACFieldSample::get_temperature);
	godot::ClassDB::bind_method(D_METHOD("set_grid_index", "grid_index"), &HVACFieldSample::set_grid_index);
	godot::ClassDB::bind_method(D_METHOD("get_grid_index"), &HVACFieldSample::get_grid_index);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position"), "set_position", "get_position");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "neighbors_valid"), "set_neighbors_valid", "get_neighbors_valid");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature"), "set_temperature", "get_temperature");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "grid_index"), "set_grid_index", "get_grid_index");
}