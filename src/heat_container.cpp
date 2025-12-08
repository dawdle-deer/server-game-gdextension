#include "heat_container.h"

#include <godot_cpp/core/math.hpp>

void HeatContainer::set_temperature(float p_temperature) {
	temperature = p_temperature;
}
float HeatContainer::get_temperature() const {
	return temperature;
}

void HeatContainer::set_mass(float p_mass) {
	mass = Math::max(p_mass, 0.00001f);
}
float HeatContainer::get_mass() const {
	return mass;
}

void HeatContainer::set_heat_retention(float p_heat_retention) {
	heat_retention = p_heat_retention;
}
float HeatContainer::get_heat_retention() const {
	return heat_retention;
}

/// @brief One-way blend. Does not modify target
/// @param p_target
/// @param p_delta
void HeatContainer::blend_to(const HeatContainer *p_target, float p_delta) {
	ERR_FAIL_NULL_MSG(p_target, "Can't blend to heat, target is null!");
	temperature = Math::lerp(
			temperature,
			p_target->temperature,
			Math::clamp(p_delta * p_target->mass / mass * (1.0f - heat_retention), 0.0f, 1.0f));
}

/// @brief Two-way blend. Modifies target
/// @param p_target
/// @param p_delta
void HeatContainer::blend_with(HeatContainer *p_target, float p_delta) {
	ERR_FAIL_NULL_MSG(p_target, "Can't blend heat with null target!");
	float temp_cache = temperature;
	temperature = Math::lerp(
			temperature,
			p_target->temperature,
			Math::clamp(p_delta * p_target->mass / mass * (1.0f - heat_retention), 0.0f, 1.0f));
	p_target->temperature = Math::lerp(
			p_target->temperature,
			temp_cache,
			Math::clamp(p_delta * mass / p_target->mass * (1.0f - p_target->heat_retention), 0.0f, 1.0f));
}

/// @brief One-way blend to single value
/// @param p_temperature
/// @param p_delta
/// @param ignore_mass
void HeatContainer::blend_to_temperature(float p_temperature, float p_delta) {
	temperature = Math::lerp(
			temperature,
			p_temperature,
			Math::clamp(p_delta / mass * (1.0f - heat_retention), 0.0f, 1.0f));
}

void HeatContainer::_bind_methods() {
	godot::ClassDB::bind_method(D_METHOD("set_temperature", "temperature"), &HeatContainer::set_temperature);
	godot::ClassDB::bind_method(D_METHOD("get_temperature"), &HeatContainer::get_temperature);
	godot::ClassDB::bind_method(D_METHOD("set_mass", "mass"), &HeatContainer::set_mass);
	godot::ClassDB::bind_method(D_METHOD("get_mass"), &HeatContainer::get_mass);
	godot::ClassDB::bind_method(D_METHOD("set_heat_retention", "heat_retention"), &HeatContainer::set_heat_retention);
	godot::ClassDB::bind_method(D_METHOD("get_heat_retention"), &HeatContainer::get_heat_retention);

	godot::ClassDB::bind_method(D_METHOD("blend_to", "target", "delta"), &HeatContainer::blend_to);
	godot::ClassDB::bind_method(D_METHOD("blend_with", "target", "delta"), &HeatContainer::blend_with);
	godot::ClassDB::bind_method(D_METHOD("blend_to_temperature", "temperature", "delta"), &HeatContainer::blend_to_temperature);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature"), "set_temperature", "get_temperature");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "heat_retention"), "set_heat_retention", "get_heat_retention");
}