#pragma once

#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/classes/wrapped.hpp"
#include "godot_cpp/variant/variant.hpp"

using namespace godot;

class HeatContainer : public Resource {
	GDCLASS(HeatContainer, RefCounted)

protected:
	static void _bind_methods();

public:
	float temperature = 0.0f;
	float mass = 1.0f;
	float heat_retention = 0.0f;

	void blend_to(const HeatContainer *target, float delta);
	void blend_with(HeatContainer *target, float delta);
	void blend_to_temperature(float temperature, float delta, bool ignore_mass);

	// setter/getters

	void set_temperature(float p_temperature);
	float get_temperature() const;

	void set_mass(float p_mass);
	float get_mass() const;

	void set_heat_retention(float p_heat_retention);
	float get_heat_retention() const;

	HeatContainer() = default;
	~HeatContainer() override = default;
};