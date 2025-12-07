#pragma once

#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/classes/wrapped.hpp"
#include "godot_cpp/variant/variant.hpp"

using namespace godot;

class HVACSimParameters : public Resource {
	GDCLASS(HVACSimParameters, Resource)

protected:
	static void _bind_methods();

public:
	bool propagating = false;
	bool cooling = false;

	float cool_rate = 0.05f;
	float ambient_temperature = 20.0f;
	float efficiency = 1.0f;
	float air_propagation_speed = 1.0f;
	float element_propagation_speed = 2.0f;
	int air_propagation_limit = 5000;

	// setter/getters

	void set_propagating(bool p_propagating);
	bool get_propagating() const;

	void set_cooling(bool p_cooling);
	bool get_cooling() const;

	void set_cool_rate(float p_cool_rate);
	float get_cool_rate() const;

	void set_ambient_temperature(float p_ambient_temperature);
	float get_ambient_temperature() const;

	void set_efficiency(float p_efficiency);
	float get_efficiency() const;

	void set_air_propagation_speed(float p_air_propagation_speed);
	float get_air_propagation_speed() const;

	void set_element_propagation_speed(float p_element_propagation_speed);
	float get_element_propagation_speed() const;

	void set_air_propagation_limit(int p_air_propagation_limit);
	int get_air_propagation_limit() const;

	HVACSimParameters() = default;
	~HVACSimParameters() override = default;
};