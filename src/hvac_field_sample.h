#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

class HVACFieldSample : public Resource {
	GDCLASS(HVACFieldSample, Resource)

protected:
	static void _bind_methods();

public:
	Vector3 position;
	int neighbors_valid;
	float temperature;
	int grid_index;

	void blend_to_temperature(float p_temperature, float p_delta);

	// setter/getters

	void set_position(Vector3 p_position);
	Vector3 get_position() const;

	void set_neighbors_valid(int p_neighbors_valid);
	int get_neighbors_valid() const;

	void set_temperature(float p_temperature);
	float get_temperature() const;

	void set_grid_index(int p_grid_index);
	int get_grid_index() const;

	HVACFieldSample() = default;
	~HVACFieldSample() override = default;
};