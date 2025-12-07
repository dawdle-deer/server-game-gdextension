#pragma once

#include "godot_cpp/classes/box_shape3d.hpp"
#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/classes/wrapped.hpp"
#include "godot_cpp/variant/variant.hpp"

#include "heat_container.h"
#include "hvac_field_sample.h"

using namespace godot;

class HVACField : public Resource {
	GDCLASS(HVACField, RefCounted)

protected:
	static void _bind_methods();

public:
	Vector3i grid_size;
	TypedArray<HVACFieldSample> samples;
	TypedArray<HVACFieldSample> sample_grid;
	TypedArray<int> index_offsets_map;
	TypedArray<Vector3> bounds_basis_axis_map;
	TypedArray<float> sample_distance_map;
	float air_sample_mass;

	void set_grid_size(Vector3i p_grid_size);
	Vector3i get_grid_size() const;
	void set_samples(TypedArray<HVACFieldSample> p_samples);
	TypedArray<HVACFieldSample> get_samples() const;
	void set_sample_grid(TypedArray<HVACFieldSample> p_sample_grid);
	TypedArray<HVACFieldSample> get_sample_grid() const;
	void set_index_offsets_map(TypedArray<int> p_index_offsets_map);
	TypedArray<int> get_index_offsets_map() const;
	void set_bounds_basis_axis_map(TypedArray<Vector3> p_bounds_basis_axis_map);
	TypedArray<Vector3> get_bounds_basis_axis_map() const;
	void set_sample_distance_map(TypedArray<float> p_sample_distance_map);
	TypedArray<float> get_sample_distance_map() const;
	void set_air_sample_mass(float p_air_sample_mass);
	float get_air_sample_mass() const;

	int grid_pos_to_idx_v(Vector3i p_pos);
	int grid_pos_to_idx(int x, int y, int z);

	void generate_field(Transform3D p_bounds_transform, Vector3 p_bounds_size);
	void propagate_heat_elements(float p_delta);
	void propagate_air_samples(float p_delta);

	void blend_samples_to(TypedArray<int> p_sample_indices, float p_temperature, float p_blend_amount);
	void blend_samples_with(TypedArray<int> p_sample_indices, HeatContainer *p_heat_container, float p_blend_amount);

	HVACFieldSample get_sample_at(Vector3 p_position);
	TypedArray<int> get_grid_indices_in_box(Vector3 p_center, Vector3 p_size, Basis p_basis);
	float get_average_temp(TypedArray<int> p_sample_indices);

	Vector3i pos_to_grid(Vector3 p_position);
	Vector3 pos_to_grid_unrounded(Vector3 p_position);
	Vector3 grid_to_pos(Vector3i p_grid_pos);
	Vector3 unrounded_grid_to_pos(Vector3 p_grid_pos_unrounded);

	bool is_in_grid_bounds(Vector3i p_grid_position);

	HVACField() = default;
	~HVACField() override = default;
};