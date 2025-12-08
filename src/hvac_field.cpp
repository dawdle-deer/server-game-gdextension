#include "hvac_field.h"

#include <godot_cpp/classes/physics_point_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/random_number_generator.hpp>

#include <hvac_field_sample.h>

class DebugDraw3D;

int HVACField::grid_pos_to_idx_v(Vector3i p_pos) {
	return (p_pos.y * grid_size.x * grid_size.z) + (p_pos.z * grid_size.x) + p_pos.x;
}
int HVACField::grid_pos_to_idx(int x, int y, int z) {
	return (y * grid_size.x * grid_size.z) + (z * grid_size.x) + x;
}

int HVACField::pos_to_idx(Vector3 p_pos) {
	return grid_pos_to_idx_v(pos_to_grid(p_pos));
}

void HVACField::generate_field(Transform3D p_bounds_transform, Vector3 p_bounds_size, int p_statics_collision_mask, PhysicsDirectSpaceState3D *p_physics_space_state, float p_min_temp_variance, float p_max_temp_variance) {
	ERR_FAIL_NULL(p_physics_space_state);
	ERR_FAIL_NULL(sim_parameters);

	print_line_rich("[u]Generating HVAC Field[/u]");

	Ref<PhysicsPointQueryParameters3D> query = memnew(PhysicsPointQueryParameters3D);
	query->set_collision_mask(p_statics_collision_mask);
	Ref<PhysicsRayQueryParameters3D> n_query = PhysicsRayQueryParameters3D::create(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), p_statics_collision_mask);
	Ref<PhysicsRayQueryParameters3D> up_query = PhysicsRayQueryParameters3D::create(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), p_statics_collision_mask);
	Ref<HVACFieldSample> cur_sample;

	grid_size = (p_bounds_size / sample_spacing).ceil();
	grid_half_size = Vector3(grid_size) * 0.5f;
	print_line_rich("\tfield size: ", grid_size);

	bounds_transform = p_bounds_transform;
	inv_bounds_transform = p_bounds_transform.affine_inverse();
	Basis bounds_basis = p_bounds_transform.get_basis();

	sample_distance_map = TypedArray<float>();
	sample_distance_map.push_back(sample_spacing.y);
	sample_distance_map.push_back(sample_spacing.y);
	sample_distance_map.push_back(sample_spacing.z);
	sample_distance_map.push_back(sample_spacing.z);
	sample_distance_map.push_back(sample_spacing.x);
	sample_distance_map.push_back(sample_spacing.x);

	bounds_basis_axis_map = TypedArray<Vector3>();
	bounds_basis_axis_map.push_back(bounds_basis[0]);
	bounds_basis_axis_map.push_back(-bounds_basis[0]);
	bounds_basis_axis_map.push_back(bounds_basis[1]);
	bounds_basis_axis_map.push_back(-bounds_basis[1]);
	bounds_basis_axis_map.push_back(bounds_basis[2]);
	bounds_basis_axis_map.push_back(-bounds_basis[2]);

	index_offsets_map = TypedArray<int>();
	index_offsets_map.push_back(grid_size.x * grid_size.z);
	index_offsets_map.push_back(-grid_size.x * grid_size.z);
	index_offsets_map.push_back(grid_size.x);
	index_offsets_map.push_back(-grid_size.x);
	index_offsets_map.push_back(1);
	index_offsets_map.push_back(-1);

	samples.clear();
	sample_grid.clear();
	sample_grid.resize(grid_size.x * grid_size.y * grid_size.z);
	for (size_t y = 0; y < grid_size.y; y++) {
		for (size_t z = 0; z < grid_size.z; z++) {
			for (size_t x = 0; x < grid_size.x; x++) {
				sample_grid[grid_pos_to_idx(x, y, z)] = nullptr;
				// Make sure sample isn't inside a surface
				Vector3 sample_pos_world = grid_to_pos(Vector3i(x, y, z));
				query->set_position(sample_pos_world);
				if (!p_physics_space_state->intersect_point(query).is_empty()) {
					//print("sample " + str(query.position) + " inside surface " + result[0]["collider"].get_parent().name + ", ignoring")
					continue;
				}
				// Make sure sample is indoors by checking for a roof above, a floor below, and a wall in each cardinal direction
				up_query->set_from(sample_pos_world);
				up_query->set_to(sample_pos_world + Vector3(0.0f, 15.0f, 0.0f));
				if (p_physics_space_state->intersect_ray(up_query).is_empty()) {
					// print("sample " + str(query.position) + " outdoors, ignoring")
					continue;
				}
				up_query->set_to(sample_pos_world + Vector3(0.0f, -15.0f, 0.0f));
				if (p_physics_space_state->intersect_ray(up_query).is_empty()) {
					// print("sample " + str(query.position) + " outdoors, ignoring")
					continue;
				}
				bool wall_missed = false;
				for (size_t i = 0; i < 6; i++) {
					up_query->set_to(sample_pos_world + Vector3(bounds_basis_axis_map[i]) * 60);
					if (p_physics_space_state->intersect_ray(up_query).is_empty()) {
						wall_missed = true;
						break;
					}
					up_query->set_to(up_query->get_to() + Vector3(0.0f, 15.0f, 0.0f));
					if (p_physics_space_state->intersect_ray(up_query).is_empty()) {
						wall_missed = true;
						break;
					}
				}
				if (wall_missed) {
					continue;
				}
				Ref<RandomNumberGenerator> rand_gen = memnew(RandomNumberGenerator);
				// Generate / save sample data
				cur_sample = Ref<HVACFieldSample>(memnew(HVACFieldSample));
				cur_sample->grid_index = grid_pos_to_idx(x, y, z);
				cur_sample->position = sample_pos_world;
				cur_sample->temperature = rand_gen->randf_range(p_min_temp_variance, p_max_temp_variance) + sim_parameters->ambient_temperature;
				samples.push_back(cur_sample);
				sample_grid[cur_sample->grid_index] = cur_sample;
				// Cast rays to see if neighboring samples are obscured
				// TODO : perform this step later to avoid up query and halve neighbor checks ?
				n_query->set_from(sample_pos_world);
				for (size_t i = 0; i < 6; i++) {
					Vector3 neighbor_pos = sample_pos_world + Vector3(bounds_basis_axis_map[i]) * float(sample_distance_map[i]);
					n_query->set_to(neighbor_pos);
					up_query->set_from(neighbor_pos);
					up_query->set_to(neighbor_pos + Vector3(0.0f, 15.0f, 0.0f));
					if (!p_physics_space_state->intersect_ray(n_query).is_empty() || p_physics_space_state->intersect_ray(up_query).is_empty()) {
						continue;
					}
					cur_sample->neighbors_valid |= 1 << i;
				}
			}
		}
	}
	print_line_rich("[u]Finished Generating HVAC Field[/u]");
}

void HVACField::propagate_air_samples(float p_delta) {
	ERR_FAIL_NULL(sim_parameters);
	if (samples.is_empty()) {
		return;
	}

	float air_propagation = p_delta * sim_parameters->air_propagation_speed * sim_parameters->efficiency;
	float cool_speed = p_delta * sim_parameters->cool_rate * sim_parameters->efficiency * 0.001f;
	int sample_count = samples.size();
	int sample_update_count = sample_count;
	// int sample_update_count = Math::min(sim_parameters->air_propagation_limit, sample_count);

	for (size_t s = 0; s < sample_update_count; s++) {
		// int idx = (air_sample_iterator + s) % sample_count;
		int idx = s;
		ERR_FAIL_INDEX(idx, sample_count);
		Ref<HVACFieldSample> sample = samples[idx];
		if (sample.is_null()) {
			continue;
		}

		float neighbor_avg = 0;
		int n_cnt = 0;
		for (size_t i = 0; i < 6; i++) {
			if ((sample->neighbors_valid & (1 << i)) > 0) {
				int index = sample->grid_index + int(index_offsets_map[i]);
				if (index >= max_sample_idx || index < 0) {
					continue;
				}
				Ref<HVACFieldSample> n_sample = sample_grid[index];
				if (n_sample.is_null()) {
					continue;
				}
				neighbor_avg += n_sample->temperature;
				n_sample->blend_to_temperature(sample->temperature, air_propagation);
				n_cnt++;
			}
		}

		if (n_cnt > 0) {
			neighbor_avg /= n_cnt;
			sample->blend_to_temperature(neighbor_avg, air_propagation);
		}
		if (sim_parameters->cooling) {
			sample->temperature = Math::lerp(sample->temperature, sim_parameters->ambient_temperature, cool_speed);
		}
	}

	// air_sample_iterator += sample_update_count;
}

void HVACField::propagate_heat_container_to_box(float p_delta, HeatContainer *p_heat_container, Vector3 p_box_center, Vector3 p_box_size, Basis p_box_basis, bool distribute_evenly) {
	ERR_FAIL_NULL(p_heat_container);
	if (samples.is_empty()) {
		return;
	}

	TypedArray<int> sample_indices = get_grid_indices_in_box(p_box_center, p_box_size, p_box_basis);
	if (sample_indices.is_empty()) {
		return;
	}

	float blend_amount = Math::clamp(p_delta * sim_parameters->element_propagation_speed * sim_parameters->efficiency, 0.0f, 1.0f);
	blend_samples_with(sample_indices, p_heat_container, blend_amount, !distribute_evenly);
}

void HVACField::blend_samples_with(TypedArray<int> p_sample_indices, HeatContainer *p_heat_container, float p_blend_amount, bool ignore_sample_count) {
	ERR_FAIL_NULL(p_heat_container);
	ERR_FAIL_NULL(sim_parameters);
	if (samples.is_empty() || p_sample_indices.is_empty()) {
		return;
	}
	float average_sample_temperature = get_average_temp(p_sample_indices);
	float count_factor = ignore_sample_count ? 1.0 : p_sample_indices.size();
	float mass_ratio = p_heat_container->mass / sim_parameters->air_sample_mass;
	blend_samples_to(p_sample_indices, p_heat_container->temperature, p_blend_amount * mass_ratio / count_factor);
	p_heat_container->blend_to_temperature(average_sample_temperature, p_blend_amount * count_factor * p_heat_container->mass / mass_ratio);
}

void HVACField::blend_samples_to(TypedArray<int> p_sample_indices, float p_temperature, float p_blend_amount) {
	if (samples.is_empty() || p_sample_indices.is_empty()) {
		return;
	}
	for (size_t i = 0; i < p_sample_indices.size(); i++) {
		if (int(p_sample_indices[i]) >= max_sample_idx) {
			continue;
		}
		Ref<HVACFieldSample> sample = sample_grid[p_sample_indices[i]];
		if (sample == nullptr) {
			continue;
		}
		sample->blend_to_temperature(p_temperature, p_blend_amount);
	}
}

Ref<HVACFieldSample> HVACField::get_sample_at(Vector3 p_position) {
	Vector3i grid_pos = pos_to_grid(p_position);
	if (!is_in_grid_bounds(grid_pos)) {
		return nullptr;
	}
	int index = grid_pos_to_idx_v(grid_pos);
	ERR_FAIL_INDEX_V(index, max_sample_idx, nullptr);
	return sample_grid[index];
}

AABB HVACField::get_grid_bounding_aabb(Vector3 p_center, AABB p_bounds, Basis p_basis) {
	AABB grid_aligned_bounds = AABB(pos_to_grid_unrounded(p_center), Vector3(0.0f, 0.0f, 0.0f));
	for (size_t i = 0; i < 8; i++) {
		Vector3 corner_pos_world = p_bounds.position + p_basis.xform(p_bounds.get_endpoint(i) - p_bounds.position);
		Vector3 corner_pos_grid = pos_to_grid_unrounded(corner_pos_world).floor();
		// if (draw_debug_shapes) {
		// 	debug_drawer->draw_sphere(corner_pos, 0.03, Color(1, 0.921, 0.803));
		// 	debug_drawer->draw_sphere(grid_to_pos(corner_pos_grid), 0.03, Color(0.541, 0.168, 0.886));box_origin
		// }
		grid_aligned_bounds.expand_to(corner_pos_grid);
		grid_aligned_bounds.expand_to(corner_pos_grid + Vector3(1, 1, 1));
	}
	return grid_aligned_bounds;
}

TypedArray<int> HVACField::get_grid_indices_in_box(Vector3 p_center, Vector3 p_size, Basis p_basis) {
	Vector3 box_origin = p_center - p_basis.xform(p_size * 0.5);
	AABB box_bounds = AABB(box_origin, p_size);
	// if (draw_debug_shapes) {
	// 	debug_drawer->scoped_config()->set_thickness(0.004)->set_center_brightness(0.8);
	// 	if (draw_in_bounds_space) {
	// 		debug_drawer->draw_box(box_bounds.position, Quaternion(), box_bounds.size, Color(0, 1, 0));
	// 	} else {
	// 		debug_drawer->draw_box(box_bounds.position, box_to_world_basis.get_rotation_quaternion(), box_bounds.size, Color(0.491, 1, 0.83));
	// 	}
	// 	debug_drawer->draw_sphere(p_center, 0.03, Color(0.862, 0.078, 0.235));
	// }
	AABB grid_aligned_bounds = get_grid_bounding_aabb(p_center, box_bounds, p_basis);

	Vector3i size_grid = Vector3i(grid_aligned_bounds.size.ceil());
	Vector3i origin_grid = Vector3i(grid_aligned_bounds.position.floor());
	// if (draw_debug_shapes) {
	// 	debug_drawer->scoped_config()->set_thickness(0.006)->set_center_brightness(0.8);
	// 	debug_drawer->draw_box(grid_to_pos(origin_grid), Quaternion(), Vector3(size_grid) * sample_spacing, Color(0, 0, 0.545));
	// }
	TypedArray<int> samples = TypedArray<int>();
	for (size_t y = 0; y <= size_grid.y; y++) {
		for (size_t z = 0; z <= size_grid.z; z++) {
			for (size_t x = 0; x <= size_grid.x; x++) {
				Vector3i grid_pos = origin_grid + Vector3i(x, y, z);
				Vector3 world_pos = grid_to_pos(grid_pos);
				Vector3 box_pos = box_origin + p_basis.xform_inv(world_pos - box_origin);
				// if (draw_debug_shapes) {
				// 	debug_drawer->scoped_config()->set_thickness(0.004)->set_center_brightness(0.8);
				// 	if (draw_in_bounds_space) {
				// 		debug_drawer->draw_sphere(box_pos, 0.03, Color(0, 1, 0));
				// 	} else {
				// 		debug_drawer->draw_sphere(world_pos, 0.05, Color(0.1333, 0.545, 0.1333));
				// 	}
				// }
				if (!is_in_grid_bounds(grid_pos) || !box_bounds.has_point(box_pos)) {
					continue;
				}
				int index = grid_pos_to_idx_v(grid_pos);
				if (!samples.has(index) && sample_grid[index]) {
					samples.push_back(index);
				}
			}
		}
	}

	return samples;
}

float HVACField::get_average_temp(TypedArray<int> p_sample_indices) {
	ERR_FAIL_NULL_V(sim_parameters, 0.0f);
	ERR_FAIL_COND_V_MSG(samples.is_empty() || p_sample_indices.is_empty(), sim_parameters->ambient_temperature, "Can't get average temperature of empty sample array!");
	float sum = 0;
	int n_cnt = 0;
	for (size_t i = 0; i < p_sample_indices.size(); i++) {
		int idx = p_sample_indices[i];
		ERR_FAIL_INDEX_V(idx, max_sample_idx, 0.0f);
		Ref<HVACFieldSample> sample = sample_grid[idx];
		if (sample.is_null()) {
			continue;
		}
		sum += sample->temperature;
		n_cnt++;
	}

	if (n_cnt > 0) {
		return sum / n_cnt;
	} else {
		return sim_parameters->ambient_temperature;
	}
}

Vector3i HVACField::pos_to_grid(Vector3 p_position) {
	return Vector3i((inv_bounds_transform.xform(p_position) / sample_spacing + grid_half_size).round());
}

Vector3 HVACField::pos_to_grid_unrounded(Vector3 p_position) {
	return inv_bounds_transform.xform(p_position) / sample_spacing + grid_half_size;
}

Vector3 HVACField::grid_to_pos(Vector3i p_grid_pos) {
	return bounds_transform.xform((Vector3(p_grid_pos) - grid_half_size) * sample_spacing);
}

Vector3 HVACField::unrounded_grid_to_pos(Vector3 p_grid_pos_unrounded) {
	return bounds_transform.xform((p_grid_pos_unrounded - grid_half_size) * sample_spacing);
}

bool HVACField::is_in_grid_bounds(Vector3i p_grid_position) {
	return p_grid_position.x > 0 &&
			p_grid_position.y > 0 &&
			p_grid_position.z > 0 &&
			p_grid_position.x < grid_size.x &&
			p_grid_position.y < grid_size.y &&
			p_grid_position.z < grid_size.z;
}

// Getter/setter spam :)

void HVACField::set_grid_size(Vector3i p_grid_size) {
	grid_size = p_grid_size;
	grid_half_size = Vector3(grid_size) * 0.5f;
}
Vector3i HVACField::get_grid_size() const {
	return grid_size;
}

void HVACField::set_sample_spacing(Vector3 p_sample_spacing) {
	sample_spacing = p_sample_spacing;
}

Vector3 HVACField::get_sample_spacing() const {
	return sample_spacing;
}

void HVACField::set_samples(TypedArray<HVACFieldSample> p_samples) {
	samples = p_samples;
}
TypedArray<HVACFieldSample> HVACField::get_samples() const {
	return samples;
}

void HVACField::set_sample_grid(TypedArray<HVACFieldSample> p_sample_grid) {
	sample_grid = p_sample_grid;
	max_sample_idx = sample_grid.size();
}
TypedArray<HVACFieldSample> HVACField::get_sample_grid() const {
	return sample_grid;
}

void HVACField::set_index_offsets_map(TypedArray<int> p_index_offsets_map) {
	index_offsets_map = p_index_offsets_map;
}
TypedArray<int> HVACField::get_index_offsets_map() const {
	return index_offsets_map;
}

void HVACField::set_bounds_basis_axis_map(TypedArray<Vector3> p_bounds_basis_axis_map) {
	bounds_basis_axis_map = p_bounds_basis_axis_map;
}
TypedArray<Vector3> HVACField::get_bounds_basis_axis_map() const {
	return bounds_basis_axis_map;
}

void HVACField::set_sample_distance_map(TypedArray<float> p_sample_distance_map) {
	sample_distance_map = p_sample_distance_map;
}
TypedArray<float> HVACField::get_sample_distance_map() const {
	return sample_distance_map;
}

void HVACField::set_sim_parameters(Ref<HVACSimParameters> p_sim_parameters) {
	sim_parameters = p_sim_parameters;
}

Ref<HVACSimParameters> HVACField::get_sim_parameters() const {
	return sim_parameters;
}

void HVACField::set_bounds_transform(Transform3D p_bounds_transform) {
	bounds_transform = p_bounds_transform;
	inv_bounds_transform = bounds_transform.affine_inverse();
}

Transform3D HVACField::get_bounds_transform() const {
	return bounds_transform;
}

void HVACField::set_draw_debug_shapes(bool p_draw_debug_shapes) {
	draw_debug_shapes = p_draw_debug_shapes;
}

bool HVACField::get_draw_debug_shapes() const {
	return draw_debug_shapes;
}

void HVACField::set_draw_in_bounds_space(bool p_draw_in_bounds_space) {
	draw_in_bounds_space = p_draw_in_bounds_space;
}

bool HVACField::get_draw_in_bounds_space() const {
	return draw_in_bounds_space;
}

// HVACField::HVACField() {
// 	debug_drawer = memnew(DebugDraw3D);
// }

// HVACField::~HVACField() {
// 	memdelete(debug_drawer);
// }

void HVACField::_bind_methods() {
	// Useful methods
	godot::ClassDB::bind_method(D_METHOD("grid_pos_to_idx_v", "grid_pos"), &HVACField::grid_pos_to_idx_v);
	godot::ClassDB::bind_method(D_METHOD("grid_pos_to_idx", "x", "y", "z"), &HVACField::grid_pos_to_idx);
	godot::ClassDB::bind_method(D_METHOD("pos_to_idx", "pos"), &HVACField::pos_to_idx);

	godot::ClassDB::bind_method(D_METHOD("generate_field", "bounds_transform", "bounds_size", "statics_collision_mask", "physics_space_state", "min_temp_variance", "max_temp_variance"), &HVACField::generate_field);
	godot::ClassDB::bind_method(D_METHOD("propagate_air_samples", "delta"), &HVACField::propagate_air_samples);
	godot::ClassDB::bind_method(D_METHOD("propagate_heat_container_to_box", "delta", "heat_container", "box_center", "box_size", "box_basis", "distribute_evenly"), &HVACField::propagate_heat_container_to_box);

	godot::ClassDB::bind_method(D_METHOD("blend_samples_to", "sample_indices", "temperature", "blend_amount"), &HVACField::blend_samples_to);
	godot::ClassDB::bind_method(D_METHOD("blend_samples_with", "sample_indices", "heat_container", "blend_amount", "ignore_sample_count"), &HVACField::blend_samples_with);

	godot::ClassDB::bind_method(D_METHOD("get_sample_at", "pos"), &HVACField::get_sample_at);
	godot::ClassDB::bind_method(D_METHOD("get_grid_bounding_aabb", "center", "bounds", "basis"), &HVACField::get_grid_bounding_aabb);
	godot::ClassDB::bind_method(D_METHOD("get_grid_indices_in_box", "center", "size", "basis"), &HVACField::get_grid_indices_in_box);
	godot::ClassDB::bind_method(D_METHOD("get_average_temp", "sample_indices"), &HVACField::get_average_temp);

	godot::ClassDB::bind_method(D_METHOD("pos_to_grid", "pos"), &HVACField::pos_to_grid);
	godot::ClassDB::bind_method(D_METHOD("pos_to_grid_unrounded", "pos"), &HVACField::pos_to_grid_unrounded);
	godot::ClassDB::bind_method(D_METHOD("grid_to_pos", "grid_pos"), &HVACField::grid_to_pos);
	godot::ClassDB::bind_method(D_METHOD("unrounded_grid_to_pos", "grid_pos_unrounded"), &HVACField::unrounded_grid_to_pos);

	godot::ClassDB::bind_method(D_METHOD("is_in_grid_bounds", "grid_pos"), &HVACField::is_in_grid_bounds);

	// Getter/setter spam
	godot::ClassDB::bind_method(D_METHOD("set_grid_size", "grid_size"), &HVACField::set_grid_size);
	godot::ClassDB::bind_method(D_METHOD("get_grid_size"), &HVACField::get_grid_size);
	godot::ClassDB::bind_method(D_METHOD("set_samples", "samples"), &HVACField::set_samples);
	godot::ClassDB::bind_method(D_METHOD("get_samples"), &HVACField::get_samples);
	godot::ClassDB::bind_method(D_METHOD("set_sample_grid", "sample_grid"), &HVACField::set_sample_grid);
	godot::ClassDB::bind_method(D_METHOD("get_sample_grid"), &HVACField::get_sample_grid);
	godot::ClassDB::bind_method(D_METHOD("set_index_offsets_map", "index_offsets_map"), &HVACField::set_index_offsets_map);
	godot::ClassDB::bind_method(D_METHOD("get_index_offsets_map"), &HVACField::get_index_offsets_map);
	godot::ClassDB::bind_method(D_METHOD("set_bounds_basis_axis_map", "bounds_basis_axis_map"), &HVACField::set_bounds_basis_axis_map);
	godot::ClassDB::bind_method(D_METHOD("get_bounds_basis_axis_map"), &HVACField::get_bounds_basis_axis_map);
	godot::ClassDB::bind_method(D_METHOD("set_sample_distance_map", "sample_distance_map"), &HVACField::set_sample_distance_map);
	godot::ClassDB::bind_method(D_METHOD("get_sample_distance_map"), &HVACField::get_sample_distance_map);
	godot::ClassDB::bind_method(D_METHOD("set_sample_spacing", "sample_spacing"), &HVACField::set_sample_spacing);
	godot::ClassDB::bind_method(D_METHOD("get_sample_spacing"), &HVACField::get_sample_spacing);
	godot::ClassDB::bind_method(D_METHOD("set_sim_parameters", "sim_parameters"), &HVACField::set_sim_parameters);
	godot::ClassDB::bind_method(D_METHOD("get_sim_parameters"), &HVACField::get_sim_parameters);
	godot::ClassDB::bind_method(D_METHOD("set_bounds_transform", "bounds_transform"), &HVACField::set_bounds_transform);
	godot::ClassDB::bind_method(D_METHOD("get_bounds_transform"), &HVACField::get_bounds_transform);
	godot::ClassDB::bind_method(D_METHOD("set_draw_debug_shapes", "draw_debug_shapes"), &HVACField::set_draw_debug_shapes);
	godot::ClassDB::bind_method(D_METHOD("get_draw_debug_shapes"), &HVACField::get_draw_debug_shapes);
	godot::ClassDB::bind_method(D_METHOD("set_draw_in_bounds_space", "draw_in_bounds_space"), &HVACField::set_draw_in_bounds_space);
	godot::ClassDB::bind_method(D_METHOD("get_draw_in_bounds_space"), &HVACField::get_draw_in_bounds_space);

	// Properties
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "sample_spacing"), "set_sample_spacing", "get_sample_spacing");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "air_sample_mass"), "set_air_sample_mass", "get_air_sample_mass");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "sim_parameters", PROPERTY_HINT_RESOURCE_TYPE, "HVACSimParameters"), "set_sim_parameters", "get_sim_parameters");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "draw_debug_shapes"), "set_draw_debug_shapes", "get_draw_debug_shapes");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "draw_in_bounds_space"), "set_draw_in_bounds_space", "get_draw_in_bounds_space");
	ADD_GROUP("Readouts", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3I, "grid_size", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_grid_size", "get_grid_size");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "samples", PROPERTY_HINT_TYPE_STRING, "24/17:HVACFieldSample", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_samples", "get_samples");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "sample_grid", PROPERTY_HINT_TYPE_STRING, "24/17:HVACFieldSample", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_sample_grid", "get_sample_grid");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "index_offsets_map", PROPERTY_HINT_ARRAY_TYPE, "int", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_index_offsets_map", "get_index_offsets_map");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "bounds_basis_axis_map", PROPERTY_HINT_ARRAY_TYPE, "Vector3", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_bounds_basis_axis_map", "get_bounds_basis_axis_map");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "sample_distance_map", PROPERTY_HINT_ARRAY_TYPE, "float", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_sample_distance_map", "get_sample_distance_map");
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM3D, "bounds_transform", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_bounds_transform", "get_bounds_transform");
}
