#include "hvac_field.h"

int HVACField::grid_pos_to_idx_v(Vector3i p_pos) {
	return (p_pos.y * grid_size.x * grid_size.z) + (p_pos.z * grid_size.x) + p_pos.x;
}
int HVACField::grid_pos_to_idx(int x, int y, int z) {
	return (y * grid_size.x * grid_size.z) + (z * grid_size.x) + x;
}

void HVACField::generate_field(Transform3D p_bounds_transform, Vector3 p_bounds_size) {
	/*
	print_rich("[u]Generating HVAC Field[/u]")

	var query := PhysicsPointQueryParameters3D.new()
	query.collision_mask = statics_collision_mask
	var n_query := PhysicsRayQueryParameters3D.create(Vector3.ZERO, Vector3.ZERO, statics_collision_mask)
	var up_query := PhysicsRayQueryParameters3D.create(Vector3.ZERO, Vector3.ZERO, statics_collision_mask)
	var cur_sample : HVACFieldSample

	var bounds_basis = bounds.global_basis
	dists = [field_spacing.y, field_spacing.y, field_spacing.z, field_spacing.z, field_spacing.x, field_spacing.x]
	dirs = [bounds_basis.y, -bounds_basis.y, bounds_basis.z, -bounds_basis.z, bounds_basis.x, -bounds_basis.x]

	var im := field_mesh.mesh as ImmediateMesh
	var n := field_neighbor_mesh.mesh as ImmediateMesh
	var draw_mesh := field_mesh.is_visible_in_tree()
	var draw_neighbor_mesh := field_neighbor_mesh.is_visible_in_tree()
	if draw_mesh:
		im.clear_surfaces()
		im.surface_begin(Mesh.PRIMITIVE_POINTS)
	if draw_neighbor_mesh:
		n.clear_surfaces()
		n.surface_begin(Mesh.PRIMITIVE_POINTS)

	bounds_box = bounds.shape as BoxShape3D
	field_grid_size = (bounds_box.size / field_spacing).ceil()
	field_index_offsets = [field_grid_size.x * field_grid_size.z, -field_grid_size.x * field_grid_size.z, field_grid_size.x, -field_grid_size.x, 1, -1]
	print_rich("\tfield size: " + str(field_grid_size))

	field_samples.clear()
	field_grid.clear()
	field_grid.resize(field_grid_size.x * field_grid_size.y * field_grid_size.z)
	for y in range(field_grid_size.y):
		for z in range(field_grid_size.z):
			for x in range(field_grid_size.x):
#Make sure sample isn't inside a surface
				query.position = grid_to_world_pos(Vector3i(x, y, z))
				if get_world_3d().direct_space_state.intersect_point(query):
#print("sample " + str(query.position) + " inside surface " + result[0]["collider"].get_parent().name + ", ignoring")
					continue
#Make sure sample is indoors by checking for a roof above, a floor below, and a wall in each cardinal direction
				up_query.from = query.position
				up_query.to = up_query.from + Vector3.UP * 15
				if not get_world_3d().direct_space_state.intersect_ray(up_query):
#print("sample " + str(query.position) + " outdoors, ignoring")
					continue
				up_query.to = up_query.from + Vector3.DOWN * 15
				if not get_world_3d().direct_space_state.intersect_ray(up_query):
#print("sample " + str(query.position) + " outdoors, ignoring")
					continue
				var wall_missed : bool = false
				for i in range(6):
					up_query.to = up_query.from + dirs[i] * 60
					if not get_world_3d().direct_space_state.intersect_ray(up_query):
						wall_missed = true
						break
					up_query.to += Vector3.UP * 15
					if not get_world_3d().direct_space_state.intersect_ray(up_query):
						wall_missed = true
						break
				if wall_missed:
					continue
#Generate / save sample data
				cur_sample = HVACFieldSample.new()
				cur_sample.grid_index = grid_pos_to_idx(x, y, z)
				cur_sample.position = query.position
				cur_sample.temperature = randf_range(min_temp, max_temp) if seed_noise else ambient_temp
				field_samples.push_back(cur_sample)
				field_grid[cur_sample.grid_index] = cur_sample
				if draw_mesh:
					im.surface_set_color(color_gradient.sample(remap(cur_sample.temperature, min_temp, max_temp, 0, 1)))
					im.surface_add_vertex(query.position)
#Cast rays to see if neighboring samples are obscured
#TODO : perform this step later to avoid up query and halve neighbor checks ?
				n_query.from = query.position
				for i in range(6):
					n_query.to = query.position + dirs[i] * dists[i]
					up_query.from = n_query.to
					up_query.to = up_query.from + Vector3.UP * 15
					if get_world_3d().direct_space_state.intersect_ray(n_query) or \
							not get_world_3d().direct_space_state.intersect_ray(up_query):
						continue
					cur_sample.neighbors_valid |= 1 << i
					if draw_neighbor_mesh:
						n.surface_set_color(cols[i])
						n.surface_add_vertex(query.position + dirs[i] * dists[i] * 0.05)
#else:
#n.surface_set_color(Color.BLACK)
	if draw_mesh:
		im.surface_end()
	if draw_neighbor_mesh:
		n.surface_end()

	print_rich("[u]Finished Generating HVAC Field[/u]")
	*/
}

void HVACField::propagate_heat_elements(float p_delta) {
	/*
	for element in HeatElement.all_heat_elements:
		if not element.enabled or not element.is_inside_tree():
			continue

		var blob := element.scan_blob
		var affected_samples : Array[int] = []
		if blob is ScanBlobPoint:
			var element_radius : int = ceili(blob.radius)
			var samples := get_samples_in_box(
				element.global_position,
				Vector3.ONE * element_radius * 2,
				element.global_basis
			)
			affected_samples.append_array(samples)
		elif blob is ScanBlobLine:
			var blob_delta : Vector3 = blob.end_point.global_position - blob.start_point.global_position
			var element_length : int = ceili(blob_delta.length())
			var element_radius : int = ceili(blob.radius)
			var look_basis := Basis.looking_at(-blob_delta)
			var samples := get_samples_in_box(
				(blob.start_point.global_position + blob.end_point.global_position) * 0.5,
				Vector3(element_radius * 2, element_radius * 2, element_length),
				look_basis
			)
			affected_samples.append_array(samples)

		if affected_samples.is_empty():
			continue

		var count_factor := float(affected_samples.size()) if element.distribute_across_surface else 1.0
		var blend_amount := clampf(
			delta * heat_element_propagation_speed * efficiency / count_factor,
			0.0,
			1.0
		)
		var mass_ratio := air_sample_mass / element.heat_contents.mass
		var element_temp := element.heat_contents.temperature
		for sample in affected_samples:
			var nearest_sample := field_grid[sample]
			if nearest_sample:
				if draw_debug_shapes:
					DebugDraw3D.draw_sphere(nearest_sample.position, 0.08, Color.HOT_PINK)
				element.heat_contents.blend_to_temp(
					nearest_sample.temperature,
					blend_amount * mass_ratio,
					false
				)
				nearest_sample.temperature = lerpf(
					nearest_sample.temperature,
					element_temp,
					clampf(blend_amount / mass_ratio, 0, 1)
				)*/
}

void HVACField::propagate_air_samples(float p_delta) {
	/*var air_propagation : float = delta * air_propagation_speed * efficiency
	var cool_speed : float = delta * cool_rate * efficiency
	var sample_count : int = field_samples.size()
	var sample_update_count : int = min(air_sample_propagation_frame_limit, sample_count)
	for index in range(sample_update_count):
		var sample := field_samples[(air_sample_iterator + index) % sample_count]
		var neighbor_avg : float = 0
		var n_cnt : int = 0
		for i in range(6):
			if (sample.neighbors_valid & (1 << i)) > 0:
				var n_sample := field_grid[sample.grid_index + field_index_offsets[i]]
				if n_sample:
					neighbor_avg += n_sample.temperature
					n_cnt += 1
		if n_cnt > 0:
			neighbor_avg /= n_cnt
			for i in range(6):
				if (sample.neighbors_valid & (1 << i)) > 0:
					var n_sample := field_grid[sample.grid_index + field_index_offsets[i]]
					if n_sample:
						n_sample.temperature = lerpf(n_sample.temperature, neighbor_avg, air_propagation)
			sample.temperature = lerpf(sample.temperature, neighbor_avg, air_propagation)
		if cooling:
			sample.temperature = lerpf(sample.temperature, ambient_temp, cool_speed)
	air_sample_iterator += sample_update_count*/
}

void HVACField::blend_samples_to(TypedArray<int> p_sample_indices, float p_temperature, float p_blend_amount) {
	for (size_t i = 0; i < p_sample_indices.size(); i++) {
		HVACFieldSample *sample = cast_to<HVACFieldSample>(sample_grid[p_sample_indices[i]]);
		if (sample == nullptr) {
			continue;
		}
		sample->blend_to_temperature(p_temperature, p_blend_amount);
	}
}

void HVACField::blend_samples_with(TypedArray<int> p_sample_indices, HeatContainer *p_heat_container, float p_blend_amount) {
	float temperature_cache = p_heat_container->temperature;
	float mass_ratio = air_sample_mass / p_heat_container->mass;
	float average_sample_temperature = get_average_temp(p_sample_indices);
	blend_samples_to(p_sample_indices, temperature_cache, p_blend_amount * mass_ratio);
	p_heat_container->blend_to_temperature(average_sample_temperature, p_blend_amount / mass_ratio, true);
}

HVACFieldSample HVACField::get_sample_at(Vector3 p_position) {
	return HVACFieldSample();
}

TypedArray<int> HVACField::get_grid_indices_in_box(Vector3 p_center, Vector3 p_size, Basis p_basis) {
	AABB box_bounds = AABB(p_center - p_basis * p_size * 0.5, p_size);
	// if draw_debug_shapes:
	// 	DebugDraw3D.scoped_config().set_thickness(0.004).set_center_brightness(0.8)
	// 	if draw_bounds_space:
	// 		DebugDraw3D.draw_box(box_bounds.position, Quaternion.IDENTITY, box_bounds.size, Color.GREEN)
	// 	else:
	// 		DebugDraw3D.draw_box(box_bounds.position, box_basis.get_rotation_quaternion(), box_bounds.size, Color.AQUAMARINE)
	TypedArray<int> samples;
	Vector3 grid_center = world_to_grid_pos_unrounded(p_center);
	// if draw_debug_shapes and draw_center:
	// 	DebugDraw3D.draw_sphere(pos, 0.03, Color.CRIMSON)
	// 	DebugDraw3D.draw_sphere(grid_to_world_pos_unrounded(grid_center), 0.08, Color.RED)
	AABB grid_aligned_bounds = AABB(grid_center, Vector3(0.0f, 0.0f, 0.0f));
	for (size_t i = 0; i < 8; i++) {
		var corner_pos_world := box_bounds.position + box_basis * (box_bounds.get_endpoint(i) - box_bounds.position)
		var corner_pos_grid := Vector3i(world_to_grid_pos_unrounded(corner_pos_world).floor())
		if draw_debug_shapes:
			DebugDraw3D.draw_sphere(corner_pos_world, 0.05, Color.BLANCHED_ALMOND)
			DebugDraw3D.draw_sphere(grid_to_world_pos(corner_pos_grid), 0.05, Color.BLUE_VIOLET)
		for y in range(0, 2):
			for z in range(0, 2):
				for x in range(0, 2):
					grid_aligned_bounds = grid_aligned_bounds.expand(corner_pos_grid + Vector3i(x, y, z))
	}

	var size_grid := Vector3i(grid_aligned_bounds.size.ceil())
	var origin_grid := Vector3i(grid_aligned_bounds.position.floor())
	if draw_debug_shapes:
		DebugDraw3D.scoped_config().set_thickness(0.006).set_center_brightness(0.8)
		DebugDraw3D.draw_box(grid_to_world_pos(origin_grid), Quaternion.IDENTITY, Vector3(size_grid) * field_spacing, Color.DARK_BLUE)
	for y in range(0, size_grid.y + 1):
		for z in range(0, size_grid.z + 1):
			for x in range(0, size_grid.x + 1):
				var grid_pos := origin_grid + Vector3i(x, y, z)
				var world_pos := grid_to_world_pos(grid_pos)
				var box_pos := box_bounds.position + box_basis.inverse() * (world_pos - box_bounds.position)# - (box_bounds.position - box_center)
				if draw_debug_shapes:
					if draw_bounds_space:
						DebugDraw3D.draw_sphere(box_pos, 0.03, Color.GREEN)
					else:
						DebugDraw3D.draw_sphere(world_pos, 0.05, Color.FOREST_GREEN)
#print("world pos: " + str(element.global_position + element.global_basis * (Vector3(x, y, z) - Vector3.ONE * element_radius)))
#print("grid pos: " + str(grid_pos))
				if not is_in_grid_bounds(grid_pos) or not box_bounds.has_point(box_pos):
					continue
#print("index: " + str(grid_pos_to_idx(grid_pos)))
				var index := grid_pos_to_idx_v(grid_pos)
				if index not in samples and field_grid[index]:
					samples.push_back(index)
	return samples
	return TypedArray<int>();
}

float HVACField::get_average_temp(TypedArray<int> p_sample_indices) {
	return 0.0f;
}

Vector3i HVACField::pos_to_grid(Vector3 p_position) {
	return Vector3i();
}

Vector3 HVACField::pos_to_grid_unrounded(Vector3 p_position) {
	return Vector3();
}

Vector3 HVACField::grid_to_pos(Vector3i p_grid_pos) {
	return Vector3();
}

Vector3 HVACField::unrounded_grid_to_pos(Vector3 p_grid_pos_unrounded) {
	return Vector3();
}

bool HVACField::is_in_grid_bounds(Vector3i p_grid_position) {
	return false;
}

// Getter/setter spam :)

void HVACField::set_grid_size(Vector3i p_grid_size) {
	grid_size = p_grid_size;
}
Vector3i HVACField::get_grid_size() const {
	return grid_size;
}

void HVACField::set_samples(TypedArray<HVACFieldSample> p_samples) {
	samples = p_samples;
}
TypedArray<HVACFieldSample> HVACField::get_samples() const {
	return samples;
}

void HVACField::set_sample_grid(TypedArray<HVACFieldSample> p_sample_grid) {
	sample_grid = p_sample_grid;
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

void HVACField::set_air_sample_mass(float p_air_sample_mass) {
	air_sample_mass = p_air_sample_mass;
}

float HVACField::get_air_sample_mass() const {
	return air_sample_mass;
}

void HVACField::_bind_methods() {
	// Useful methods
	godot::ClassDB::bind_method(D_METHOD("grid_pos_to_idx_v"), &HVACField::grid_pos_to_idx_v);
	godot::ClassDB::bind_method(D_METHOD("grid_pos_to_idx"), &HVACField::grid_pos_to_idx);
	godot::ClassDB::bind_method(D_METHOD("generate_field"), &HVACField::generate_field);
	godot::ClassDB::bind_method(D_METHOD("propagate_heat_elements"), &HVACField::propagate_heat_elements);
	godot::ClassDB::bind_method(D_METHOD("propagate_air_samples"), &HVACField::propagate_air_samples);
	godot::ClassDB::bind_method(D_METHOD("blend_samples_to"), &HVACField::blend_samples_to);
	godot::ClassDB::bind_method(D_METHOD("blend_samples_with"), &HVACField::blend_samples_with);
	godot::ClassDB::bind_method(D_METHOD("get_sample_at"), &HVACField::get_sample_at);
	godot::ClassDB::bind_method(D_METHOD("get_grid_indices_in_box"), &HVACField::get_grid_indices_in_box);
	godot::ClassDB::bind_method(D_METHOD("get_average_temp"), &HVACField::get_average_temp);
	godot::ClassDB::bind_method(D_METHOD("pos_to_grid"), &HVACField::pos_to_grid);
	godot::ClassDB::bind_method(D_METHOD("pos_to_grid_unrounded"), &HVACField::pos_to_grid_unrounded);
	godot::ClassDB::bind_method(D_METHOD("grid_to_pos"), &HVACField::grid_to_pos);
	godot::ClassDB::bind_method(D_METHOD("unrounded_grid_to_pos"), &HVACField::unrounded_grid_to_pos);
	godot::ClassDB::bind_method(D_METHOD("is_in_grid_bounds"), &HVACField::is_in_grid_bounds);

	// Getter/setter spam
	godot::ClassDB::bind_method(D_METHOD("set_grid_size", "p_grid_size"), &HVACField::set_grid_size);
	godot::ClassDB::bind_method(D_METHOD("get_grid_size"), &HVACField::get_grid_size);
	godot::ClassDB::bind_method(D_METHOD("set_samples", "p_samples"), &HVACField::set_samples);
	godot::ClassDB::bind_method(D_METHOD("get_samples"), &HVACField::get_samples);
	godot::ClassDB::bind_method(D_METHOD("set_sample_grid", "p_sample_grid"), &HVACField::set_sample_grid);
	godot::ClassDB::bind_method(D_METHOD("get_sample_grid"), &HVACField::get_sample_grid);
	godot::ClassDB::bind_method(D_METHOD("set_index_offsets_map", "p_index_offsets_map"), &HVACField::set_index_offsets_map);
	godot::ClassDB::bind_method(D_METHOD("get_index_offsets_map"), &HVACField::get_index_offsets_map);
	godot::ClassDB::bind_method(D_METHOD("set_bounds_basis_axis_map", "p_bounds_basis_axis_map"), &HVACField::set_bounds_basis_axis_map);
	godot::ClassDB::bind_method(D_METHOD("get_bounds_basis_axis_map"), &HVACField::get_bounds_basis_axis_map);
	godot::ClassDB::bind_method(D_METHOD("set_sample_distance_map", "p_sample_distance_map"), &HVACField::set_sample_distance_map);
	godot::ClassDB::bind_method(D_METHOD("get_sample_distance_map"), &HVACField::get_sample_distance_map);
	godot::ClassDB::bind_method(D_METHOD("set_air_sample_mass", "p_air_sample_mass"), &HVACField::set_air_sample_mass);
	godot::ClassDB::bind_method(D_METHOD("get_air_sample_mass"), &HVACField::get_air_sample_mass);

	// Properties
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "grid_size", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_grid_size", "get_grid_size");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "samples", PROPERTY_HINT_TYPE_STRING, "24/17:HVACFieldSample", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_samples", "get_samples");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "sample_grid", PROPERTY_HINT_TYPE_STRING, "24/17:HVACFieldSample", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_sample_grid", "get_sample_grid");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "index_offsets_map", PROPERTY_HINT_ARRAY_TYPE, "int", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_index_offsets_map", "get_index_offsets_map");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "bounds_basis_axis_map", PROPERTY_HINT_ARRAY_TYPE, "Vector3", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_bounds_basis_axis_map", "get_bounds_basis_axis_map");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "sample_distance_map", PROPERTY_HINT_ARRAY_TYPE, "float", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_sample_distance_map", "get_sample_distance_map");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "air_sample_mass", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY), "set_air_sample_mass", "get_air_sample_mass");
}

var air_sample_iterator : int = 0


func world_to_grid_pos(global_pos : Vector3) -> Vector3i:
	return world_to_grid_pos_unrounded(global_pos).round()
func world_to_grid_pos_unrounded(global_pos : Vector3) -> Vector3:
	var bounds_pos := bounds.to_local(global_pos)
	return bounds_pos / field_spacing + field_grid_size * 0.5
func grid_to_world_pos(grid_pos : Vector3i) -> Vector3:
	return grid_to_world_pos_unrounded(Vector3(grid_pos))
func grid_to_world_pos_unrounded(grid_pos_unrounded : Vector3) -> Vector3:
	var bounds_pos := (grid_pos_unrounded - field_grid_size * 0.5) * field_spacing
	return bounds.to_global(bounds_pos)

func is_in_grid_bounds(grid_pos : Vector3i) -> bool:
	return grid_pos == grid_pos.abs() and grid_pos.x < field_grid_size.x and grid_pos.y < field_grid_size.y and grid_pos.z < field_grid_size.z

func sample_point(pos : Vector3) -> HVACFieldSample:
	var grid_pos := world_to_grid_pos(pos)
	if is_in_grid_bounds(grid_pos):
		return field_grid[grid_pos_to_idx_v(grid_pos)]
	else:
		return null

const draw_center : bool = false
const draw_bounds_space : bool = false
func get_samples_in_box(pos : Vector3, size : Vector3, box_basis : Basis) -> Array[int]:
	var box_bounds := AABB(pos - box_basis * size * 0.5, size)
	if draw_debug_shapes:
		DebugDraw3D.scoped_config().set_thickness(0.004).set_center_brightness(0.8)
		if draw_bounds_space:
			DebugDraw3D.draw_box(box_bounds.position, Quaternion.IDENTITY, box_bounds.size, Color.GREEN)
		else:
			DebugDraw3D.draw_box(box_bounds.position, box_basis.get_rotation_quaternion(), box_bounds.size, Color.AQUAMARINE)
	var samples : Array[int]
	var box_center := pos
	var grid_center := world_to_grid_pos_unrounded(box_center)
	if draw_debug_shapes and draw_center:
		DebugDraw3D.draw_sphere(pos, 0.03, Color.CRIMSON)
		DebugDraw3D.draw_sphere(grid_to_world_pos_unrounded(grid_center), 0.08, Color.RED)
	var grid_aligned_bounds := AABB(grid_center, Vector3.ZERO)
	for i in range(8):
#var corner_pos_world : = box_basis * (box_bounds.get_endpoint(i) - box_bounds.position)
		var corner_pos_world := box_bounds.position + box_basis * (box_bounds.get_endpoint(i) - box_bounds.position)
		var corner_pos_grid := Vector3i(world_to_grid_pos_unrounded(corner_pos_world).floor())
		if draw_debug_shapes:
			DebugDraw3D.draw_sphere(corner_pos_world, 0.05, Color.BLANCHED_ALMOND)
			DebugDraw3D.draw_sphere(grid_to_world_pos(corner_pos_grid), 0.05, Color.BLUE_VIOLET)
		for y in range(0, 2):
			for z in range(0, 2):
				for x in range(0, 2):
					grid_aligned_bounds = grid_aligned_bounds.expand(corner_pos_grid + Vector3i(x, y, z))

	var size_grid := Vector3i(grid_aligned_bounds.size.ceil())
	var origin_grid := Vector3i(grid_aligned_bounds.position.floor())
	if draw_debug_shapes:
		DebugDraw3D.scoped_config().set_thickness(0.006).set_center_brightness(0.8)
		DebugDraw3D.draw_box(grid_to_world_pos(origin_grid), Quaternion.IDENTITY, Vector3(size_grid) * field_spacing, Color.DARK_BLUE)
	for y in range(0, size_grid.y + 1):
		for z in range(0, size_grid.z + 1):
			for x in range(0, size_grid.x + 1):
				var grid_pos := origin_grid + Vector3i(x, y, z)
				var world_pos := grid_to_world_pos(grid_pos)
				var box_pos := box_bounds.position + box_basis.inverse() * (world_pos - box_bounds.position)# - (box_bounds.position - box_center)
				if draw_debug_shapes:
					if draw_bounds_space:
						DebugDraw3D.draw_sphere(box_pos, 0.03, Color.GREEN)
					else:
						DebugDraw3D.draw_sphere(world_pos, 0.05, Color.FOREST_GREEN)
#print("world pos: " + str(element.global_position + element.global_basis * (Vector3(x, y, z) - Vector3.ONE * element_radius)))
#print("grid pos: " + str(grid_pos))
				if not is_in_grid_bounds(grid_pos) or not box_bounds.has_point(box_pos):
					continue
#print("index: " + str(grid_pos_to_idx(grid_pos)))
				var index := grid_pos_to_idx_v(grid_pos)
				if index not in samples and field_grid[index]:
					samples.push_back(index)
	return samples
