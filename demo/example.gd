extends Node


func _ready() -> void:
	var example := HVACField.new()
	example.print_type(example)
