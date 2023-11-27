@tool

extends Node3D

@export var raycast_count: int = 1
@export var raycast_range: float = 10.0
@export var create_raycasts: bool = false


func _process(delta):
	if create_raycasts:
		print("creating raycasts")
		create_raycasts = false
		
		for n in self.get_children():
			n.queue_free()
		
		for i in range(raycast_count):
			var new_cast = RayCast3D.new()
			var theta = 2*PI/raycast_count * i
			new_cast.target_position.x = cos(theta) * raycast_range + self.transform.origin.x
			new_cast.target_position.z = sin(theta) * raycast_range + self.transform.origin.y
			#new_cast.target_position.x = raycast_range
			new_cast.target_position.y = 0
			#new_cast.rotation.y = theta
#			new_cast.debug_shape_thickness = 1
			print(new_cast.target_position)
			add_child(new_cast)
			new_cast.owner = get_tree().edited_scene_root
	
