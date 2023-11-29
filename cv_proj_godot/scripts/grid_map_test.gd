extends MultiMeshInstance2D

@onready var icon: MeshInstance2D = $'../Icon'

@export var grid_x: int = 5
@export var grid_y: int = 5
@export var tile_size: int = 10

func _ready():
	#multimesh = MultiMesh.new()
	var multimesh = get_multimesh()
	multimesh.transform_format = MultiMesh.TRANSFORM_2D
	multimesh.instance_count = 10
	multimesh.visible_instance_count = 10
	multimesh.use_colors = true
	#multimesh.mesh = icon.mesh
	
	for i in multimesh.visible_instance_count:
		multimesh.set_instance_transform(i, Transform2D(PI/4.0, Vector2(i*tile_size,i*tile_size)))

	set_multimesh(multimesh)
